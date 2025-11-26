package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Cooldown;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.misc.MathExtraUtil;

public class Flywheel extends SubsystemBase {
    private final FlywheelIO io;
    private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private static final LoggedTunable<LinearVelocity> preemptiveTargetSpeed = LoggedTunable.from("Shooter/Flywheel/Pre-emptive/Target Speed", MetersPerSecond::of, 17);
    private static final LoggedTunable<LinearVelocity> customTargetSpeed = LoggedTunable.from("Shooter/Flywheel/Custom/Target Speed", MetersPerSecond::of, 15);
    private static final LoggedTunable<LinearVelocity> customIncrement = LoggedTunable.from("Shooter/Flywheel/Custom/Increment", MetersPerSecond::of, 0.5);
    
    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from("Shooter/Flywheel/PID",
        new PIDConstants(
            0.0,
            0.0,
            0.0
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from("Shooter/Flywheel/FF",
        new FFConstants(
            0.0,
            0.0,
            0.0,
            0.0
        )
    );

    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
        ffConsts.get().kS(),
        ffConsts.get().kV(),
        ffConsts.get().kA()
    );
    
    private final Alert leftMotorDisconnectedAlert = new Alert("Shooter/Flywheel/Alerts", "Left Motor Disconnected", AlertType.kError);
    private final Alert rightMotorDisconnectedAlert = new Alert("Shooter/Flywheel/Alerts", "Right Motor Disconnected", AlertType.kError);
    private final Alert leftMotorDisconnectedGlobalAlert = new Alert("Flywheel Left Motor Disconnected!", AlertType.kError);
    private final Alert rightMotorDisconnectedGlobalAlert = new Alert("Flywheel Right Motor Disconnected!", AlertType.kError);

    private double averageSurfaceVeloMPS = 0.0;

    public Flywheel(FlywheelIO io) {
        super("Shooter/Flywheel");

        System.out.println("[Init Flywheel] Instantiating Flywheel with " + io.getClass().getSimpleName());
        this.io = io;
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Flywheel/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Flywheel/Update Inputs");
        Logger.processInputs("Inputs/Shooter/Flywheel", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Flywheel/Process Inputs");

        this.averageSurfaceVeloMPS = FlywheelConstants.flywheel.radiansToMeters(MathExtraUtil.average(this.inputs.leftMotor.encoder.getVelocityRadsPerSec(), this.inputs.rightMotor.encoder.getVelocityRadsPerSec()));

        Logger.recordOutput("Shooter/Flywheel/Average MPS", this.getAverageSurfaceVeloMPS());

        if (pidConsts.hasChanged(this.hashCode())) {
            this.io.configPID(pidConsts.get());
        }
        if (ffConsts.hasChanged(this.hashCode())) {
            ffConsts.get().update(this.ff);
        }

        this.leftMotorDisconnectedAlert.set(!this.inputs.leftMotorConnected);
        this.rightMotorDisconnectedAlert.set(!this.inputs.rightMotorConnected);
        this.leftMotorDisconnectedGlobalAlert.set(!this.inputs.leftMotorConnected);
        this.rightMotorDisconnectedGlobalAlert.set(!this.inputs.rightMotorConnected);
        
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Flywheel/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Flywheel");
    }

    public double getAverageSurfaceVeloMPS() {
        return this.averageSurfaceVeloMPS;
    }

    private void applySurfaceVeloMPS(double surfaceVeloMPS) {
        var motorSpeed = FlywheelConstants.motorToMechanism.inverse().applyUnsigned(FlywheelConstants.flywheel.metersToRadians(surfaceVeloMPS));
        Logger.recordOutput("Shooter/Flywheel/Goal Speed", motorSpeed);
        this.io.setVelocity(motorSpeed, this.ff.calculate(motorSpeed));
    }

    public Command genSurfaceVeloCommand(String name, DoubleSupplier targetSurfaceVeloMPS) {
        final var flywheel = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(flywheel);
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                flywheel.applySurfaceVeloMPS(targetSurfaceVeloMPS.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                flywheel.io.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command idle() {
        final var flywheel = this;
        return new Command() {
            {
                this.setName("Idle");
                this.addRequirements(flywheel);
            }

            @Override
            public void execute() {
                flywheel.io.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command custom() {
        return genSurfaceVeloCommand(
            "Custom",
            () -> customTargetSpeed.get().in(MetersPerSecond)
        );
    }
    public Command customIncrement(BooleanSupplier increase, BooleanSupplier decrease) {
        DoubleSupplier speed = new DoubleSupplier() {
            private double speed = customTargetSpeed.get().in(MetersPerSecond);
            private final Cooldown cooldown = new Cooldown();
            @Override
            public double getAsDouble() {
                Logger.recordOutput("Custom Shoot/Shooter Speed", speed);
                if(!cooldown.hasExpired()) {
                    return speed;
                }
                if(increase.getAsBoolean()) {
                    cooldown.reset(0.25);
                    speed += customIncrement.get().in(MetersPerSecond);
                }
                if(decrease.getAsBoolean()) {
                    cooldown.reset(0.25);
                    speed -= customIncrement.get().in(MetersPerSecond);
                }

                return speed;
            }
        };
        return genSurfaceVeloCommand(
            "Custom Increment",
            speed
        );
    }
}
