package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.AimingParameters;
import frc.util.Cooldown;
import frc.util.FFConstants;
import frc.util.LoggedInternalButton;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.misc.MathExtraUtil;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunable<LinearVelocity> preemptiveTargetSpeed = LoggedTunable.from("Shooter/Pre-emptive/Target Speed", MetersPerSecond::of, 17);
    private static final LoggedTunable<LinearVelocity> customTargetSpeed = LoggedTunable.from("Shooter/Custom/Target Speed", MetersPerSecond::of, 15);
    private static final LoggedTunable<LinearVelocity> customMinimumSpeed = LoggedTunable.from("Shooter/Custom/Minimum Speed", MetersPerSecond::of, 50);
    private static final LoggedTunable<LinearVelocity> customMaximumSpeed = LoggedTunable.from("Shooter/Custom/Maximum Speed", MetersPerSecond::of, 50);
    private static final LoggedTunable<LinearVelocity> customIncrement = LoggedTunable.from("Shooter/Custom/Increment", MetersPerSecond::of, 0.5);
    
    public final InternalButton readyToShoot = new LoggedInternalButton("Shooter/Ready to Shoot");
    public final InternalButton autoShootEnabled = new LoggedInternalButton("Shooter/AutoShoot Enabled");
    public final Trigger readyToAutoShoot = readyToShoot.and(autoShootEnabled);

    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from("Shooter/PID",
        new PIDConstants(
            3.0,
            0.0,
            0.05
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from("Shooter/FF",
        new FFConstants(
            0.0,
            0.0,
            0.0,
            0.0
        )
    );
    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConstraints = LoggedTunable.from("Profile",
        new Constraints(
            0.25 * 2 * Math.PI,
            0.5 * 2 * Math.PI
        )
    );
    
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(
        ffConsts.get().kS(),
        ffConsts.get().kV(),
        ffConsts.get().kA()
    );
    private TrapezoidProfile motionProfile = new TrapezoidProfile(profileConstraints.get());
    private State setpointState = new State();
    
    private final Alert leftMotorDisconnectedAlert = new Alert("Shooter/Alerts", "Left Motor Disconnected", AlertType.kError);
    private final Alert rightMotorDisconnectedAlert = new Alert("Shooter/Alerts", "Right Motor Disconnected", AlertType.kError);
    private final Alert leftMotorDisconnectedGlobalAlert = new Alert("Shooter Left Motor Disconnected!", AlertType.kError);
    private final Alert rightMotorDisconnectedGlobalAlert = new Alert("Shooter Right Motor Disconnected!", AlertType.kError);

    public Shooter(ShooterIO io) {
        System.out.println("[Init Shooter] Instantiating Shooter with " + io.getClass().getSimpleName());
        this.io = io;
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter/Before");
        io.updateInputs(inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter/Update Inputs");
        Logger.processInputs("Inputs/Shooter", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter/Process Inputs");
        Logger.recordOutput("Shooter/Average MPS", getAverageSurfaceSpeed());

        if (pidConsts.hasChanged(this.hashCode())) {
            io.configPID(pidConsts.get());
        }
        if (ffConsts.hasChanged(this.hashCode())) {
            ffConsts.get().update(ff);
        }
        if (profileConstraints.hasChanged(this.hashCode())) {
            motionProfile = new TrapezoidProfile(profileConstraints.get());
        }

        this.leftMotorDisconnectedAlert.set(!this.inputs.leftMotorConnected);
        this.rightMotorDisconnectedAlert.set(!this.inputs.rightMotorConnected);
        this.leftMotorDisconnectedGlobalAlert.set(!this.inputs.leftMotorConnected);
        this.rightMotorDisconnectedGlobalAlert.set(!this.inputs.rightMotorConnected);
        
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter");
    }

    public double getAverageSurfaceSpeed() {
        return ShooterConstants.flywheel.angularVelocityToLinearVelocity(MathExtraUtil.average(inputs.leftMotor.encoder.getVelocityRadsPerSec(), inputs.rightMotor.encoder.getVelocityRadsPerSec()));
    }

    private void applySurfaceSpeed(double surfaceSpeed) {
        var goalSpeed = surfaceSpeed * ShooterConstants.shooterSpeedEnvCoef.getAsDouble();
        var motorSpeed = ShooterConstants.flywheel.linearVelocityToAngularVelocity(goalSpeed) / ShooterConstants.motorToMechRatio.reductionUnsigned();
        Logger.recordOutput("Shooter/Goal Speed", motorSpeed);
        io.setVelocity(motorSpeed, ff.calculate(surfaceSpeed));
    }

    private void setReadyToShoot(double minimum, double maximum) {
        readyToShoot.setPressed(MathExtraUtil.isWithin(getAverageSurfaceSpeed(), minimum, maximum));
    }
    
    private Command genCommand(
        String name,
        DoubleSupplier targetSpeed,
        DoubleSupplier minimumSpeed,
        DoubleSupplier maximumSpeed,
        boolean enableAutoShoot
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                autoShootEnabled.setPressed(enableAutoShoot);
            }

            @Override
            public void execute() {
                applySurfaceSpeed(targetSpeed.getAsDouble());
                setReadyToShoot(minimumSpeed.getAsDouble(), maximumSpeed.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                readyToShoot.setPressed(false);
                autoShootEnabled.setPressed(false);
            }
        };
    }

    private Command genCommand(
        String name,
        LoggedTunable<LinearVelocity> targetSpeed,
        LoggedTunable<LinearVelocity> minimumSpeed,
        LoggedTunable<LinearVelocity> maximumSpeed,
        boolean enableAutoShoot
    ) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                autoShootEnabled.setPressed(enableAutoShoot);
            }

            @Override
            public void execute() {
                applySurfaceSpeed(targetSpeed.get().in(MetersPerSecond));
                setReadyToShoot(minimumSpeed.get().in(MetersPerSecond), maximumSpeed.get().in(MetersPerSecond));
            }

            @Override
            public void end(boolean interrupted) {
                readyToShoot.setPressed(false);
                autoShootEnabled.setPressed(false);
            }
        };
    }

    private static final double VelocityMax = MetersPerSecond.of(Double.POSITIVE_INFINITY).in(MetersPerSecond);
    public Command idle() {
        var subsystem = this;
        return new Command() {
            {
                setName("Idle");
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                io.stop(NeutralMode.COAST);
            }
        };
    }

    public Command aimWithAutoShoot() {
        return genCommand(
            "Aim-AutoShoot", 
            AimingParameters::targetShooterSpeed,
            AimingParameters::minimumShooterSpeed, 
            () -> VelocityMax,
            true
        );
    }
    
    public Command aimWithoutAutoShoot() {
        return genCommand(
            "Aim",
            AimingParameters::targetShooterSpeed,
            AimingParameters::minimumShooterSpeed,
            () -> VelocityMax,
            false
        );
    }
    public Command preemptive() {
        return genCommand(
            "Pre-emptive",
            () -> preemptiveTargetSpeed.get().in(MetersPerSecond),
            () -> 0,
            () -> VelocityMax,
            false
        );
    }
    public Command custom() {
        return genCommand(
            "Custom",
            customTargetSpeed,
            customMinimumSpeed,
            customMaximumSpeed,
            false
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
        return genCommand(
            "Custom Increment",
            speed,
            speed,
            speed,
            false
        );
    }
}
