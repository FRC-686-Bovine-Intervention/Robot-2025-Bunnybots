package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;

public class Shooter extends SubsystemBase {
    private final ShooterIO io;
    private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from("PID",
        new PIDConstants(
            3.0,
            0.0,
            0.05
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from("FF",
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
    
    private static final LoggedTunableNumber mpsToRps = new LoggedTunableNumber("Shooter/MetersPerSecToRotsPerSec", 1);
    private static final LoggedTunable<AngularVelocity> maxVelocity = LoggedTunable.from("Shooter/MaxVelocity", RotationsPerSecond::of, 100);
    private static final LoggedTunable<AngularVelocity> idleVelocity = LoggedTunable.from("Shooter/IdleVelocity", RotationsPerSecond::of, 0);
    private static final LoggedTunable<Angle> shooterAngle = LoggedTunable.from("Shooter/ShooterAngle", Degrees::of, 45);

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

    public Command idle() {
        final var shooter = this;
        return new Command() {
            {
                this.addRequirements(shooter);
                this.setName("Idle");
            }

            @Override
            public void execute() {
                shooter.io.stop(NeutralMode.COAST);
            }
        };
    }

    public Command shoot(DoubleSupplier distanceToTarget, DoubleSupplier heightDifference){
        final var shooter = this;
        return new Command() {
            {
                this.addRequirements(shooter);
                this.setName("Shoot");
            }

            @Override
            public void execute() {
                var targetVelocity = findAngularVelocity(distanceToTarget.getAsDouble(), heightDifference.getAsDouble());
                var setpoint = motionProfile.calculate(Robot.defaultPeriodSecs, setpointState, new State(targetVelocity, 0.0));
                var ffOutVolts = ff.calculateWithVelocities(setpointState.velocity, setpoint.velocity);
                setpointState = setpoint;

                shooter.io.setAngularVelocity(targetVelocity, ffOutVolts);
            }
        };
    }

    public static double findAngularVelocity(double d, double h) {
        double theta = shooterAngle.get().in(Radians);
        double g = 9.81;
        double cosTheta = Math.cos(theta);
        double targetVelocity = Math.sqrt(
            (g * d * d) /
            (2 * cosTheta * cosTheta * (d * Math.tan(theta) - h))
        );
        return mpsToRps.getAsDouble() * targetVelocity;
    }
}
