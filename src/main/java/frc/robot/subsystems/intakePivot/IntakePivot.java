package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.angle.ArmMech;

public class IntakePivot extends SubsystemBase{
    private final IntakePivotIO io;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();

    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
        "IntakePivot/Profile",
        DegreesPerSecond,
        DegreesPerSecondPerSecond,
        RadiansPerSecond,
        RadiansPerSecondPerSecond,
        new TrapezoidProfile.Constraints(
            90,
            180
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
        "IntakePivot/FF",
        new FFConstants(
            0,
            0,
            17 /2/Math.PI,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "IntakePivot/PID",
        new PIDConstants(
            150,
            0,
            0
        )
    );

    private TrapezoidProfile motionProfile = new TrapezoidProfile(profileConsts.get());
    private final State measuredState = new State();
    private final State setpointState = new State();
    private final State goalState = new State();
    private boolean motionProfiling = false;
    private final ArmFeedforward feedforward = new ArmFeedforward(0, 0, 0, 0);

    private double angleRads = 0;
    private double velocityRadsPerSec = 0.0;

    public final ArmMech mech = new ArmMech(IntakePivotConstants.pivotBase);

    private final Alert motorDisconnectedAlert = new Alert("IntakePivot/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert encoderDisconnectedAlert = new Alert("IntakePivot/Alerts", "Encoder Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Pivot Motor Disconnected!", AlertType.kError);
    private final Alert encoderDisconnectedGlobalAlert = new Alert("Intake Pivot Encoder Disconnected!", AlertType.kError);

    public IntakePivot(IntakePivotIO io) {
        System.out.println("[Init IntakePivot] Instantiating IntakePivot with " + io.getClass().getSimpleName());
        this.io = io;

        ffConsts.get().update(this.feedforward);
        this.io.configPID(pidConsts.get());
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/IntakePivot/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/IntakePivot/Update Inputs");
        Logger.processInputs("Inputs/IntakePivot", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/IntakePivot/Process Inputs");

        this.angleRads = IntakePivotConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads());
        this.velocityRadsPerSec = IntakePivotConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

        this.measuredState.position = this.getAngleRads();
        this.measuredState.velocity = this.getVelocityRadsPerSec();

        this.mech.setRads(this.getAngleRads());

        Logger.recordOutput("IntakePivot/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("IntakePivot/Velocity/Measured", this.getVelocityRadsPerSec());

        if (profileConsts.hasChanged(hashCode())) {
            this.motionProfile = new TrapezoidProfile(profileConsts.get());
        }
        if (ffConsts.hasChanged(hashCode())) {
            ffConsts.get().update(this.feedforward);
        }
        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.get());
        }

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.encoderDisconnectedAlert.set(!this.inputs.encoderConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);
        this.encoderDisconnectedGlobalAlert.set(!this.inputs.encoderConnected);

        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/IntakePivot/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/IntakePivot");
    }

    public double getAngleRads() {
        return this.angleRads;
    }
    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }
    public double getAppliedVolts() {
        return this.inputs.motor.motor.getAppliedVolts();
    }

    public void setVolts(double volts) {
        this.motionProfiling = false;
        this.io.setVolts(volts);
    }

    public void stop(Optional<NeutralMode> neutralMode) {
        this.motionProfiling = false;
        this.io.stop(neutralMode);
    }

    private void setAngleGoalRads(double angleRads, TrapezoidProfile motionProfile) {
        this.goalState.position = angleRads;
        this.goalState.velocity = 0.0;
        if (!this.motionProfiling) {
            this.setpointState.position = this.measuredState.position;
            this.setpointState.velocity = this.measuredState.velocity;
            this.motionProfiling = true;
        }
        var newSetpointState = motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, this.goalState);
        var ffout = this.feedforward.calculateWithVelocities(this.setpointState.position, this.setpointState.velocity, newSetpointState.velocity);
        this.setpointState.position = newSetpointState.position;
        this.setpointState.velocity = newSetpointState.velocity;
        this.io.setPosition(
            this.setpointState.position,
            this.setpointState.velocity,
            ffout
        );
        Logger.recordOutput("Pivot/FF/FF Out", ffout);
        Logger.recordOutput("Pivot/Angle/Setpoint", this.setpointState.position);
        Logger.recordOutput("Pivot/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Pivot/Angle/Goal", this.goalState.position);
        Logger.recordOutput("Pivot/Velocity/Goal", this.goalState.velocity);
    }

    public void setAngleGoalRads(double angleRads) {
        this.setAngleGoalRads(angleRads, this.motionProfile);
    }

    public Command coast() {
        var subsystem = this;
        return new Command() {
            {
                setName("Coast");
                addRequirements(subsystem);
            }

            @Override
            public void initialize() {
                subsystem.stop(Optional.of(NeutralMode.Coast));
            }

            @Override
            public void end(boolean interrupted) {
                subsystem.stop(Optional.of(NeutralMode.Brake));
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    private Command genCommand(String name, DoubleSupplier angleRads) {
        var subsystem = this;
        return new Command() {
            {
                setName(name);
                addRequirements(subsystem);
            }

            @Override
            public void execute() {
                subsystem.setAngleGoalRads(angleRads.getAsDouble());
            }
        };
    }

    public Command idle() {
        return genCommand(
            "Idle", 
            () -> IntakePivotConstants.minAngle.in(Radians)
        );
    }

    public Command deploy() {
        return genCommand(
            "Deploy", 
            () -> IntakePivotConstants.maxAngle.in(Radians)
        );
    }
}
