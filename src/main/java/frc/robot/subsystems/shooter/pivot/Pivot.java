package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;
import frc.util.EdgeDetector;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.angle.ArmMech;

public class Pivot extends SubsystemBase {
    private final PivotIO io;
    private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();

    protected static final LoggedTunable<Angle> idleAngle = LoggedTunable.from("Shooter/Pivot/Idle Angle", Degrees::of, PivotConstants.minAngle.in(Degrees));
    protected static final LoggedTunable<Angle> rezeroThreshold = LoggedTunable.from("Shooter/Pivot/Rezero Threshold", Degrees::of, 2.0);
    
    private static final LoggedTunable<TrapezoidProfile.Constraints> profileConsts = LoggedTunable.fromDashboardUnits(
        "Shooter/Pivot/Profile",
        DegreesPerSecond,
        DegreesPerSecondPerSecond,
        RadiansPerSecond,
        RadiansPerSecondPerSecond,
        new TrapezoidProfile.Constraints(
            0.0,
            0.0
        )
    );
    private static final LoggedTunable<FFConstants> ffConsts = LoggedTunable.from(
        "Shooter/Pivot/FF",
        new FFConstants(
            0.0,
            0.0,
            0.0,
            0.0
        )
    );
    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Shooter/Pivot/PID",
        new PIDConstants(
            0.0,
            0.0,
            0.0
        )
    );

    private TrapezoidProfile motionProfile = new TrapezoidProfile(profileConsts.get());
    private final State measuredState = new State();
    private final State setpointState = new State();
    private final State goalState = new State();
    private boolean motionProfiling = false;
    private final ArmFeedforward feedforward = new ArmFeedforward(ffConsts.get().kS(), ffConsts.get().kG(), ffConsts.get().kV(), ffConsts.get().kA());
    
    private double angleRads = 0.0;
    private double velocityRadsPerSec = 0.0;
    private double motorOffsetRads = 0.0;
    private boolean calibrated = false;

    private final EdgeDetector limitSwitchEdgeDetector = new EdgeDetector(false);

    public final ArmMech mech = new ArmMech(PivotConstants.pivotBase);

    private final Alert notCalibratedAlert = new Alert("Shooter/Pivot/Alerts", "Not Calibrated", AlertType.kError);
    private final Alert notCalibratedGlobalAlert = new Alert("Pivot Not Calibrated!", AlertType.kError);

    private final Alert motorDisconnectedAlert = new Alert("Shooter/Pivot/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Pivot Motor Disconnected!", AlertType.kError);

    public Pivot(PivotIO io) {
        super("Shooter/Pivot");

        System.out.println("[Init Pivot] Instantiating Pivot with " + io.getClass().getSimpleName());
        this.io = io;

        ffConsts.get().update(this.feedforward);
        this.io.configPID(pidConsts.get());
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Pivot/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Pivot/Update Inputs");
        Logger.processInputs("Inputs/Shooter/Pivot", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Pivot/Process Inputs");

        this.limitSwitchEdgeDetector.update(this.inputs.limitSwitch);
        if (this.limitSwitchEdgeDetector.risingEdge()) {
            this.resetInternalAngleRads(PivotConstants.minAngle.in(Radians));
            if (!this.calibrated || Math.abs(this.motorOffsetRads) >= rezeroThreshold.get().in(Radians)) {
                this.io.resetMotorPositionRads(PivotConstants.motorToMechanism.inverse().applyUnsigned(this.angleRads));
                this.motorOffsetRads = 0.0;
            }
            this.calibrated = true;
        }

        this.angleRads = PivotConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads()) + this.motorOffsetRads;
        this.velocityRadsPerSec = PivotConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec());

        this.measuredState.position = this.getAngleRads();
        this.measuredState.velocity = this.getVelocityRadsPerSec();

        this.mech.setRads(this.getAngleRads());

        Logger.recordOutput("Shooter/Pivot/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("Shooter/Pivot/Velocity/Measured", this.getVelocityRadsPerSec());

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
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

        this.notCalibratedAlert.set(!this.calibrated);
        this.notCalibratedGlobalAlert.set(!this.calibrated);

        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Pivot/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Shooter Pivot");
    }

    private void resetInternalAngleRads(double angleRads) {
        this.angleRads = angleRads;
        this.motorOffsetRads = this.angleRads - PivotConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
    }

    // public static Transform3d getRobotToPivot(double angle) {
    //     return new Transform3d(
    //         PivotConstants.pivotBase.getTranslation(),
    //         new Rotation3d(
    //             0,
    //             PivotConstants.minAngle.in(Radians)-angle,
    //             0
    //         )
    //     );
    // }

    public double getAngleRads() {
        return this.angleRads;
    }
    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }

    private void stop(Optional<NeutralMode> neutralMode) {
        this.motionProfiling = false;
        this.io.stop(neutralMode);
    }

    private void setAngleGoalRads(double angleRads) {
        this.goalState.position = angleRads;
        this.goalState.velocity = 0.0;
        if (!this.motionProfiling) {
            this.setpointState.position = this.measuredState.position;
            this.setpointState.velocity = this.measuredState.velocity;
            this.motionProfiling = true;
        }
        var newSetpointState = this.motionProfile.calculate(RobotConstants.rioUpdatePeriodSecs, this.setpointState, this.goalState);
        var ffout = this.feedforward.calculateWithVelocities(this.setpointState.position, this.setpointState.velocity, newSetpointState.velocity);
        this.setpointState.position = newSetpointState.position;
        this.setpointState.velocity = newSetpointState.velocity;
        this.io.setPositionRads(
            PivotConstants.motorToMechanism.inverse().applyUnsigned(this.setpointState.position - this.motorOffsetRads),
            PivotConstants.motorToMechanism.inverse().applyUnsigned(this.setpointState.velocity),
            ffout
        );
        Logger.recordOutput("Shooter/Pivot/FF/FF Out", ffout);
        Logger.recordOutput("Shooter/Pivot/Angle/Setpoint", this.setpointState.position);
        Logger.recordOutput("Shooter/Pivot/Velocity/Setpoint", this.setpointState.velocity);
        Logger.recordOutput("Shooter/Pivot/Angle/Goal", this.goalState.position);
        Logger.recordOutput("Shooter/Pivot/Velocity/Goal", this.goalState.velocity);
    }

    public Command coast() {
        final var pivot = this;
        return new Command() {
            {
                this.setName("Coast");
                this.addRequirements(pivot);
            }

            @Override
            public void initialize() {
                pivot.stop(NeutralMode.COAST);
            }

            @Override
            public void end(boolean interrupted) {
                pivot.stop(NeutralMode.DEFAULT);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command genAngleCommand(String name, DoubleSupplier angleRads) {
        final var pivot = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(pivot);
            }

            @Override
            public void execute() {
                pivot.setAngleGoalRads(angleRads.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                pivot.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command idle() {
        return this.genAngleCommand(
            "Idle", 
            () -> idleAngle.get().in(Radians)
        );
    }
}
