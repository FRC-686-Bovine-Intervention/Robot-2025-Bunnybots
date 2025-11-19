package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.angle.ArmMech;

public class IntakeSlam extends SubsystemBase{
    private final IntakeSlamIO io;
    private final IntakeSlamIOInputsAutoLogged inputs = new IntakeSlamIOInputsAutoLogged();

    protected static final LoggedTunable<Angle> minAngle = LoggedTunable.from("Intake/Slam/Min Angle", Degrees::of, 20);
    protected static final LoggedTunable<Angle> maxAngle = LoggedTunable.from("Intake/Slam/Max Angle", Degrees::of, 20);

    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Intake/Slam/PID",
        new PIDConstants(
            150,
            0,
            0
        )
    );

    private double angleRads = 0;
    private double velocityRadsPerSec = 0.0;

    public final ArmMech mech = new ArmMech(IntakeSlamConstants.slamBase);

    private final Alert motorDisconnectedAlert = new Alert("Intake/Slam/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Slam Motor Disconnected!", AlertType.kError);

    public IntakeSlam(IntakeSlamIO io) {
        System.out.println("[Init IntakeSlam] Instantiating IntakeSlam with " + io.getClass().getSimpleName());
        this.io = io;

        this.io.configPID(pidConsts.get());
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Slam/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Slam/Update Inputs");
        Logger.processInputs("Inputs/Intake/Slam", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Slam/Process Inputs");

        this.angleRads = IntakeSlamConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
        this.velocityRadsPerSec = IntakeSlamConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec());

        this.mech.setRads(this.getAngleRads());

        Logger.recordOutput("Intake/Slam/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("Intake/Slam/Velocity/Measured", this.getVelocityRadsPerSec());

        if (pidConsts.hasChanged(hashCode())) {
            this.io.configPID(pidConsts.get());
        }

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Slam/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Slam");
    }

    private double getAngleRads() {
        return this.angleRads;
    }

    private double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }

    private void stop(Optional<NeutralMode> neutralMode) {
        this.io.stop(neutralMode);
    }

    private void setAngleGoalRads(double angleRads) {
        this.io.setPosition(
            angleRads,
            0.0
        );
        Logger.recordOutput("Intake/Slam/Angle/Goal", angleRads);
        Logger.recordOutput("Intake/Slam/Velocity/Goal", 0.0);
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
            () -> minAngle.get().in(Radians)
        );
    }

    public Command deploy() {
        return genCommand(
            "Deploy", 
            () -> maxAngle.get().in(Radians)
        );
    }
}
