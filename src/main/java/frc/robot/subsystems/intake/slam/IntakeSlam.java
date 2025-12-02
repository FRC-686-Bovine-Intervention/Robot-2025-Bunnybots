package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.robotStructure.FourBarLinkage;
import frc.util.robotStructure.angle.ArmMech;

public class IntakeSlam extends SubsystemBase {
    private final IntakeSlamIO io;
    private final IntakeSlamIOInputsAutoLogged inputs = new IntakeSlamIOInputsAutoLogged();

    private static final LoggedTunable<Angle> retractAngle = LoggedTunable.from("Intake/Slam/Retract/Angle", Degrees::of, 80);
    private static final LoggedTunable<Angle> deployAngle = LoggedTunable.from("Intake/Slam/Deploy/Angle", Degrees::of, 0);
    private static final LoggedTunable<Angle> deployFlopAngle = LoggedTunable.from("Intake/Slam/Deploy/Flop Angle", Degrees::of, 70);

    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from(
        "Intake/Slam/PID",
        new PIDConstants(
            0.0,
            0.0,
            0.0
        )
    );
    
    private final Alert motorDisconnectedAlert = new Alert("Intake/Slam/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Slam Motor Disconnected!", AlertType.kError);

    private double angleRads = 0.0;
    private double velocityRadsPerSec = 0.0;
    private double motorOffsetRads = 0.0;

    private final FourBarLinkage primaryLinkage = new FourBarLinkage(
        IntakeSlamConstants.primaryFrameLength.in(Meters),
        IntakeSlamConstants.primaryDriverLength.in(Meters),
        IntakeSlamConstants.primaryFollowerLength.in(Meters),
        IntakeSlamConstants.primaryCouplerLength.in(Meters),
        true,
        false
    );
    private final FourBarLinkage secondaryLinkage = new FourBarLinkage(
        IntakeSlamConstants.secondaryFrameLength.in(Meters),
        IntakeSlamConstants.secondaryDriverLength.in(Meters),
        IntakeSlamConstants.secondaryFollowerLength.in(Meters),
        IntakeSlamConstants.secondaryCouplerLength.in(Meters),
        true,
        false
    );
    public final ArmMech primaryDriverMech = new ArmMech(IntakeSlamConstants.primaryDriverBase);
    public final ArmMech primaryFollowerMech = new ArmMech(IntakeSlamConstants.primaryFollowerBase);
    public final ArmMech primaryCouplerMech = new ArmMech(IntakeSlamConstants.primaryCouplerBase);
    public final ArmMech secondaryFollowerMech = new ArmMech(IntakeSlamConstants.secondaryFollowerBase);
    public final ArmMech secondaryCouplerMech = new ArmMech(IntakeSlamConstants.secondaryCouplerBase);

    public IntakeSlam(IntakeSlamIO io) {
        System.out.println("[Init IntakeSlam] Instantiating IntakeSlam with " + io.getClass().getSimpleName());
        this.io = io;

        this.io.configPID(pidConsts.get());
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Update Inputs");
        Logger.processInputs("Inputs/Intake/Slam", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Process Inputs");

        this.resetInternalAngleRads(IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getPositionRads()));
        this.velocityRadsPerSec = IntakeSlamConstants.sensorToMechanism.applyUnsigned(this.inputs.encoder.getVelocityRadsPerSec());

        var maxAngle = Units.degreesToRadians(80.052205);

        Logger.recordOutput("DEBUG/primaryDriver", IntakeSlamConstants.cadOrigin.plus(IntakeSlamConstants.primaryDriverBase).inverse());
        Logger.recordOutput("DEBUG/primaryFollower", IntakeSlamConstants.cadOrigin.plus(IntakeSlamConstants.primaryFollowerBase).inverse());
        Logger.recordOutput("DEBUG/primaryCoupler", IntakeSlamConstants.cadOrigin.plus(IntakeSlamConstants.primaryDriverBase).plus(IntakeSlamConstants.primaryCouplerBase).inverse());
        
        this.primaryLinkage.setDriverAngleRads(this.getAngleRads() - IntakeSlamConstants.primaryFrameNormalAngle.in(Radians));
        Logger.recordOutput("DEBUG/primary driver angle", this.primaryLinkage.getDriverAngleRads());
        Logger.recordOutput("DEBUG/primary follower angle", this.primaryLinkage.getFollowerAngleRads());
        Logger.recordOutput("DEBUG/primary coupler angle", this.primaryLinkage.getCouplerAngleRads());

        this.secondaryLinkage.setDriverAngleRads(this.primaryLinkage.getCouplerAngleRads() + IntakeSlamConstants.primaryFrameNormalAngle.in(Radians) - this.getAngleRads() - IntakeSlamConstants.secondaryFrameNormalAngle.in(Radians));
        Logger.recordOutput("DEBUG/secondary driver angle", this.secondaryLinkage.getDriverAngleRads());
        Logger.recordOutput("DEBUG/secondary follower angle", this.secondaryLinkage.getFollowerAngleRads());
        Logger.recordOutput("DEBUG/secondary coupler angle", this.secondaryLinkage.getCouplerAngleRads());

        this.primaryDriverMech.setRads(this.angleRads - maxAngle);
        this.primaryFollowerMech.setRads(this.primaryLinkage.getFollowerAngleRads() + IntakeSlamConstants.primaryFrameNormalAngle.in(Radians) - Units.degreesToRadians(87.881550));
        this.primaryCouplerMech.setRads(this.primaryLinkage.getCouplerAngleRads() + IntakeSlamConstants.primaryFrameNormalAngle.in(Radians) - this.primaryLinkage.getDriverAngleRads() - maxAngle - Units.degreesToRadians(-139.604453));
        this.secondaryFollowerMech.setRads(this.secondaryLinkage.getFollowerAngleRads() + IntakeSlamConstants.secondaryFrameNormalAngle.in(Radians) - Units.degreesToRadians(-144.10743));
        this.secondaryCouplerMech.setRads(this.secondaryLinkage.getCouplerAngleRads() + IntakeSlamConstants.secondaryFrameNormalAngle.in(Radians) - this.secondaryLinkage.getDriverAngleRads() - Units.degreesToRadians(-180));

        Logger.recordOutput("Intake/Slam/Angle/Measured", this.getAngleRads());
        Logger.recordOutput("Intake/Slam/Velocity/Measured", this.getVelocityRadsPerSec());

        if (pidConsts.hasChanged(this.hashCode())) {
            this.io.configPID(pidConsts.get());
        }

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);

        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Slam");
    }

    private void resetInternalAngleRads(double angleRads) {
        this.angleRads = angleRads;
        this.motorOffsetRads = this.angleRads - IntakeSlamConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
    }

    public double getAngleRads() {
        return this.angleRads;
    }

    public double getVelocityRadsPerSec() {
        return this.velocityRadsPerSec;
    }

    private void setAngleGoalRads(double angleRads) {
        this.io.setPositionRads(
            // IntakeSlamConstants.motorToMechanism.inverse().applyUnsigned(angleRads - this.motorOffsetRads),
            angleRads,
            0.0,
            0.0
        );
        Logger.recordOutput("Intake/Slam/Angle/Goal", angleRads);
        Logger.recordOutput("Intake/Slam/Velocity/Goal", 0.0);
    }

    public Command coast() {
        final var slam = this;
        return new Command() {
            {
                this.setName("Coast");
                this.addRequirements(slam);
            }

            @Override
            public void initialize() {
                slam.io.stop(NeutralMode.COAST);
            }

            @Override
            public void end(boolean interrupted) {
                slam.io.stop(NeutralMode.DEFAULT);
            }

            @Override
            public boolean runsWhenDisabled() {
                return true;
            }
        };
    }

    public Command retract() {
        final var slam = this;
        return new Command() {
            {
                this.setName("Retract");
                this.addRequirements(slam);
            }

            @Override
            public void execute() {
                slam.setAngleGoalRads(retractAngle.get().in(Radians));
            }

            @Override
            public void end(boolean interrupted) {
                slam.io.stop(NeutralMode.DEFAULT);
            }
        };
    }

    public Command deploy() {
        final var slam = this;
        return new Command() {
            {
                this.setName("Deploy");
                this.addRequirements(slam);
            }

            @Override
            public void execute() {
                if (slam.getAngleRads() > deployFlopAngle.get().in(Radians)) {
                    slam.setAngleGoalRads(deployAngle.get().in(Radians));
                } else {
                    slam.io.stop(NeutralMode.COAST);
                }
            }

            @Override
            public void end(boolean interrupted) {
                slam.io.stop(NeutralMode.COAST);
            }
        };
    }
}
