package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Degrees;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class IntakeSlam extends SubsystemBase {
    private final IntakeSlamIO io;
    private final IntakeSlamIOInputsAutoLogged inputs = new IntakeSlamIOInputsAutoLogged();

    private static final LoggedTunable<PIDConstants> pidConsts = LoggedTunable.from("Intake/Slam/PID", new PIDConstants(
        0,
        0,
        0
    ));

    private static final LoggedTunable<Angle> idleAngle = LoggedTunable.from("Intake/Slam/Deploy Angle", Degrees::of, 90.0);
    private static final LoggedTunable<Angle> deployAngle = LoggedTunable.from("Intake/Slam/Deploy Angle", Degrees::of, 90.0);

    private final PIDController pid = new PIDController(0,0,0);

    private double positionRads = 0.0;
    private double velocityRadsPerSec = 0.0;


    private IntakeSlam(IntakeSlamIO io) {
        this.io = io;
        pidConsts.get().update(this.pid);
    }

    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Inputs/Intake/Slam", this.inputs);
        
        this.positionRads = IntakeSlamConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getPositionRads());
        this.velocityRadsPerSec = IntakeSlamConstants.motorToMechanism.applyUnsigned(this.inputs.motor.encoder.getVelocityRadsPerSec());

        if(pidConsts.hasChanged(this.hashCode())) {
            pidConsts.get().update(this.pid);
        }
    }
}
