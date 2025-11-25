package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class IntakeSlamIOSim extends IntakeSlamIOTalonFX{
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(5, 2),
        DCMotor.getFalcon500(2),
        IntakeSlamConstants.motorToMechanism.reductionUnsigned(),
        1,
        IntakeSlam.retractAngle.get().in(Radians),
        IntakeSlam.deployAngle.get().in(Radians),
        false,
        IntakeSlam.retractAngle.get().in(Radians)
    );

    @Override
    public void updateInputs(IntakeSlamIOInputs inputs) {
        var motorSimState = motor.getSimState();

        pivotSim.setInputVoltage(motorSimState.getMotorVoltage());
        pivotSim.update(RobotConstants.rioUpdatePeriodSecs);

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
