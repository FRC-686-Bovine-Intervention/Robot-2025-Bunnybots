package frc.robot.subsystems.intakePivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class IntakePivotIOSim extends IntakePivotIOTalonFX{
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(5, 2),
        DCMotor.getFalcon500(2),
        IntakePivotConstants.motorToMechanism.reductionUnsigned(),
        1,
        IntakePivotConstants.minAngle.in(Radians),
        IntakePivotConstants.maxAngle.in(Radians),
        false,
        IntakePivotConstants.minAngle.in(Radians)
    );

    @Override
    public void updateInputs(IntakePivotIOInputs inputs) {
        var motorSimState = motor.getSimState();
        var cancoderSimState = cancoder.getSimState();

        pivotSim.setInputVoltage(motorSimState.getMotorVoltage());
        pivotSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(pivotSim.getAngleRads());
        var velocity = RadiansPerSecond.of(pivotSim.getVelocityRadPerSec());

        cancoderSimState.setRawPosition(IntakePivotConstants.sensorToMechanism.inverse().applyUnsigned(position.unaryMinus()));
        cancoderSimState.setVelocity(IntakePivotConstants.sensorToMechanism.inverse().applyUnsigned(velocity.unaryMinus()));

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        super.updateInputs(inputs);
    }
}
