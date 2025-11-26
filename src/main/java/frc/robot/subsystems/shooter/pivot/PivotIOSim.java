package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.RobotConstants;

public class PivotIOSim extends PivotIOTalonFX {
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
        LinearSystemId.identifyPositionSystem(5, 2),
        DCMotor.getKrakenX60(1),
        PivotConstants.motorToMechanism.reductionUnsigned(),
        0.5,
        PivotConstants.minAngle.in(Radians),
        PivotConstants.maxAngle.in(Radians),
        false,
        PivotConstants.minAngle.in(Radians)
    );

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        var motorSimState = this.motor.getSimState();

        this.pivotSim.setInputVoltage(motorSimState.getMotorVoltage());
        this.pivotSim.update(RobotConstants.rioUpdatePeriodSecs);

        var position = Radians.of(this.pivotSim.getAngleRads());
        var velocity = RadiansPerSecond.of(this.pivotSim.getVelocityRadPerSec());

        motorSimState.setRawRotorPosition(PivotConstants.motorToMechanism.inverse().applyUnsigned(position));
        motorSimState.setRotorVelocity(PivotConstants.motorToMechanism.inverse().applyUnsigned(velocity));

        motorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

        motorSimState.setReverseLimit(this.pivotSim.hasHitLowerLimit());

        super.updateInputs(inputs);
    }
}
