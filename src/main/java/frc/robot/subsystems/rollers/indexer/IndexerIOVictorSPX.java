package frc.robot.subsystems.rollers.indexer;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;

import frc.robot.constants.HardwareDevices;

public class IndexerIOVictorSPX implements IndexerIO {
    private final VictorSPX leftMotor = HardwareDevices.indexerLeftMotorID.victorSPX();
    private final VictorSPX rightMotor = HardwareDevices.indexerRightMotorID.victorSPX();

    public IndexerIOVictorSPX() {
        var config = new VictorSPXConfiguration();
        this.leftMotor.configAllSettings(config);
        this.rightMotor.configAllSettings(config);
        this.leftMotor.setInverted(InvertType.InvertMotorOutput);
        this.rightMotor.setInverted(InvertType.None);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.leftMotorConnected = true;
        inputs.leftMotor.updateFrom(this.leftMotor);
        inputs.rightMotorConnected = true;
        inputs.rightMotor.updateFrom(this.rightMotor);
    }

    @Override
    public void setVolts(double volts) {
        this.leftMotor.set(VictorSPXControlMode.PercentOutput, volts / 12.0);
        this.rightMotor.set(VictorSPXControlMode.PercentOutput, volts / 12.0);
    }
}
