package frc.robot.subsystems.rollers.kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.loggerUtil.inputs.LoggedMotor.MotorStatusSignalCache;

public class KickerIOTalonFX implements KickerIO {
    private final TalonFX motor = HardwareDevices.kickerMotorID.talonFX();

    private final MotorStatusSignalCache motorStatusSignalCache;

    private final BaseStatusSignal[] refreshSignals;
    private final BaseStatusSignal[] motorConnectedSignals;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public KickerIOTalonFX() {
        var config = new TalonFXConfiguration();
        config.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)
        ;
        this.motor.getConfigurator().apply(config);

        this.motorStatusSignalCache = MotorStatusSignalCache.from(this.motor);

        this.refreshSignals = new BaseStatusSignal[] {
            this.motorStatusSignalCache.appliedVoltage(),
            this.motorStatusSignalCache.statorCurrent(),
            this.motorStatusSignalCache.supplyCurrent(),
            this.motorStatusSignalCache.torqueCurrent(),
            this.motorStatusSignalCache.deviceTemperature(),
        };
        this.motorConnectedSignals = new BaseStatusSignal[] {
            this.motorStatusSignalCache.appliedVoltage(),
            this.motorStatusSignalCache.statorCurrent(),
            this.motorStatusSignalCache.supplyCurrent(),
            this.motorStatusSignalCache.torqueCurrent(),
            this.motorStatusSignalCache.deviceTemperature(),
        };

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.getStatusSignals());
        // BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
        // BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
        this.motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        BaseStatusSignal.refreshAll(this.refreshSignals);
        inputs.motorConnected = BaseStatusSignal.isAllGood(this.motorConnectedSignals);
        inputs.motor.updateFrom(this.motorStatusSignalCache);
    }

    @Override
    public void setVolts(double volts) {
        this.motor.setControl(this.voltageRequest
            .withOutput(volts)
        );
    }
}
