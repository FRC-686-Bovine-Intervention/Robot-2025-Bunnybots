package frc.util.genericSubsystem.roller;

import static edu.wpi.first.units.Units.Amps;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.Current;
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.hardwareID.can.CANDevice;
import frc.util.loggerUtil.inputs.LoggedMotor.MotorStatusSignalCache;

public abstract class GenericRollerSystemIOTalonFX implements GenericRollerSystemIO{
    protected final TalonFX motor;

    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final NeutralOut neutralOutRequest = new NeutralOut();
    private final CoastOut coastOutRequest = new CoastOut();
    private final StaticBrake staticBrakeRequest = new StaticBrake();

    private final MotorStatusSignalCache motorStatusSignalCache;

    public GenericRollerSystemIOTalonFX(CANDevice device, boolean clockwise, Optional<NeutralMode> defaultNeutralMode, Current currentLimit) {
        this.motor = device.talonFX();
        
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withNeutralMode(defaultNeutralMode.get().getPhoenix6NeutralMode())
            .withInverted(clockwise ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        ;
        motorConfig.CurrentLimits
            .withStatorCurrentLimit(currentLimit)
            .withStatorCurrentLimitEnable(true)
        ;

        this.motor.getConfigurator().apply(motorConfig);

        this.motorStatusSignalCache = MotorStatusSignalCache.from(this.motor);

        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.motorStatusSignalCache.getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.motor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.motor));
        this.motor.optimizeBusUtilization();
    }
    
    @Override
    public void updateInputs(GenericRollerSystemIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            this.motorStatusSignalCache.appliedVoltage(),
            this.motorStatusSignalCache.statorCurrent(),
            this.motorStatusSignalCache.supplyCurrent(),
            this.motorStatusSignalCache.torqueCurrent(),
            this.motorStatusSignalCache.deviceTemperature()
        );
        inputs.motorConnected = BaseStatusSignal.isAllGood(
            this.motorStatusSignalCache.appliedVoltage(),
            this.motorStatusSignalCache.statorCurrent(),
            this.motorStatusSignalCache.supplyCurrent(),
            this.motorStatusSignalCache.torqueCurrent(),
            this.motorStatusSignalCache.deviceTemperature()
        );
        inputs.motor.updateFrom(this.motorStatusSignalCache);
    }

    @Override
    public void setVolts(double volts) {
        this.motor.setControl(this.voltageRequest
            .withOutput(volts)
        );
    }

    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        this.motor.setControl(NeutralMode.selectControlRequest(neutralMode, neutralOutRequest, coastOutRequest, staticBrakeRequest));
    }

    @Override
    public void clearMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.motor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.motor);
                }
            }
        }
    }
}
