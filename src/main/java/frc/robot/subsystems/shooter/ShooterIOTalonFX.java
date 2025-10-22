package frc.robot.subsystems.shooter;

import java.util.Optional;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.constants.HardwareDevices;
import frc.robot.constants.RobotConstants;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.faults.DeviceFaults;
import frc.util.faults.DeviceFaults.FaultType;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor.EncodedMotorStatusSignalCache;

public class ShooterIOTalonFX implements ShooterIO {
    protected final TalonFX leftMotor = HardwareDevices.shooterLeftMotorID.talonFX();
    protected final TalonFX rightMotor = HardwareDevices.shooterRightMotorID.talonFX();

    private final EncodedMotorStatusSignalCache leftMotorStatusSignalCache;
    private final EncodedMotorStatusSignalCache rightMotorStatusSignalCache;
    
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final NeutralOut neutralOutRequest = new NeutralOut();
    private final CoastOut coastOutRequest = new CoastOut();
    private final StaticBrake staticBrakeRequest = new StaticBrake();
    private final StrictFollower followerRequest;

    public ShooterIOTalonFX() {
        var motorConfig = new TalonFXConfiguration();
        motorConfig.MotorOutput
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast)
        ;
        this.leftMotor.getConfigurator().apply(motorConfig);

        motorConfig.MotorOutput
            .withInverted(InvertedValue.CounterClockwise_Positive)
        ;
        this.rightMotor.getConfigurator().apply(motorConfig);
        this.followerRequest = new StrictFollower(this.leftMotor.getDeviceID());
        this.rightMotor.setControl(this.followerRequest);

        this.leftMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.leftMotor);
        this.rightMotorStatusSignalCache = EncodedMotorStatusSignalCache.from(this.rightMotor);
        
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.leftMotorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency, this.rightMotorStatusSignalCache.encoder().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.leftMotorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.rioUpdateFrequency.div(2), this.rightMotorStatusSignalCache.motor().getStatusSignals());
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.leftMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.leftMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getFaultStatusSignals(this.rightMotor));
        BaseStatusSignal.setUpdateFrequencyForAll(RobotConstants.deviceFaultUpdateFrequency, FaultType.getStickyFaultStatusSignals(this.rightMotor));
        this.leftMotor.optimizeBusUtilization();
        this.rightMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            this.leftMotorStatusSignalCache.encoder().position(),
            this.leftMotorStatusSignalCache.encoder().velocity(),
            this.leftMotorStatusSignalCache.motor().appliedVoltage(),
            this.leftMotorStatusSignalCache.motor().statorCurrent(),
            this.leftMotorStatusSignalCache.motor().supplyCurrent(),
            this.leftMotorStatusSignalCache.motor().torqueCurrent(),
            this.leftMotorStatusSignalCache.motor().deviceTemperature(),
            this.rightMotorStatusSignalCache.encoder().position(),
            this.rightMotorStatusSignalCache.encoder().velocity(),
            this.rightMotorStatusSignalCache.motor().appliedVoltage(),
            this.rightMotorStatusSignalCache.motor().statorCurrent(),
            this.rightMotorStatusSignalCache.motor().supplyCurrent(),
            this.rightMotorStatusSignalCache.motor().torqueCurrent(),
            this.rightMotorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.leftMotorConnected = BaseStatusSignal.isAllGood(
            this.leftMotorStatusSignalCache.encoder().position(),
            this.leftMotorStatusSignalCache.encoder().velocity(),
            this.leftMotorStatusSignalCache.motor().appliedVoltage(),
            this.leftMotorStatusSignalCache.motor().statorCurrent(),
            this.leftMotorStatusSignalCache.motor().supplyCurrent(),
            this.leftMotorStatusSignalCache.motor().torqueCurrent(),
            this.leftMotorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.rightMotorConnected = BaseStatusSignal.isAllGood(
            this.rightMotorStatusSignalCache.encoder().position(),
            this.rightMotorStatusSignalCache.encoder().velocity(),
            this.rightMotorStatusSignalCache.motor().appliedVoltage(),
            this.rightMotorStatusSignalCache.motor().statorCurrent(),
            this.rightMotorStatusSignalCache.motor().supplyCurrent(),
            this.rightMotorStatusSignalCache.motor().torqueCurrent(),
            this.rightMotorStatusSignalCache.motor().deviceTemperature()
        );
        inputs.leftMotor.updateFrom(this.leftMotorStatusSignalCache);
        inputs.rightMotor.updateFrom(this.rightMotorStatusSignalCache);
    }

    @Override
    public void setVoltage(double volts) {
        this.leftMotor.setControl(this.voltageRequest
            .withOutput(volts)
        );
        this.rightMotor.setControl(this.followerRequest);
    }

    @Override
    public void setVelocity(double velocity, double feedforwardVolts) {
        this.leftMotor.setControl(this.velocityRequest
            .withVelocity(velocity)
            .withFeedForward(feedforwardVolts)
        );
        this.rightMotor.setControl(this.followerRequest);
    }

    @Override
    public void stop(Optional<NeutralMode> neutralMode) {
        var controlRequest = NeutralMode.selectControlRequest(neutralMode, this.neutralOutRequest, this.coastOutRequest, this.staticBrakeRequest);
        this.leftMotor.setControl(controlRequest);
        this.rightMotor.setControl(controlRequest);
    }

    @Override
    public void configPID(PIDConstants pidConstants) {
        var leftConfig = new Slot0Configs();
        var rightConfig = new Slot0Configs();
        this.leftMotor.getConfigurator().refresh(leftConfig);
        this.rightMotor.getConfigurator().refresh(rightConfig);
        pidConstants.update(leftConfig);
        pidConstants.update(rightConfig);
        this.leftMotor.getConfigurator().apply(leftConfig);
        this.rightMotor.getConfigurator().apply(rightConfig);
    }

    @Override
    public void clearLeftMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.leftMotor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.leftMotor);
                }
            }
        }
    }

    @Override
    public void clearRightMotorStickyFaults(long bitmask) {
        if (bitmask == DeviceFaults.noneMask) {return;}
        if (bitmask == DeviceFaults.allMask) {
            this.rightMotor.clearStickyFaults();
        } else {
            for (var faultType : FaultType.possibleTalonFXFaults) {
                if (faultType.isPartOf(bitmask)) {
                    faultType.clearStickyFaultOn(this.rightMotor);
                }
            }
        }
    }
}
