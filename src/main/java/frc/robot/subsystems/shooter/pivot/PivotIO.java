package frc.robot.subsystems.shooter.pivot;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        boolean limitSwitch = false;
        boolean motorConnected = false;
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
    }

    public default void updateInputs(PivotIOInputs inputs) {}

    public default void setVolts(double volts) {}

    public default void setPositionRads(double positionRads, double velocityRadsPerSec, double feedforwardVolts) {}
    
    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void clearMotorStickyFaults(long bitmask) {}
    public default void clearEncoderStickyFaults(long bitmask) {}
}
