package frc.robot.subsystems.shooter.flywheel;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface FlywheelIO {
    @AutoLog
    public static class FlywheelIOInputs {
        boolean leftMotorConnected = false;
        boolean rightMotorConnected = false;
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
        LoggedFaults leftMotorFaults = new LoggedFaults();
        LoggedFaults rightMotorFaults = new LoggedFaults();
    }
    
    public default void updateInputs(FlywheelIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVelocity(double velocityRadsPerSec, double feedforwardVolts) {}

    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void clearLeftMotorStickyFaults(long bitmask) {}
    public default void clearRightMotorStickyFaults(long bitmask) {}
}
