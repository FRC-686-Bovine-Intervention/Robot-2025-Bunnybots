package frc.robot.subsystems.shooter;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Velocity;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.inputs.LoggedEncodedMotor;
import frc.util.loggerUtil.inputs.LoggedFaults;

public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        boolean leftMotorConnected = false;
        boolean rightMotorConnected = false;
        LoggedEncodedMotor leftMotor = new LoggedEncodedMotor();
        LoggedEncodedMotor rightMotor = new LoggedEncodedMotor();
        LoggedFaults leftMotorFaults = new LoggedFaults();
        LoggedFaults rightMotorFaults = new LoggedFaults();
    }
    
    public default void updateInputs(ShooterIOInputs inputs) {}

    public default void setVoltage(double volts) {}

    public default void setVelocity(double velocity, double feedforwardVolts) {}

    public default void stop(Optional<NeutralMode> neutralMode) {}

    public default void configPID(PIDConstants pidConstants) {}

    public default void clearLeftMotorStickyFaults(long bitmask) {}
    public default void clearRightMotorStickyFaults(long bitmask) {}
}
