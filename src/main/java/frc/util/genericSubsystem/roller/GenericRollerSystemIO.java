package frc.util.genericSubsystem.roller;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;

import frc.util.NeutralMode;
import frc.util.loggerUtil.inputs.LoggedFaults;
import frc.util.loggerUtil.inputs.LoggedMotor;

public interface GenericRollerSystemIO {
    @AutoLog
    abstract class GenericRollerSystemIOInputs {
        public boolean motorConnected = false;
        public LoggedMotor motor = new LoggedMotor();
        public LoggedFaults motorFaults = new LoggedFaults();
    }

    default void updateInputs(GenericRollerSystemIOInputs inputs) {}

    default void setVolts(double volts) {}

    default void stop(Optional<NeutralMode> neutralMode) {}

    default void clearMotorStickyFaults(long bitmask) {}
}
