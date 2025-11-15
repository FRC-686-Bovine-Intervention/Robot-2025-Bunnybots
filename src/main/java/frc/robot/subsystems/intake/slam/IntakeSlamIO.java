package frc.robot.subsystems.intake.slam;

import org.littletonrobotics.junction.AutoLog;

import frc.util.loggerUtil.inputs.LoggedEncodedMotor;

public interface IntakeSlamIO {
    @AutoLog
    public static class IntakeSlamIOInputs {
        LoggedEncodedMotor motor = new LoggedEncodedMotor();
        boolean motorConnected = false;
    }

    public default void updateInputs(IntakeSlamIOInputs inputs) {}
    public default void setVolts(double volts) {}
}
