package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

import frc.util.loggerUtil.inputs.LoggedMotor;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersIOInputs {
        boolean motorConnected = false;
        LoggedMotor motor = new LoggedMotor();
    }

    public default void updateInputs(IntakeRollersIOInputs inputs) {}

    public default void setVolts(double volts) {}
}
