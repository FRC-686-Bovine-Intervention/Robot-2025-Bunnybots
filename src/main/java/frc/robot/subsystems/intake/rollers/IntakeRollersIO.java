package frc.robot.subsystems.intake.rollers;

import org.littletonrobotics.junction.AutoLog;

import frc.util.loggerUtil.inputs.LoggedMotor;

public interface IntakeRollersIO {
    @AutoLog
    public static class IntakeRollersIOInputs {
        LoggedMotor motor = new LoggedMotor();
        boolean motorConnected = false;

    }

    public default void updateInputs(IntakeRollersIOInputs inputs) {}

    public default void setVolts(double volts) {}

}
