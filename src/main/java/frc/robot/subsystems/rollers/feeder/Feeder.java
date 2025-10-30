package frc.robot.subsystems.rollers.feeder;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.util.genericSubsystem.roller.GenericRollerSystem;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Feeder extends GenericRollerSystem {
    public static final LoggedTunable<Voltage> shootVoltage =    LoggedTunable.from("Rollers/Feeder/Shoot", Volts::of, +3);
    public static final LoggedTunable<Voltage> intakeVoltage =  LoggedTunable.from("Rollers/Feeder/Intake", Volts::of, +4);
    public static final LoggedTunable<Voltage> reverseVoltage = LoggedTunable.from("Rollers/Feeder/Reverse", Volts::of, -6);
    public static final LoggedTunable<Voltage> idleVoltage =    LoggedTunable.from("Rollers/Feeder/Idle", Volts::of, 0);

    public Feeder(FeederIO io) {
        super("Rollers/Feeder", io);
    }
}
