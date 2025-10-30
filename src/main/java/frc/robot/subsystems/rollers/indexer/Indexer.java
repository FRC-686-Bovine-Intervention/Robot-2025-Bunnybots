package frc.robot.subsystems.rollers.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.util.genericSubsystem.roller.GenericRollerSystem;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Indexer extends GenericRollerSystem{
    public static final LoggedTunable<Voltage> agitateVoltage = LoggedTunable.from("Rollers/Indexer/Agitate", Volts::of, +6);
    public static final LoggedTunable<Voltage> feedVoltage =    LoggedTunable.from("Rollers/Indexer/Feed", Volts::of, +3);
    public static final LoggedTunable<Voltage> intakeVoltage =  LoggedTunable.from("Rollers/Indexer/Intake", Volts::of, +4);
    public static final LoggedTunable<Voltage> reverseVoltage = LoggedTunable.from("Rollers/Indexer/Reverse", Volts::of, -6);
    public static final LoggedTunable<Voltage> idleVoltage =    LoggedTunable.from("Rollers/Indexer/Idle", Volts::of, 0);
    
    public Indexer(IndexerIO io) {
        super("Rollers/Indexer", io);
    }
}
