package frc.robot.subsystems.rollers.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.util.genericSubsystem.roller.GenericRollerSystem;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Intake extends GenericRollerSystem {
    public static final LoggedTunable<Voltage> intakeVoltage =  LoggedTunable.from("Rollers/Indexer/Intake", Volts::of, +4);
    public static final LoggedTunable<Voltage> reverseVoltage = LoggedTunable.from("Rollers/Indexer/Reverse", Volts::of, -6);
    public static final LoggedTunable<Voltage> idleVoltage =    LoggedTunable.from("Rollers/Indexer/Idle", Volts::of, 0);
    
    public Intake(IntakeIO io){
        super("Rollers/Intake", io);
    }    
}
