package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.rollers.indexer.Indexer;
import frc.robot.subsystems.rollers.kicker.Kicker;
import frc.util.LoggedTracer;
import frc.util.VirtualSubsystem;

public class Rollers extends VirtualSubsystem {
    public final Kicker kicker;
    public final Indexer indexer;

    private final RollerSensorsIO sensorsIO;
    private final RollerSensorsIOInputsAutoLogged sensorsInputs = new RollerSensorsIOInputsAutoLogged();

    public Rollers(Kicker kicker, Indexer indexer, RollerSensorsIO sensorsIO) {
        this.kicker = kicker;
        this.indexer = indexer;
        this.sensorsIO = sensorsIO;
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Rollers/Before");
        this.sensorsIO.updateInputs(this.sensorsInputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Rollers/Update Inputs");
        Logger.processInputs("Inputs/Rollers/Sensors", this.sensorsInputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Rollers/Process Inputs");
    }
}
