package frc.robot.subsystems.rollers.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        super("Rollers/Indexer");

        System.out.println("[Init Indexer] Instantiating Indexer with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Rollers Indexer/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Rollers Indexer/Update Inputs");
        Logger.processInputs("Inputs/Rollers/Indexer", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Rollers Indexer/Process Inputs");
    }
}
