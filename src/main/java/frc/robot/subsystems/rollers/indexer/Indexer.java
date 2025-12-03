package frc.robot.subsystems.rollers.indexer;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Rollers/Indexer/Idle Volts", Volts::of, 0.0);
    private static final LoggedTunable<Voltage> stageVoltage = LoggedTunable.from("Rollers/Indexer/Stage Volts", Volts::of, 3.0);
    private static final LoggedTunable<Voltage> kickVoltage = LoggedTunable.from("Rollers/Indexer/Kick Volts", Volts::of, 6.0);

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
    
    private Command genVoltsCommand(String name, DoubleSupplier voltSupplier) {
        final var indexer = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(indexer);
            }

            @Override
            public void execute() {
                indexer.io.setVolts(voltSupplier.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                indexer.io.setVolts(0.0);
            }
        };
    }

    public Command idle() {
        return this.genVoltsCommand("Idle", () -> idleVoltage.get().in(Volts));
    }
    public Command stage() {
        return this.genVoltsCommand("Stage", () -> stageVoltage.get().in(Volts));
    }
    public Command kick() {
        return this.genVoltsCommand("Kick", () -> kickVoltage.get().in(Volts));
    }
}
