package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    public boolean stageBeamBroken() {
        return this.sensorsInputs.kickerSensor;
    }

    public Command idle() {
        return Commands.parallel(
            this.kicker.idle(),
            this.indexer.idle()
        ).withName("Idle");
    }

    public Command stage() {
        return Commands.parallel(
            this.kicker.stage(),
            this.indexer.stage()
        ).withName("Stage");
    }

    public Command kick() {
        return Commands.parallel(
            this.kicker.kick(),
            this.indexer.kick()
        ).withName("Kick");
    }

    public Command reverse() {
        return Commands.parallel(
            this.kicker.reverse(),
            this.indexer.reverse()
        ).withName("Reverse");
    }
}
