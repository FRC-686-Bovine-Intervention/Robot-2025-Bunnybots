package frc.robot.subsystems.rollers.kicker;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    public Kicker(KickerIO io) {
        super("Rollers/Kicker");

        System.out.println("[Init Kicker] Instantiating Kicker with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Rollers Kicker/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Rollers Kicker/Update Inputs");
        Logger.processInputs("Inputs/Rollers/Kicker", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Rollers Kicker/Process Inputs");
    }
}
