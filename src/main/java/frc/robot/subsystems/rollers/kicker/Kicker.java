package frc.robot.subsystems.rollers.kicker;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Kicker extends SubsystemBase {
    private final KickerIO io;
    private final KickerIOInputsAutoLogged inputs = new KickerIOInputsAutoLogged();

    private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Rollers/Kicker/Idle Volts", Volts::of, 0.0);
    private static final LoggedTunable<Voltage> stageVoltage = LoggedTunable.from("Rollers/Kicker/Stage Volts", Volts::of, 3.0);
    private static final LoggedTunable<Voltage> kickVoltage = LoggedTunable.from("Rollers/Kicker/Kick Volts", Volts::of, 6.0);

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

    private Command genVoltsCommand(String name, DoubleSupplier voltSupplier) {
        final var kicker = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(kicker);
            }

            @Override
            public void execute() {
                kicker.io.setVolts(voltSupplier.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                kicker.io.setVolts(0.0);
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
