package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class IntakeRollers extends SubsystemBase {
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Intake/Rollers/Idle Volts", Volts::of, 0.0);
    private static final LoggedTunable<Voltage> intakeVoltage = LoggedTunable.from("Intake/Rollers/Intake Volts", Volts::of, 4.5);

    public IntakeRollers(IntakeRollersIO io) {
        super("Intake/Rollers");

        System.out.println("[Init IntakeRollers] Instantiating IntakeRollers with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Rollers/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Rollers/Update Inputs");
        Logger.processInputs("Inputs/Intake/Rollers", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake Rollers/Process Inputs");
    }

    private Command genVoltsCommand(String name, DoubleSupplier voltSupplier) {
        final var rollers = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(rollers);
            }

            @Override
            public void execute() {
                rollers.io.setVolts(voltSupplier.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                rollers.io.setVolts(0.0);
            }
        };
    }

    public Command idle() {
        return this.genVoltsCommand("Idle", () -> idleVoltage.get().in(Volts));
    }
    public Command intake() {
        return this.genVoltsCommand("Intake", () -> intakeVoltage.get().in(Volts));
    }
}
