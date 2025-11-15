package frc.robot.subsystems.intake.rollers;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class IntakeRollers extends SubsystemBase {
    private final IntakeRollersIO io;
    private final IntakeRollersIOInputsAutoLogged inputs = new IntakeRollersIOInputsAutoLogged();

    private static final LoggedTunable<Voltage> intakeVoltage = LoggedTunable.from("Intake/Rollers/Intake Voltage", Volts::of,+6);
    private static final LoggedTunable<Voltage> idleVoltage = LoggedTunable.from("Intake/Idle/Intake Voltage", Volts::of,+0);
    private static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Intake/Eject/Intake Voltage", Volts::of,-4);
    
    public IntakeRollers(IntakeRollersIO io) {
        this.io = io;
    }
 
    @Override
    public void periodic() {
        this.io.updateInputs(this.inputs);
        Logger.processInputs("Inputs/Intake/Rollers", this.inputs);
    }

    public Command genVoltageCommand(String name, DoubleSupplier voltsSupplier) {
        final var intake = this;
        return new Command() {
            {
                this.setName(name);
                this.addRequirements(intake);
            }

            @Override
            public void execute() {
                intake.io.setVolts(voltsSupplier.getAsDouble());
            }
            @Override
            public void end(boolean interrupted) {
                intake.io.setVolts(0);
            } 
        };
    }
    public Command idle() {
        return genVoltageCommand("Idle", ()->idleVoltage.get().in(Volts));
    }

    public Command intake() {
        return genVoltageCommand("Intake", ()->intakeVoltage.get().in(Volts));
    }

    public Command eject() {
        return genVoltageCommand("Eject", ()->ejectVoltage.get().in(Volts));
    }
}
