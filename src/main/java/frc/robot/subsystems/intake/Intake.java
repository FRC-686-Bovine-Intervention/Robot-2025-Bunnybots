package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    public static final LoggedTunable<Voltage> intakeVoltage = LoggedTunable.from("Intake/Voltages/Intake", Volts::of, +6);
    public static final LoggedTunable<Voltage> ejectVoltage = LoggedTunable.from("Intake/Voltages/Eject", Volts::of, -3);

    private final Alert motorDisconnectedAlert = new Alert("Intake/Alerts", "Motor Disconnected", AlertType.kError);
    private final Alert motorDisconnectedGlobalAlert = new Alert("Intake Motor Disconnected!", AlertType.kError);

    public Intake(IntakeIO io) {
        System.out.println("[Init Intake] Instantiated Intake with " + io.getClass().getSimpleName());
        this.io = io;
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Update Inputs");
        Logger.processInputs("Inputs/Intake", this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Process Inputs");

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);
        
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake");
    }

    private Command genCommand(
        String name,
        DoubleSupplier volts
    ) {
        final var intake = this;

        return new Command() {
            {
                this.setName(name);
                this.addRequirements(intake);
            }

            @Override
            public void execute() {
                intake.io.setVolts(volts.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                intake.io.setVolts(0);
            }
        };
    }

    public Command stop() {
        return genCommand(
            "Stop",
            () -> 0.0
        );
    }

    public Command intake() {
        return genCommand(
            "Intake",
            () -> intakeVoltage.get().in(Volts)
        );
    }

    public Command eject() {
        return genCommand(
            "Eject",
            () -> ejectVoltage.get().in(Volts)
        );
    }
}
