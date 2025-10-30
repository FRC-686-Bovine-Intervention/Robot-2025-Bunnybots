package frc.util.genericSubsystem.roller;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;

public abstract class GenericRollerSystem extends SubsystemBase{
    private final String name;
    private final GenericRollerSystemIO io;
    protected final GenericRollerSystemIOInputsAutoLogged inputs = new GenericRollerSystemIOInputsAutoLogged();
    private final Alert motorDisconnectedAlert;
    private final Alert motorDisconnectedGlobalAlert;

    public GenericRollerSystem(String name, GenericRollerSystemIO io) {
        this.name = name;
        this.io = io;

        motorDisconnectedAlert = new Alert(name + "/Alerts", "Motor disconnected!", AlertType.kError);
        motorDisconnectedGlobalAlert = new Alert(name + " motor disconnected!", AlertType.kError);
    }

    @Override
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/"+ name + "/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/"+ name + "/Update Inputs");
        Logger.processInputs("Inputs/" + name, this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/"+ name + "/Process Inputs");

        this.motorDisconnectedAlert.set(!this.inputs.motorConnected);
        this.motorDisconnectedGlobalAlert.set(!this.inputs.motorConnected);
        
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/Subsystem/Intake");
    }

    public Command setVoltage(DoubleSupplier volts) {
        final var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("setVoltage");
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                subsystem.io.setVolts(volts.getAsDouble());
            }

            @Override
            public void end(boolean interrupted) {
                subsystem.io.setVolts(0);
            }
        };
    }

    public Command stop() {
        final var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("stop");
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                subsystem.io.stop(Optional.empty());
            }
        };
    }

    public Command stop(Optional<NeutralMode> neutralMode) {
        final var subsystem = this;
        return new Command() {
            {
                addRequirements(subsystem);
                setName("stop");
            }

            @Override
            public void initialize() {

            }

            @Override
            public void execute() {
                subsystem.io.stop(neutralMode);
            }
        };
    }
}