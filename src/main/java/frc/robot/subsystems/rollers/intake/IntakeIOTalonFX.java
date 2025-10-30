package frc.robot.subsystems.rollers.intake;

import static edu.wpi.first.units.Units.Amps;

import frc.robot.constants.HardwareDevices;
import frc.util.NeutralMode;
import frc.util.genericSubsystem.roller.GenericRollerSystemIOTalonFX;

public class IntakeIOTalonFX extends GenericRollerSystemIOTalonFX  implements IntakeIO {
    public IntakeIOTalonFX() {
        super(HardwareDevices.intakeMotorID, true, NeutralMode.COAST, Amps.of(80));
    }
}
