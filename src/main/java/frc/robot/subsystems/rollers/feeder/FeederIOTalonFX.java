package frc.robot.subsystems.rollers.feeder;

import static edu.wpi.first.units.Units.Amps;

import frc.robot.constants.HardwareDevices;
import frc.util.NeutralMode;
import frc.util.genericSubsystem.roller.GenericRollerSystemIOTalonFX;

public class FeederIOTalonFX extends GenericRollerSystemIOTalonFX implements FeederIO {
    public FeederIOTalonFX() {
        super(HardwareDevices.feederMotorID, true, NeutralMode.BRAKE, Amps.of(80));
    }
}
