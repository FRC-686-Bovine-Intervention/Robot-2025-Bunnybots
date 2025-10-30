package frc.robot.subsystems.rollers.indexer;

import static edu.wpi.first.units.Units.Amps;

import frc.robot.constants.HardwareDevices;
import frc.util.NeutralMode;
import frc.util.genericSubsystem.roller.GenericRollerSystemIOTalonFX;

public class IndexerIOTalonFX extends GenericRollerSystemIOTalonFX implements IndexerIO {

    public IndexerIOTalonFX() {
        super(HardwareDevices.indexerMotorID, true, NeutralMode.COAST, Amps.of(80));
    }
    
}
