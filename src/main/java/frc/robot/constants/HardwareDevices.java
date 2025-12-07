package frc.robot.constants;

import frc.util.hardwareID.can.CANBus;
import frc.util.hardwareID.can.CANDevice;
import frc.util.hardwareID.rioPorts.DIOPort;
import frc.util.hardwareID.rioPorts.PWMPort;

public class HardwareDevices {
    /*
     * PDH Ports
     * 10: Left Pivot           9: Mini Power Module
     * 11:                      8: Right Pivot
     * 12: Elevator             7:
     * 13: Cage                 6:
     * 14: Wrist                5: 
     * 15: Intake               4: Chain Winch
     * 16: Back Left Turn       3: Back Right Turn
     * 17: Back Left Drive      2: Back Right Drive
     * 18: Front Left Turn      1: Front Right Turn
     * 19: Front Left Drive     0: Front Right Drive
     * 20: RoboRIO
     * 21: Radio DC
     * 22: Radio POE
     * 23: 
     */
    public static final CANBus rio = CANBus.rio();
    public static final CANBus canivore = CANBus.newBus("canivore");

    // Drive
    public static final CANDevice pigeonID = canivore.id(0);
    // | Front Left
    public static final CANDevice frontLeftDriveMotorID = canivore.id(1);
    public static final CANDevice frontLeftTurnMotorID = rio.id(1);
    // | Front Right
    public static final CANDevice frontRightDriveMotorID = canivore.id(2);
    public static final CANDevice frontRightTurnMotorID = rio.id(2);
    // | Back Left
    public static final CANDevice backLeftDriveMotorID = canivore.id(3);
    public static final CANDevice backLeftTurnMotorID = rio.id(3);
    // | Back Right
    public static final CANDevice backRightDriveMotorID = canivore.id(4);
    public static final CANDevice backRightTurnMotorID = rio.id(4);

    // Shooter
    // | Pivot
    public static final CANDevice pivotMotorID = canivore.id(7);
    // | Flywheel
    public static final CANDevice flywheelLeftMotorID = canivore.id(5);
    public static final CANDevice flywheelRightMotorID = canivore.id(6);
    // | CANDi
    public static final CANDevice candiID = canivore.id(0);

    // Intake
    // | Slam
    public static final CANDevice intakeSlamMotorID = canivore.id(8);
    public static final CANDevice intakeSlamEncoderID = canivore.id(8);
    // | Rollers
    public static final CANDevice intakeRollerMotorID = canivore.id(9);

    // Rollers
    // | Kicker
    public static final CANDevice kickerMotorID = canivore.id(10);
    // | Indexer
    public static final CANDevice indexerLeftMotorID = rio.id(5);
    public static final CANDevice indexerRightMotorID = rio.id(6);

    // RIO
    public static final PWMPort ledPort = PWMPort.port(0);
    public static final DIOPort button1 = DIOPort.port(0);
    public static final DIOPort button2 = DIOPort.port(0);
    public static final DIOPort button3 = DIOPort.port(0);
}
