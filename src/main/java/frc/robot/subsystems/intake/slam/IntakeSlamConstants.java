package frc.robot.subsystems.intake.slam;

import frc.util.mechanismUtil.GearRatio;

public class IntakeSlamConstants {
    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(5)
        .planetary(5)
        .sprocket(20).sprocket(20)
        .sprocket(20).sprocket(20)
    ;

    public static final boolean calibrationSensorInverted = false;
}
