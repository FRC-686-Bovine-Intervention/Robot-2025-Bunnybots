package frc.robot.subsystems.shooter.flywheel;

import static edu.wpi.first.units.Units.Inches;

import frc.util.mechanismUtil.GearRatio;
import frc.util.mechanismUtil.LinearRelation;

public final class FlywheelConstants {
    public static final GearRatio motorToMechanism = new GearRatio()
        .sprocket(1)
        .sprocket(1)
    ;
        
    public static final LinearRelation flywheel = LinearRelation.wheelDiameter(Inches.of(4));
}
