package frc.robot.subsystems.intake.slam;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.mechanismUtil.GearRatio;

public class IntakeSlamConstants {
    public static final Transform3d slamBase = new Transform3d(
        new Translation3d(
            Inches.zero(),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );

    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(5)
        .planetary(5)
        .sprocket(20)
        .sprocket(20)
        .sprocket(20)
        .sprocket(20)
    ;

    public static final boolean calibrationSensorInverted = false;
}
