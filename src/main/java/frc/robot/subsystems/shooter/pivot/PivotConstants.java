package frc.robot.subsystems.shooter.pivot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import frc.util.mechanismUtil.GearRatio;

public class PivotConstants {
    public static final Angle minAngle = Degrees.of(0);
    public static final Angle maxAngle = Degrees.of(41);

    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(5)
        .gear(1).gear(1).axle()
        .gear(10).gear(213).axle()
    ;

    public static final Transform3d pivotBase = new Transform3d(
        new Translation3d(
            Inches.zero(),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );
}
