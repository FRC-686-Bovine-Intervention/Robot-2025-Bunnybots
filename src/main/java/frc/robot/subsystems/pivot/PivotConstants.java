package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Angle;
import frc.robot.constants.RobotConstants;
import frc.util.mechanismUtil.GearRatio;

public class PivotConstants {
    public static final Transform3d pivotBase = new Transform3d(
        new Translation3d(
            Inches.zero(),
            Inches.zero(),
            Inches.zero()
        ),
        Rotation3d.kZero
    );

    public static final Pose2d pivotRobotSpace = new Pose2d(
        new Translation2d(
            PivotConstants.pivotBase.getMeasureX(),
            PivotConstants.pivotBase.getMeasureZ()
        ),
        Rotation2d.kZero
    );

    public static final GearRatio motorToMechanism = new GearRatio()
        .planetary(3)
        .gear(1)
        .gear(1)
        .axle()
        .gear(10)
        .gear(80)
        .axle()
    ;
    public static final GearRatio sensorToMechanism = new GearRatio()
        
    ;

    public static final Angle minAngle = Degrees.of(20);
    public static final Angle maxAngle = Degrees.of(115);

    /* Meters to Degrees */
    public static final InterpolatingDoubleTreeMap pivotAltitude = new InterpolatingDoubleTreeMap();
    static {
        pivotAltitude.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), Degrees.of(43.5-2).in(Degrees));
        pivotAltitude.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), Degrees.of(37.0-2).in(Degrees));
        pivotAltitude.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), Degrees.of(29.5-2).in(Degrees));
        pivotAltitude.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), Degrees.of(24.5-1).in(Degrees));
        pivotAltitude.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), Degrees.of(20.25).in(Degrees));
    }
}
