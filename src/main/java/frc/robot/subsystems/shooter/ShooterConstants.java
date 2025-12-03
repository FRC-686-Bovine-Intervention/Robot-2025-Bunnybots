package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.RobotConstants;

public class ShooterConstants {
    public static final InterpolatingDoubleTreeMap highGoalTargetFlywheelVeloMPS = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetFlywheelVeloMPS.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        highGoalTargetFlywheelVeloMPS.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        highGoalTargetFlywheelVeloMPS.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        highGoalTargetFlywheelVeloMPS.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        highGoalTargetFlywheelVeloMPS.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }
    public static final InterpolatingDoubleTreeMap highGoalTargetPivotAltitudeRads = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetPivotAltitudeRads.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        highGoalTargetPivotAltitudeRads.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        highGoalTargetPivotAltitudeRads.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        highGoalTargetPivotAltitudeRads.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        highGoalTargetPivotAltitudeRads.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }
    public static final InterpolatingDoubleTreeMap highGoalTargetDrivetrainOffsetRads = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetDrivetrainOffsetRads.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        highGoalTargetDrivetrainOffsetRads.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        highGoalTargetDrivetrainOffsetRads.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        highGoalTargetDrivetrainOffsetRads.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        highGoalTargetDrivetrainOffsetRads.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        highGoalTargetDrivetrainOffsetRads.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }

    public static final InterpolatingDoubleTreeMap lowGoalTargetFlywheelVeloMPS = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetFlywheelVeloMPS.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        lowGoalTargetFlywheelVeloMPS.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        lowGoalTargetFlywheelVeloMPS.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        lowGoalTargetFlywheelVeloMPS.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        lowGoalTargetFlywheelVeloMPS.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }
    public static final InterpolatingDoubleTreeMap lowGoalTargetPivotAltitudeRads = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetPivotAltitudeRads.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        lowGoalTargetPivotAltitudeRads.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        lowGoalTargetPivotAltitudeRads.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        lowGoalTargetPivotAltitudeRads.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        lowGoalTargetPivotAltitudeRads.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }
    public static final InterpolatingDoubleTreeMap lowGoalTargetDrivetrainOffsetRads = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetDrivetrainOffsetRads.put(Centimeters.of(118).plus(RobotConstants.robotLength.div(2)).in(Meters), 14.5);
        lowGoalTargetDrivetrainOffsetRads.put(Centimeters.of(190).plus(RobotConstants.robotLength.div(2)).in(Meters), 16.0);
        lowGoalTargetDrivetrainOffsetRads.put(Centimeters.of(280).plus(RobotConstants.robotLength.div(2)).in(Meters), 21.0);
        lowGoalTargetDrivetrainOffsetRads.put(Centimeters.of(370).plus(RobotConstants.robotLength.div(2)).in(Meters), 24.5);
        lowGoalTargetDrivetrainOffsetRads.put(Centimeters.of(500).plus(RobotConstants.robotLength.div(2)).in(Meters), 31.5);
    }
}
