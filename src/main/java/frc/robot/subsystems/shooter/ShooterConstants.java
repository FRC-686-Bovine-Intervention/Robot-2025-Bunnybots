package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.RobotConstants;

public class ShooterConstants {
    public static final InterpolatingDoubleTreeMap highGoalTargetFlywheelVeloMPS = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetFlywheelVeloMPS.put(Meters.of(1.828).in(Meters), 20.0);
        highGoalTargetFlywheelVeloMPS.put(Meters.of(1.907).in(Meters), 20.0);
        highGoalTargetFlywheelVeloMPS.put(Meters.of(2.508).in(Meters), 20.0);
        highGoalTargetFlywheelVeloMPS.put(Meters.of(3.034).in(Meters), 20.0);
        highGoalTargetFlywheelVeloMPS.put(Meters.of(4.131).in(Meters), 20.0);
        highGoalTargetFlywheelVeloMPS.put(Meters.of(4.353).in(Meters), 25.0);
    }
    public static final InterpolatingDoubleTreeMap highGoalTargetPivotAltitudeRads = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetPivotAltitudeRads.put(Meters.of(1.828).in(Meters), Math.toRadians(25.0));
        highGoalTargetPivotAltitudeRads.put(Meters.of(1.907).in(Meters), Math.toRadians(22.5));
        highGoalTargetPivotAltitudeRads.put(Meters.of(2.508).in(Meters), Math.toRadians(20.0));
        highGoalTargetPivotAltitudeRads.put(Meters.of(3.034).in(Meters), Math.toRadians(15.0));
        highGoalTargetPivotAltitudeRads.put(Meters.of(4.131).in(Meters), Math.toRadians(12.5));
        highGoalTargetPivotAltitudeRads.put(Meters.of(4.353).in(Meters), Math.toRadians(12));
    }
    public static final InterpolatingDoubleTreeMap highGoalTargetDrivetrainOffsetRads = new InterpolatingDoubleTreeMap();
    static {
        highGoalTargetDrivetrainOffsetRads.put(Meters.of(1.828).in(Meters), 0.0);
        highGoalTargetDrivetrainOffsetRads.put(Meters.of(1.907).in(Meters), 0.0);   
        highGoalTargetDrivetrainOffsetRads.put(Meters.of(3.034).in(Meters), 0.0);
        highGoalTargetDrivetrainOffsetRads.put(Meters.of(4.131).in(Meters), 0.0);    
    }

    public static final InterpolatingDoubleTreeMap lowGoalTargetFlywheelVeloMPS = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(1.415).in(Meters), 4.0);
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(2.425).in(Meters), 5.5);
        lowGoalTargetFlywheelVeloMPS.put(Meters.of(4.312).in(Meters), 7.5);
    }
    public static final InterpolatingDoubleTreeMap lowGoalTargetPivotAltitudeRads = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetPivotAltitudeRads.put(Meters.of(1.415).in(Meters), Math.toRadians(20));
        lowGoalTargetPivotAltitudeRads.put(Meters.of(2.425).in(Meters), Math.toRadians(20));
        lowGoalTargetPivotAltitudeRads.put(Meters.of(4.312).in(Meters), Math.toRadians(20));
    }
    public static final InterpolatingDoubleTreeMap lowGoalTargetDrivetrainOffsetRads = new InterpolatingDoubleTreeMap();
    static {
        lowGoalTargetDrivetrainOffsetRads.put(Meters.of(1.828).in(Meters), 0.0);
        lowGoalTargetDrivetrainOffsetRads.put(Meters.of(1.907).in(Meters), 0.0);
        lowGoalTargetDrivetrainOffsetRads.put(Meters.of(3.034).in(Meters), 0.0);
        lowGoalTargetDrivetrainOffsetRads.put(Meters.of(4.131).in(Meters), 0.0);
    }
}
