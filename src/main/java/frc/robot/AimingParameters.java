package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.misc.MathExtraUtil;

public class AimingParameters {
    private static Translation3d aimPoint = new Translation3d();
    private static Pose2d shotPose = new Pose2d(1,0,new Rotation2d());
    private static ChassisSpeeds shotSpeeds = new ChassisSpeeds();
    private static double effectiveDistance = Meters.of(0).in(Meters);
    private static double pivotAltitude = Degrees.of(0).in(Degrees);
    private static double targetShooterSpeed = MetersPerSecond.of(0).in(MetersPerSecond);
    private static double minimumShooterSpeed = MetersPerSecond.of(0).in(MetersPerSecond);
    private static final RobotState robotState = RobotState.getInstance();

    public static Pose2d shotPose() {
        return shotPose;
    }
    public static ChassisSpeeds shotSpeeds() {
        return shotSpeeds;
    }
    public static double pivotAltitude() {
        return pivotAltitude;
    }
    public static double targetShooterSpeed() {
        return targetShooterSpeed;
    }
    public static double minimumShooterSpeed() {
        return minimumShooterSpeed;
    }
    
    public static void setFrom(Drive drive) {
        setFrom(robotState.getEstimatedGlobalPose().getTranslation(), drive.getFieldMeasuredSpeeds());
    }
    public static void setFrom(Drive drive, Translation3d aimPoint) {
        setFrom(robotState.getEstimatedGlobalPose().getTranslation(), drive.getFieldMeasuredSpeeds(), aimPoint);
    }
    public static void setFrom(Translation2d robotPos) {
        setFrom(robotPos, new ChassisSpeeds());
    }
    public static void setFrom(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed) {
        setFrom(robotPos, fieldRelativeSpeed, FieldConstants.highGoalAimPoint.getOurs());
    }
    public static void setFrom(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation3d aimPoint) {
        calculate(robotPos, fieldRelativeSpeed, aimPoint);
    }

    private static final LoggedTunable<Time> lookaheadTime = LoggedTunable.from("Aiming/Lookahead Seconds", Seconds::of, 0.035);
    private static final LoggedTunable<Distance> azimuthTolerance = LoggedTunable.from("Aiming/Tolerance/Azimuth", Centimeters::of, 100);
    private static final LoggedTunable<Distance> altitudeTolerance = LoggedTunable.from("Aiming/Tolerance/Altitude", Centimeters::of, 46);

    public static final boolean withinAzimuthTolerance(Pose2d robotPose) {
        var blueRobotPose = AllianceFlipUtil.apply(robotPose);
        var blueAimPoint = AllianceFlipUtil.apply(AimingParameters.aimPoint);
        var aimPointToRobot = blueRobotPose.getTranslation().minus(blueAimPoint.toTranslation2d()).getAngle();
        var horizontalTolerance = Math.abs(aimPointToRobot.getCos()) * AimingParameters.azimuthTolerance.get().in(Meters) * 0.5;
        var azimuthTolerance = Math.atan2(horizontalTolerance, effectiveDistance);
        var result = MathUtil.isNear(AimingParameters.shotPose().getRotation().getRadians(), robotPose.getRotation().getRadians(), azimuthTolerance);
        Logger.recordOutput("AimingTolerance/Azimuth", result);
        return result;
    }

    public static final boolean withinAltitudeTolerance(double altitude) {
        var pivotDist = effectiveDistance - Pivot.robotToPivotTranslation.getX();
        var targetHeight = aimPoint.getZ() - Pivot.robotToPivotTranslation.getZ();
        var pivotToTargetDist = Math.hypot(pivotDist, targetHeight);
        var verticalTolerance = Math.abs(targetHeight / pivotToTargetDist) * AimingParameters.altitudeTolerance.get().in(Meters) * 0.5;
        var altitudeTolerance = Math.atan2(verticalTolerance, pivotToTargetDist - (targetHeight / pivotDist * verticalTolerance));
        Logger.recordOutput("AimingTolerance/pivotDist", pivotDist);
        Logger.recordOutput("AimingTolerance/pivotToTargetDist", pivotToTargetDist);
        Logger.recordOutput("AimingTolerance/verticalTolerance", verticalTolerance);
        Logger.recordOutput("AimingTolerance/altitudeTolerance", altitudeTolerance);
        var result = MathExtraUtil.isNear(AimingParameters.pivotAltitude(), altitude, Radians.of(altitudeTolerance));
        Logger.recordOutput("AimingTolerance/Altitude", result);
        return result;
    }
    
    private static void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation3d aimPoint) {
        var predictedRobotPos = robotPos.plus(GeomUtil.translationFromSpeeds(fieldRelativeSpeed).times(lookaheadTime.get().in(Seconds)));
        // var velocityTowardsSpeaker = robotPos.minus(aimPoint).toVector().unit().dot(VecBuilder.fill(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond));
        // var timeToAimPoint = robotPos.getDistance(aimPoint) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
        // var chassisOffset = fieldRelativeSpeed.times(timeToAimPoint);
        // var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
        // var pointTo = aimPoint.minus(translationalOffset);
        var driveAzimuth = aimPoint.toTranslation2d().minus(predictedRobotPos).getAngle();
        var predictedDistToAimPoint = aimPoint.toTranslation2d().getDistance(predictedRobotPos);
        AimingParameters.aimPoint = aimPoint;
        AimingParameters.shotPose = new Pose2d(robotPos, driveAzimuth);
        AimingParameters.shotSpeeds = new ChassisSpeeds(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond, 0);
        AimingParameters.effectiveDistance = predictedDistToAimPoint;
        AimingParameters.pivotAltitude = PivotConstants.pivotAltitude.get(predictedDistToAimPoint);
        AimingParameters.targetShooterSpeed = ShooterConstants.targetShooterSpeed.get(predictedDistToAimPoint);
        AimingParameters.minimumShooterSpeed = ShooterConstants.minimumShooterSpeed.get(predictedDistToAimPoint);
        Logger.recordOutput("AimingParameters/Aim Point", AimingParameters.aimPoint);
        Logger.recordOutput("AimingParameters/Shot Pose", shotPose());
        Logger.recordOutput("AimingParameters/Shot Speeds", shotSpeeds());
        Logger.recordOutput("AimingParameters/Pivot Altitude", pivotAltitude());
        Logger.recordOutput("AimingParameters/Target Shooter Speed", targetShooterSpeed());
        Logger.recordOutput("AimingParameters/Minimum Shooter Speed", minimumShooterSpeed());
        Logger.recordOutput("AimingParameters/Predicted Pose", new Pose2d(predictedRobotPos, driveAzimuth));
        Logger.recordOutput("AimingParameters/Effective Distance", effectiveDistance);
        Logger.recordOutput("AimingParameters/Shooter Mech3d", Pivot.getRobotToPivot(Math.toRadians(pivotAltitude())));
    }
}
