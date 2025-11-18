package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.geometry.GeomUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class AimingParameters {
    private static Translation3d aimPoint = new Translation3d();
    private static Pose2d shotPose = new Pose2d(1,0,new Rotation2d());
    private static ChassisSpeeds shotSpeeds = new ChassisSpeeds();
    private static double effectiveDistanceMeters = Meters.of(0).in(Meters);
    private static double pivotAltitudeDegs = Degrees.of(0).in(Degrees);
    private static double targetShooterSpeedMPS = MetersPerSecond.of(0).in(MetersPerSecond);
    private static double minimumShooterSpeedMPS = MetersPerSecond.of(0).in(MetersPerSecond);
    private static final RobotState robotState = RobotState.getInstance();
    private static double pivotOffsetDegs = 0.0;
    private static double shooterOffsetMPS = 0.0;

    public static Pose2d shotPose() {
        return shotPose;
    }
    public static ChassisSpeeds shotSpeeds() {
        return shotSpeeds;
    }
    public static double pivotAltitude() {
        return pivotAltitudeDegs;
    }
    public static double targetShooterSpeed() {
        return targetShooterSpeedMPS;
    }
    public static double minimumShooterSpeed() {
        return minimumShooterSpeedMPS;
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
        setFrom(robotPos, fieldRelativeSpeed, RobotState.getAimPoint(robotState.getSelectedShootTarget()));
    }
    public static void setFrom(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation3d aimPoint) {
        calculate(robotPos, fieldRelativeSpeed, aimPoint);
    }

    public static void applyPivotOffset(double degs) {
        pivotOffsetDegs += degs;
    }
    public static void applyShooterOffset(double mps) {
        shooterOffsetMPS += mps;
    }

    private static final LoggedTunable<Time> lookaheadTime = LoggedTunable.from("Aiming/Lookahead Seconds", Seconds::of, 0.035);
    private static final LoggedTunable<Distance> azimuthTolerance = LoggedTunable.from("Aiming/Tolerance/Azimuth", Centimeters::of, 100);
    private static final LoggedTunable<Distance> altitudeDegsTolerance = LoggedTunable.from("Aiming/Tolerance/Altitude", Centimeters::of, 46);

    public static final boolean withinAzimuthTolerance(Pose2d robotPose) {
        var blueRobotPose = AllianceFlipUtil.apply(robotPose);
        var blueAimPoint = AllianceFlipUtil.apply(AimingParameters.aimPoint);
        var aimPointToRobot = blueRobotPose.getTranslation().minus(blueAimPoint.toTranslation2d()).getAngle();
        var horizontalTolerance = Math.abs(aimPointToRobot.getCos()) * AimingParameters.azimuthTolerance.get().in(Meters) * 0.5;
        var azimuthTolerance = Math.atan2(horizontalTolerance, effectiveDistanceMeters);
        var result = MathUtil.isNear(AimingParameters.shotPose().getRotation().getRadians(), robotPose.getRotation().getRadians(), azimuthTolerance);
        Logger.recordOutput("AimingTolerance/Azimuth", result);
        return result;
    }

    public static final boolean withinAltitudeTolerance(double altitudeDegs) {
        var pivotDist = effectiveDistanceMeters - PivotConstants.pivotBase.getX();
        var targetHeight = aimPoint.getZ() - PivotConstants.pivotBase.getZ();
        var pivotToTargetDist = Math.hypot(pivotDist, targetHeight);
        var verticalTolerance = Math.abs(targetHeight / pivotToTargetDist) * AimingParameters.altitudeDegsTolerance.get().in(Meters) * 0.5;
        var altitudeDegsTolerance = Math.atan2(verticalTolerance, pivotToTargetDist - (targetHeight / pivotDist * verticalTolerance));
        Logger.recordOutput("AimingTolerance/pivotDist", pivotDist);
        Logger.recordOutput("AimingTolerance/pivotToTargetDist", pivotToTargetDist);
        Logger.recordOutput("AimingTolerance/verticalTolerance", verticalTolerance);
        Logger.recordOutput("AimingTolerance/altitudeDegsTolerance", altitudeDegsTolerance);
        var result = MathUtil.isNear(AimingParameters.pivotAltitude(), altitudeDegs, altitudeDegsTolerance);
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
        AimingParameters.effectiveDistanceMeters = predictedDistToAimPoint;
        AimingParameters.pivotAltitudeDegs = PivotConstants.pivotAltitude.get(predictedDistToAimPoint) + pivotOffsetDegs;
        AimingParameters.targetShooterSpeedMPS = ShooterConstants.targetShooterSpeed.get(predictedDistToAimPoint) + shooterOffsetMPS;
        AimingParameters.minimumShooterSpeedMPS = ShooterConstants.minimumShooterSpeed.get(predictedDistToAimPoint) + shooterOffsetMPS;
        Logger.recordOutput("AimingParameters/Aim Point", AimingParameters.aimPoint);
        Logger.recordOutput("AimingParameters/Shot Pose", shotPose());
        Logger.recordOutput("AimingParameters/Shot Speeds", shotSpeeds());
        Logger.recordOutput("AimingParameters/Pivot Altitude", pivotAltitude());
        Logger.recordOutput("AimingParameters/Target Shooter Speed", targetShooterSpeed());
        Logger.recordOutput("AimingParameters/Minimum Shooter Speed", minimumShooterSpeed());
        Logger.recordOutput("AimingParameters/Predicted Pose", new Pose2d(predictedRobotPos, driveAzimuth));
        Logger.recordOutput("AimingParameters/Effective Distance",  effectiveDistanceMeters);
        Logger.recordOutput("AimingParameters/Shooter Mech3d", Pivot.getRobotToPivot(Math.toRadians(pivotAltitude())));
    }
}