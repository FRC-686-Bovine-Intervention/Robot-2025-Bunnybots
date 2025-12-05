package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.FieldConstants.Goals.Goal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Shooter {
    public final Pivot pivot;
    public final Flywheel flywheel;
    public final Drive drive;
    private final SubsystemBase aimingResource;

    private static final LoggedTunable<Time> lookaheadTime = LoggedTunable.from("Shooter/Aiming/Lookahead Seconds", Seconds::of, 0.035);
    private static final LoggedTunable<Distance> azimuthTolerance = LoggedTunable.from("Shooter/Aiming/Tolerance/Azimuth", Centimeters::of, 100);
    private static final LoggedTunable<Distance> altitudeDegsTolerance = LoggedTunable.from("Shooter/Aiming/Tolerance/Altitude", Centimeters::of, 46);
    private static final LoggedTunable<Angle> customAzimuthOffset = LoggedTunable.from("Shooter/Aiming/Custom Azimuth Offset", Radians::of, 0.0);

    public Shooter(Pivot pivot, Flywheel flywheel, Drive drive) {
        this.pivot = pivot;
        this.flywheel = flywheel;
        this.drive = drive;
        this.aimingResource = new SubsystemBase("Shooter/Aiming") {};
    }

    private Translation3d aimPoint;
    // private Pose2d shotPose;
    // private ChassisSpeeds shotSpeeds;
    private double effectiveDistanceMeters;
    private double targetPivotAltitudeRads;
    private double targetFlywheelVeloMPS;
    private double targetDriveHeadingRads;
    private double rawTargetDriveHeadingRads;
    // private double minimumShooterSpeedMPS;
    public double getTargetDriveHeadingRads() {
        return this.targetDriveHeadingRads;
    }

    public Command aim(Pose2d robotPose, ChassisSpeeds fieldSpeeds, Goal target) {
        return this.aim(() -> robotPose, () -> fieldSpeeds, () -> target);
    }
    public Command aim(Supplier<Pose2d> robotPose, Supplier<ChassisSpeeds> fieldSpeeds, Supplier<Goal> target) {
        final var shooter = this;
        return new Command() {
            {
                this.setName("Aim");
                this.addRequirements(shooter.aimingResource);
            }

            @Override
            public void initialize() {
                shooter.calculate(robotPose.get().getTranslation(), fieldSpeeds.get(), target.get());
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };
    }

    public Command aimPivot() {
        return this.pivot.genAngleCommand(
            "Aim",
            () -> this.targetPivotAltitudeRads
        );
    }
    public Command aimFlywheel() {
        return this.flywheel.genSurfaceVeloCommand(
            "Aim",
            () -> this.targetFlywheelVeloMPS
        );
    }
    public Command aimAzimuth() {
        return this.drive.rotationalSubsystem.pidControlledHeading(
            () -> new Rotation2d(this.targetDriveHeadingRads)
        ).withName("Aim Azimuth");
    }

    public Command customAimAzimuth() {
        return this.drive.rotationalSubsystem.pidControlledHeading(
            () -> new Rotation2d(this.rawTargetDriveHeadingRads + customAzimuthOffset.get().in(Radians))
        );
    }

    private void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeeds, Goal goal) {
        this.calculate(
            robotPos,
            fieldRelativeSpeeds,
            goal.centerPoint,
            goal.type.select(ShooterConstants.highGoalTargetPivotAltitudeRads, ShooterConstants.lowGoalTargetPivotAltitudeRads),
            goal.type.select(ShooterConstants.highGoalTargetFlywheelVeloMPS, ShooterConstants.lowGoalTargetFlywheelVeloMPS),
            goal.type.select(ShooterConstants.highGoalTargetDrivetrainOffsetRads, ShooterConstants.lowGoalTargetDrivetrainOffsetRads)
        );
    }
    private void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeeds, Translation3d aimPoint, InterpolatingDoubleTreeMap pivotAltitudeMap, InterpolatingDoubleTreeMap flywheelVeloMap, InterpolatingDoubleTreeMap driveAzimuthMap) {
        this.aimPoint = aimPoint;

        var predictedX = robotPos.getX() + fieldRelativeSpeeds.vxMetersPerSecond * lookaheadTime.get().in(Seconds);
        var predictedY = robotPos.getY() + fieldRelativeSpeeds.vyMetersPerSecond * lookaheadTime.get().in(Seconds);

        var predictedToTargetX = this.aimPoint.getX() - predictedX;
        var predictedToTargetY = this.aimPoint.getY() - predictedY;

        this.rawTargetDriveHeadingRads = Math.atan2(predictedToTargetY, predictedToTargetX);
        this.targetDriveHeadingRads = rawTargetDriveHeadingRads + driveAzimuthMap.get(this.effectiveDistanceMeters);
        this.effectiveDistanceMeters = Math.hypot(predictedToTargetX, predictedToTargetY);
        Logger.recordOutput("Shooter/Aiming/Effective Distance", this.effectiveDistanceMeters);

        this.targetPivotAltitudeRads = pivotAltitudeMap.get(this.effectiveDistanceMeters);
        this.targetFlywheelVeloMPS = flywheelVeloMap.get(this.effectiveDistanceMeters);

        Logger.recordOutput("Shooter/Aiming/Aim Point", this.aimPoint);
        Logger.recordOutput("Shooter/Aiming/Shot Pose", new Pose2d(
            new Translation2d(
                predictedX,
                predictedY
            ),
            Rotation2d.fromRadians(
                this.targetDriveHeadingRads
            )
        ));
    }

    // public final boolean withinAzimuthTolerance(Pose2d robotPose) {
    //     var blueRobotPose = AllianceFlipUtil.apply(robotPose);
    //     var blueAimPoint = AllianceFlipUtil.apply(AimingParameters.aimPoint);
    //     var aimPointToRobot = blueRobotPose.getTranslation().minus(blueAimPoint.toTranslation2d()).getAngle();
    //     var horizontalTolerance = Math.abs(aimPointToRobot.getCos()) * AimingParameters.azimuthTolerance.get().in(Meters) * 0.5;
    //     var azimuthTolerance = Math.atan2(horizontalTolerance, effectiveDistanceMeters);
    //     var result = MathUtil.isNear(AimingParameters.shotPose().getRotation().getRadians(), robotPose.getRotation().getRadians(), azimuthTolerance);
    //     Logger.recordOutput("AimingTolerance/Azimuth", result);
    //     return result;
    // }

    // public final boolean withinAltitudeTolerance(double altitudeDegs) {
    //     var pivotDist = effectiveDistanceMeters - PivotConstants.pivotBase.getX();
    //     var targetHeight = aimPoint.getZ() - PivotConstants.pivotBase.getZ();
    //     var pivotToTargetDist = Math.hypot(pivotDist, targetHeight);
    //     var verticalTolerance = Math.abs(targetHeight / pivotToTargetDist) * AimingParameters.altitudeDegsTolerance.get().in(Meters) * 0.5;
    //     var altitudeDegsTolerance = Math.atan2(verticalTolerance, pivotToTargetDist - (targetHeight / pivotDist * verticalTolerance));
    //     Logger.recordOutput("AimingTolerance/pivotDist", pivotDist);
    //     Logger.recordOutput("AimingTolerance/pivotToTargetDist", pivotToTargetDist);
    //     Logger.recordOutput("AimingTolerance/verticalTolerance", verticalTolerance);
    //     Logger.recordOutput("AimingTolerance/altitudeDegsTolerance", altitudeDegsTolerance);
    //     var result = MathUtil.isNear(AimingParameters.pivotAltitude(), altitudeDegs, altitudeDegsTolerance);
    //     Logger.recordOutput("AimingTolerance/Altitude", result);
    //     return result;
    // }
    
    // private void calculate(Translation2d robotPos, ChassisSpeeds fieldRelativeSpeed, Translation3d aimPoint) {
    //     var predictedRobotPos = robotPos.plus(GeomUtil.translationFromSpeeds(fieldRelativeSpeed).times(lookaheadTime.get().in(Seconds)));
    //     // var velocityTowardsSpeaker = robotPos.minus(aimPoint).toVector().unit().dot(VecBuilder.fill(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond));
    //     // var timeToAimPoint = robotPos.getDistance(aimPoint) / (ShooterConstants.exitVelocity + velocityTowardsSpeaker);
    //     // var chassisOffset = fieldRelativeSpeed.times(timeToAimPoint);
    //     // var translationalOffset = new Translation2d(chassisOffset.vxMetersPerSecond, chassisOffset.vyMetersPerSecond);
    //     // var pointTo = aimPoint.minus(translationalOffset);
    //     var driveAzimuth = aimPoint.toTranslation2d().minus(predictedRobotPos).getAngle();
    //     var predictedDistToAimPoint = aimPoint.toTranslation2d().getDistance(predictedRobotPos);
    //     AimingParameters.aimPoint = aimPoint;
    //     AimingParameters.shotPose = new Pose2d(robotPos, driveAzimuth);
    //     AimingParameters.shotSpeeds = new ChassisSpeeds(fieldRelativeSpeed.vxMetersPerSecond, fieldRelativeSpeed.vyMetersPerSecond, 0);
    //     AimingParameters.effectiveDistanceMeters = predictedDistToAimPoint;
    //     AimingParameters.pivotAltitudeDegs = ShooterConstants.pivotAltitude.get(predictedDistToAimPoint) + pivotOffsetDegs;
    //     AimingParameters.targetShooterSpeedMPS = ShooterConstants.targetShooterVelo.get(predictedDistToAimPoint) + shooterOffsetMPS;
    //     AimingParameters.minimumShooterSpeedMPS = ShooterConstants.minimumShooterSpeed.get(predictedDistToAimPoint) + shooterOffsetMPS;
    //     Logger.recordOutput("AimingParameters/Aim Point", AimingParameters.aimPoint);
    //     Logger.recordOutput("AimingParameters/Shot Pose", shotPose());
    //     Logger.recordOutput("AimingParameters/Shot Speeds", shotSpeeds());
    //     Logger.recordOutput("AimingParameters/Pivot Altitude", pivotAltitude());
    //     Logger.recordOutput("AimingParameters/Target Shooter Speed", targetShooterSpeed());
    //     Logger.recordOutput("AimingParameters/Minimum Shooter Speed", minimumShooterSpeed());
    //     Logger.recordOutput("AimingParameters/Predicted Pose", new Pose2d(predictedRobotPos, driveAzimuth));
    //     Logger.recordOutput("AimingParameters/Effective Distance",  effectiveDistanceMeters);
    //     Logger.recordOutput("AimingParameters/Shooter Mech3d", Pivot.getRobotToPivot(Math.toRadians(pivotAltitude())));
    // }
}
