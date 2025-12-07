package frc.util.flipping;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Optional;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.FieldConstants;
import frc.util.geometry.GeomUtil;

public class AllianceFlipUtil {
    public static enum FieldFlipType {
        CenterPointRotation,
        CenterLineMirror,
        XenterLineMirror,
    }
    public static final FieldFlipType defaultFlipType = FieldFlipType.CenterLineMirror;

    public static <T extends AllianceFlippable<T>> T apply(T flippable) {
        return apply(flippable, defaultFlipType);
    }
    public static <T extends AllianceFlippable<T>> T apply(T flippable, FieldFlipType flipType) {
        if (!shouldFlip()) return flippable;
        return flip(flippable, flipType);
    }
    public static <T extends AllianceFlippable<T>> T flip(T flippable) {
        return flip(flippable, defaultFlipType);
    }
    public static <T extends AllianceFlippable<T>> T flip(T flippable, FieldFlipType flipType) {
        return flippable.flip(flipType);
    }
    
    public static Translation2d apply(Translation2d translation) {
        return apply(translation, defaultFlipType);
    }
    public static Translation2d apply(Translation2d translation, FieldFlipType flipType) {
        if (!shouldFlip()) return translation;
        return flip(translation, flipType);
    }
    public static Translation2d flip(Translation2d translation) {
        return flip(translation, defaultFlipType);
    }
    public static Translation2d flip(Translation2d translation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new Translation2d(FieldConstants.fieldLength.in(Meters) - translation.getX(), FieldConstants.fieldWidth.in(Meters) - translation.getY());
            case CenterLineMirror:      return new Translation2d(FieldConstants.fieldLength.in(Meters) - translation.getX(), translation.getY());
            case XenterLineMirror:      return new Translation2d(translation.getX(), FieldConstants.fieldWidth.in(Meters) - translation.getY());
        }
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return apply(rotation, defaultFlipType);
    }
    public static Rotation2d apply(Rotation2d rotation, FieldFlipType flipType) {
        if (!shouldFlip()) return rotation;
        return flip(rotation, flipType);
    }
    public static Rotation2d flip(Rotation2d rotation) {
        return flip(rotation, defaultFlipType);
    }
    public static Rotation2d flip(Rotation2d rotation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new Rotation2d(-rotation.getCos(), -rotation.getSin());
            case CenterLineMirror:      return new Rotation2d(-rotation.getCos(), +rotation.getSin());
            case XenterLineMirror:      return new Rotation2d(+rotation.getCos(), -rotation.getSin());
        }
    }

    public static Pose2d apply(Pose2d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Pose2d apply(Pose2d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return flip(pose, flipType);
    }
    public static Pose2d flip(Pose2d pose) {
        return flip(pose, defaultFlipType);
    }
    public static Pose2d flip(Pose2d pose, FieldFlipType flipType) {
        return new Pose2d(flip(pose.getTranslation(), flipType), flip(pose.getRotation(), flipType));
    }

    public static Transform2d apply(Transform2d transform) {
        return apply(transform, defaultFlipType);
    }
    public static Transform2d apply(Transform2d transform, FieldFlipType flipType) {
        if (!shouldFlip()) return transform;
        return flip(transform, flipType);
    }
    public static Transform2d flip(Transform2d transform) {
        return flip(transform, defaultFlipType);
    }
    public static Transform2d flip(Transform2d transform, FieldFlipType flipType) {
        return new Transform2d(flip(transform.getTranslation(), flipType), flip(transform.getRotation(), flipType));
    }

    public static Translation3d apply(Translation3d translation) {
        return apply(translation, defaultFlipType);
    }
    public static Translation3d apply(Translation3d translation, FieldFlipType flipType) {
        if (!shouldFlip()) return translation;
        return flip(translation, flipType);
    }
    public static Translation3d flip(Translation3d translation) {
        return flip(translation, defaultFlipType);
    }
    public static Translation3d flip(Translation3d translation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new Translation3d(FieldConstants.fieldLength.in(Meters) - translation.getX(), FieldConstants.fieldWidth.in(Meters) - translation.getY(), translation.getZ());
            case CenterLineMirror:      return new Translation3d(FieldConstants.fieldLength.in(Meters) - translation.getX(), translation.getY(), translation.getZ());
            case XenterLineMirror:      return new Translation3d(translation.getX(), FieldConstants.fieldWidth.in(Meters) - translation.getY(), translation.getZ());
        }
    }
    
    public static Rotation3d apply(Rotation3d rotation) {
        return apply(rotation, defaultFlipType);
    }
    public static Rotation3d apply(Rotation3d rotation, FieldFlipType flipType) {
        if (!shouldFlip()) return rotation;
        return flip(rotation, flipType);
    }
    public static Rotation3d flip(Rotation3d rotation) {
        return flip(rotation, defaultFlipType);
    }
    public static Rotation3d flip(Rotation3d rotation, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return rotation.rotateBy(GeomUtil.rotate180Transform3d.getRotation());
            case CenterLineMirror:      throw new UnsupportedOperationException("Rotation3d flipping for CenterLineMirror not implemented yet");
            case XenterLineMirror:      throw new UnsupportedOperationException("Rotation3d flipping for XenterLineMirror not implemented yet");
        }
    }
    
    public static Pose3d apply(Pose3d pose) {
        return apply(pose, defaultFlipType);
    }
    public static Pose3d apply(Pose3d pose, FieldFlipType flipType) {
        if (!shouldFlip()) return pose;
        return flip(pose, flipType);
    }
    public static Pose3d flip(Pose3d pose) {
        return flip(pose, defaultFlipType);
    }
    public static Pose3d flip(Pose3d pose, FieldFlipType flipType) {
        return new Pose3d(flip(pose.getTranslation(), flipType), flip(pose.getRotation(), flipType));
    }
    
    public static Transform3d apply(Transform3d transform) {
        return apply(transform, defaultFlipType);
    }
    public static Transform3d apply(Transform3d transform, FieldFlipType flipType) {
        if (!shouldFlip()) return transform;
        return flip(transform, flipType);
    }
    public static Transform3d flip(Transform3d transform) {
        return flip(transform, defaultFlipType);
    }
    public static Transform3d flip(Transform3d transform, FieldFlipType flipType) {
        return new Transform3d(flip(transform.getTranslation(), flipType), flip(transform.getRotation(), flipType));
    }

    public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds) {
        return applyFieldRelative(speeds, defaultFlipType);
    }
    public static ChassisSpeeds applyFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
        if (!shouldFlip()) return speeds;
        return flipFieldRelative(speeds, flipType);
    }
    public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds) {
        return flipFieldRelative(speeds, defaultFlipType);
    }
    public static ChassisSpeeds flipFieldRelative(ChassisSpeeds speeds, FieldFlipType flipType) {
        switch (flipType) {
            default:
            case CenterPointRotation:   return new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, +speeds.omegaRadiansPerSecond);
            case CenterLineMirror:      return new ChassisSpeeds(-speeds.vxMetersPerSecond, +speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
            case XenterLineMirror:      return new ChassisSpeeds(+speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, -speeds.omegaRadiansPerSecond);
        }
    }

    public static SwerveSample apply(SwerveSample sample) {
        return apply(sample, defaultFlipType);
    }
    public static SwerveSample apply(SwerveSample sample, FieldFlipType flipType) {
        if (!shouldFlip()) return sample;
        return flip(sample, flipType);
    }
    public static SwerveSample flip(SwerveSample sample) {
        return flip(sample, defaultFlipType);
    }
    public static SwerveSample flip(SwerveSample sample, FieldFlipType flipType) {
        var headingCos = Math.cos(sample.heading);
        var headingSin = Math.sin(sample.heading);
        switch (flipType) {
            default:
            case CenterPointRotation: return new SwerveSample(
                sample.t,
                FieldConstants.fieldLength.in(Meters) - sample.x,
                FieldConstants.fieldWidth.in(Meters) - sample.y,
                Math.atan2(-headingSin, -headingCos),
                -sample.vx,
                -sample.vy,
                +sample.omega,
                -sample.ax,
                -sample.ay,
                +sample.alpha,
                new double[] {-sample.moduleForcesX()[0], -sample.moduleForcesX()[1], -sample.moduleForcesX()[2], -sample.moduleForcesX()[3]},
                new double[] {-sample.moduleForcesY()[0], -sample.moduleForcesY()[1], -sample.moduleForcesY()[2], -sample.moduleForcesY()[3]}
            );
            case CenterLineMirror: return new SwerveSample(
                sample.t,
                FieldConstants.fieldLength.in(Meters) - sample.x,
                sample.y,
                Math.atan2(+headingSin, -headingCos),
                -sample.vx,
                +sample.vy,
                -sample.omega,
                -sample.ax,
                +sample.ay,
                -sample.alpha,
                new double[] {-sample.moduleForcesX()[0], -sample.moduleForcesX()[1], -sample.moduleForcesX()[2], -sample.moduleForcesX()[3]},
                new double[] {+sample.moduleForcesY()[0], +sample.moduleForcesY()[1], +sample.moduleForcesY()[2], +sample.moduleForcesY()[3]}
            );
            case XenterLineMirror: return new SwerveSample(
                sample.t,
                sample.x,
                FieldConstants.fieldWidth.in(Meters) - sample.y,
                Math.atan2(-headingSin, +headingCos),
                +sample.vx,
                -sample.vy,
                -sample.omega,
                +sample.ax,
                -sample.ay,
                -sample.alpha,
                new double[] {+sample.moduleForcesX()[0], +sample.moduleForcesX()[1], +sample.moduleForcesX()[2], +sample.moduleForcesX()[3]},
                new double[] {-sample.moduleForcesY()[0], -sample.moduleForcesY()[1], -sample.moduleForcesY()[2], -sample.moduleForcesY()[3]}
            );
        }
    }

    public static Trajectory<SwerveSample> apply(Trajectory<SwerveSample> trajectory) {
        return apply(trajectory, defaultFlipType);
    }
    public static Trajectory<SwerveSample> apply(Trajectory<SwerveSample> trajectory, FieldFlipType flipType) {
        if (!shouldFlip()) return trajectory;
        return flip(trajectory, flipType);
    }
    public static Trajectory<SwerveSample> flip(Trajectory<SwerveSample> trajectory) {
        return flip(trajectory, defaultFlipType);
    }
    public static Trajectory<SwerveSample> flip(Trajectory<SwerveSample> trajectory, FieldFlipType flipType) {
        var flippedSamples = new ArrayList<SwerveSample>(trajectory.samples().size());
        for (var sample : trajectory.samples()) {
            flippedSamples.add(flip(sample, flipType));
        }
        return new Trajectory<>(trajectory.name(), flippedSamples, trajectory.splits(), trajectory.events());
    }

    public static boolean shouldFlip() {
        return DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }
}
