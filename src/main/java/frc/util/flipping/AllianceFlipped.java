package frc.util.flipping;

import java.util.function.BiFunction;
import java.util.function.Function;

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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;

public class AllianceFlipped<T> {
    private final T blue;
    private final T red;

    public AllianceFlipped(T blue, T red) {
        this.blue = blue;
        this.red = red;
    }

    public T getBlue() {
        return blue;
    }
    public T getRed() {
        return red;
    }
    public T get(Alliance alliance) {
        return switch (alliance) {
            case Blue -> getBlue();
            case Red -> getRed();
        };
    }

    public T getOurs() {
        if (AllianceFlipUtil.shouldFlip()) {
            return getRed();
        } else {
            return getBlue();
        }
    }
    public T getTheirs() {
        if (!AllianceFlipUtil.shouldFlip()) {
            return getRed();
        } else {
            return getBlue();
        }
    }

    public <U> AllianceFlipped<U> map(BiFunction<T, Alliance, U> mappingFunction) {
        return new AllianceFlipped<U>(mappingFunction.apply(this.blue, Alliance.Blue), mappingFunction.apply(this.red, Alliance.Red));
    }
    public AllianceFlipped<T> invert() {
        return new AllianceFlipped<>(this.red, this.blue);
    }

    public static <T> AllianceFlipped<T> fromFunction(Function<Alliance, T> generator) {
        return new AllianceFlipped<T>(generator.apply(Alliance.Blue), generator.apply(Alliance.Red));
    }

    public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromBlue(T blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromBlue(T blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromRed(T red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static <T extends AllianceFlippable<T>> AllianceFlipped<T> fromRed(T red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Translation2d> fromBlue(Translation2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Translation2d> fromBlue(Translation2d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Translation2d> fromRed(Translation2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Translation2d> fromRed(Translation2d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Rotation2d> fromBlue(Rotation2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Rotation2d> fromBlue(Rotation2d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Rotation2d> fromRed(Rotation2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Rotation2d> fromRed(Rotation2d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Pose2d> fromBlue(Pose2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Pose2d> fromBlue(Pose2d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Pose2d> fromRed(Pose2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Pose2d> fromRed(Pose2d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Transform2d> fromBlue(Transform2d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Transform2d> fromBlue(Transform2d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Transform2d> fromRed(Transform2d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Transform2d> fromRed(Transform2d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Translation3d> fromBlue(Translation3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Translation3d> fromBlue(Translation3d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Translation3d> fromRed(Translation3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Translation3d> fromRed(Translation3d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Rotation3d> fromBlue(Rotation3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Rotation3d> fromBlue(Rotation3d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Rotation3d> fromRed(Rotation3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Rotation3d> fromRed(Rotation3d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Pose3d> fromBlue(Pose3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Pose3d> fromBlue(Pose3d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Pose3d> fromRed(Pose3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Pose3d> fromRed(Pose3d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Transform3d> fromBlue(Transform3d blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Transform3d> fromBlue(Transform3d blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Transform3d> fromRed(Transform3d red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Transform3d> fromRed(Transform3d red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<ChassisSpeeds> fromBlue(ChassisSpeeds blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<ChassisSpeeds> fromBlue(ChassisSpeeds blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flipFieldRelative(blue, flipType));
    }
    public static AllianceFlipped<ChassisSpeeds> fromRed(ChassisSpeeds red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<ChassisSpeeds> fromRed(ChassisSpeeds red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flipFieldRelative(red, flipType), red);
    }

    public static AllianceFlipped<SwerveSample> fromBlue(SwerveSample blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<SwerveSample> fromBlue(SwerveSample blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<SwerveSample> fromRed(SwerveSample red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<SwerveSample> fromRed(SwerveSample red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }

    public static AllianceFlipped<Trajectory<SwerveSample>> fromBlue(Trajectory<SwerveSample> blue) {
        return fromBlue(blue, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Trajectory<SwerveSample>> fromBlue(Trajectory<SwerveSample> blue, FieldFlipType flipType) {
        return new AllianceFlipped<>(blue, AllianceFlipUtil.flip(blue, flipType));
    }
    public static AllianceFlipped<Trajectory<SwerveSample>> fromRed(Trajectory<SwerveSample> red) {
        return fromRed(red, AllianceFlipUtil.defaultFlipType);
    }
    public static AllianceFlipped<Trajectory<SwerveSample>> fromRed(Trajectory<SwerveSample> red, FieldFlipType flipType) {
        return new AllianceFlipped<>(AllianceFlipUtil.flip(red, flipType), red);
    }
}
