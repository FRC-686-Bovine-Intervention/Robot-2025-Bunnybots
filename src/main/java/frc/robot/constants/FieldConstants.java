package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Predicate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import frc.util.flipping.AllianceFlipped;

public final class FieldConstants {
    public static final Distance fieldLength = Inches.of(57*12 + 6 + 7.0/8.0);
    public static final Distance fieldWidth =  Inches.of(26*12 + 5);

    public static final AllianceFlipped<Predicate<Translation2d>> onAllianceSide = new AllianceFlipped<>(
        new Predicate<>() {
            private final double halfline = fieldLength.div(2).in(Meters);
            @Override
            public boolean test(Translation2d t) {
                return t.getX() <= this.halfline;
            }
        },
        new Predicate<>() {
            private final double halfline = fieldLength.div(2).in(Meters);
            @Override
            public boolean test(Translation2d t) {
                return t.getX() >= this.halfline;
            }
        }
    );

    public static final AprilTagFieldLayout apriltagLayout;
    static {
        AprilTagFieldLayout a = null;
        try {
            a = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        } catch(Exception e) {
            e.printStackTrace();
        }
        apriltagLayout = a;
    }
    
    public static final Distance highGoalHeight = Inches.of(20);
    public static final Distance highGoalWidth = Inches.of(30);
    public static final Distance highGoalInset = Inches.of(30);
    public static final Distance lowGoalWidth = Inches.of(40);
    public static final Distance lowGoalDepth = Inches.of(29.5);

    public static final Distance highGoalToGround = Inches.of(60).plus(highGoalHeight.div(2));
    public static final Distance lowGoalToGround = Inches.of(22);

    public static final Distance bigGoalsCenterToCenter = Inches.of(144.875).plus(highGoalWidth);
    public static final Distance bigGoalWidth = Inches.of(41);
    public static final Distance bigGoalHeight = Inches.of(84);
    public static final Distance bigGoalDepth = Inches.of(34);

    public static final Pose3d outerBigGoalBlue = new Pose3d(
        new Translation3d(
            bigGoalDepth.div(2),
            bigGoalWidth.div(2),
            bigGoalHeight.div(2)
        ),
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    );

    public static final Pose3d innerBigGoalBlue = outerBigGoalBlue.transformBy(new Transform3d(
        new Translation3d(
            Inches.zero(),
            bigGoalsCenterToCenter,
            Inches.zero()
        ),
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(0))
    ));

    public static final Pose3d outerBigGoalRed = outerBigGoalBlue.transformBy(new Transform3d(
        new Translation3d(
            fieldLength.minus(bigGoalDepth.div(2)),
            Inches.zero(),
            Inches.zero()
        ),
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180))
    ));

    public static final Pose3d innerBigGoalRed = innerBigGoalBlue.transformBy(new Transform3d(
        new Translation3d(
            fieldLength.minus(bigGoalDepth.div(2)),
            Inches.zero(),
            Inches.zero()
        ),
        new Rotation3d(Degrees.of(0), Degrees.of(0), Degrees.of(180))
    ));

    public static final Transform3d highGoalAimOffset = new Transform3d(
        new Translation3d(
            highGoalInset.minus(bigGoalDepth.div(2)).unaryMinus(),
            Inches.zero(),
            highGoalToGround.minus(bigGoalHeight.div(2))
        ),
        Rotation3d.kZero
    );

    public static final Transform3d lowGoalAimOffset = new Transform3d(
        new Translation3d(
            bigGoalDepth.minus(lowGoalDepth.div(2)),
            Inches.zero(),
            lowGoalToGround
        ),
        Rotation3d.kZero
    );

    public static final AllianceFlipped<Translation3d> innerHighGoalAimPoint = new AllianceFlipped<Translation3d>(
        innerBigGoalBlue.transformBy(highGoalAimOffset).getTranslation(),
        innerBigGoalRed.transformBy(highGoalAimOffset).getTranslation()
    );
    
    public static final AllianceFlipped<Translation3d> innerLowGoalAimPoint = new AllianceFlipped<Translation3d>(
        innerBigGoalBlue.transformBy(lowGoalAimOffset).getTranslation(),
        innerBigGoalRed.transformBy(lowGoalAimOffset).getTranslation()
    );

    public static final AllianceFlipped<Translation3d> outerHighGoalAimPoint = new AllianceFlipped<Translation3d>(
        outerBigGoalBlue.transformBy(highGoalAimOffset).getTranslation(),
        outerBigGoalRed.transformBy(highGoalAimOffset).getTranslation()
    );
    
    public static final AllianceFlipped<Translation3d> outerLowGoalAimPoint = new AllianceFlipped<Translation3d>(
        outerBigGoalBlue.transformBy(lowGoalAimOffset).getTranslation(),
        outerBigGoalRed.transformBy(lowGoalAimOffset).getTranslation()
    );

    public static final AllianceFlipped<Translation3d> passAimPoint = new AllianceFlipped<Translation3d>(
        null,
        null
    );
}