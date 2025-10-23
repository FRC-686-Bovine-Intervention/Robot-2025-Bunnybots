package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Predicate;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    public static final class GoalZone {
        public static final Translation2d dimensions = new Translation2d(
            Inches.of(112.250),
            Inches.of(216)
        );
        public static final AllianceFlipped<Pose3d> sharedFieldCorner = new AllianceFlipped<Pose3d>(
            new Pose3d(new Translation3d(
                Inches.of(0),
                Inches.of(0),
                Inches.of(0)
            ), Rotation3d.kZero),
            new Pose3d(new Translation3d(
                fieldLength.minus(dimensions.getMeasureX()),
                Inches.of(0),
                Inches.of(0)
            ), new Rotation3d(0, 0, Math.PI))
        );

        public final GoalObject innerGoal;
        public final GoalObject outerGoal;
        

        public static final class GoalObject {
            public static final Translation3d dimensions = new Translation3d(
                Inches.of(34),
                Inches.of(41),
                Inches.of(84)
            );
            public static final Distance goalObjectsCenterToCenter = Inches.of(175.625).plus(dimensions.getMeasureY().div(2));

            public final GoalZone parent;
            public final int id;
            public final int apriltagID;
            public final Pose3d center;
            public final HighGoal highGoal;
            public final LowGoal lowGoal;
            public final Pose2d shootingPose;
            
            private GoalObject(GoalZone parent, int id, Alliance alliance) {
                this.parent = parent;
                this.id = id;

                this.apriltagID = switch (this.id) {
                    default -> alliance == Alliance.Red ? 8 : 7;
                    case 1 -> alliance == Alliance.Red ? 6 : 5;
                };

                this.center = switch (this.id) {
                    default -> GoalZone.sharedFieldCorner.get(alliance).transformBy(
                        new Transform3d(
                            new Translation3d(
                                dimensions.getMeasureX().div(2),
                                dimensions.getMeasureY().div(2),
                                dimensions.getMeasureZ().div(2)
                            ),
                            Rotation3d.kZero
                        )
                    );
                    case 1 -> GoalZone.sharedFieldCorner.get(alliance).transformBy(
                        new Transform3d(
                            new Translation3d(    
                                dimensions.getMeasureX().div(2),
                                dimensions.getMeasureY().div(2).plus(goalObjectsCenterToCenter),
                                dimensions.getMeasureZ().div(2)
                            ),
                            Rotation3d.kZero
                        )
                    );
                };

                this.shootingPose = switch (this.id) {
                    default -> GoalZone.sharedFieldCorner.get(alliance).toPose2d().transformBy(new Transform2d(
                            new Translation2d(
                                GoalZone.dimensions.getMeasureX().plus(RobotConstants.robotLength.div(2)),
                                dimensions.getMeasureY().div(2)
                            ),
                            Rotation2d.k180deg
                        )
                    );
                    case 1 -> Pose2d.kZero;
                };
            }
            public static final class HighGoal{
                public static final Translation2d dimensions = new Translation2d(
                    Inches.of(30),
                    Inches.of(20)
                );
                public static final Distance inset = Inches.of(30);
                public static final Distance toGround = Inches.of(60).plus(dimensions.getMeasureY().div(2));
                
                public static final Transform3d aimOffset = new Transform3d(
                    new Translation3d(
                        inset.minus(GoalObject.dimensions.getMeasureX().div(2)).unaryMinus(),
                        Inches.zero(),
                        toGround.minus(GoalObject.dimensions.getMeasureZ().div(2))
                    ),
                    Rotation3d.kZero
                );

                public final GoalObject parent;
                public final Translation3d aimPoint;
                
                private HighGoal(GoalObject parent) {
                    this.parent = parent;
                    
                    this.aimPoint = parent.center.transformBy(aimOffset).getTranslation();
                }
            }
            public static final class LowGoal{
                public static final Translation2d dimensions = new Translation2d(
                    Inches.of(29.5),
                    Inches.of(40)
                );
                public static final Distance toGround = Inches.of(22);

                public static final Transform3d aimOffset = new Transform3d(
                    new Translation3d(
                        GoalObject.dimensions.getMeasureX().minus(dimensions.getMeasureX().div(2)),
                        Inches.zero(),
                        toGround
                    ),
                    Rotation3d.kZero
                );

                public final GoalObject parent;
                public final Translation3d aimPoint;

                private LowGoal(GoalObject parent){
                    this.parent = parent;

                    this.aimPoint = parent.center.transformBy(aimOffset).getTranslation();
                }
            }
        }
    }
    
    public static final Distance highGoalHeight = Inches.of(20);
    public static final Distance highGoalWidth = Inches.of(30);
    public static final Distance highGoalInset = Inches.of(30);
    public static final Distance lowGoalWidth = Inches.of(40);
    public static final Distance lowGoalDepth = Inches.of(29.5);

    public static final Distance highGoalToGround = Inches.of(60).plus(highGoalHeight.div(2));
    public static final Distance lowGoalToGround = Inches.of(22);

    public static final Distance bigGoalsCenterToCenter = Inches.of(144.875).plus(highGoalWidth);

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

    public static final AllianceFlipped<Pose2d> innerShootingPose = new AllianceFlipped<Pose2d>(
        null, null
    );

    public static final AllianceFlipped<Pose2d> outerShootingPose = new AllianceFlipped<Pose2d>(
        null, null
    );
}