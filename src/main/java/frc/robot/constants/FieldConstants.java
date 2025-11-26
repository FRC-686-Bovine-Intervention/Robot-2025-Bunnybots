package frc.robot.constants;

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
import frc.robot.constants.FieldConstants.LunarOutpost.CosmicConverter.Goal.GoalType;
import frc.util.flipping.AllianceFlipUtil;
import frc.util.flipping.AllianceFlippable;
import frc.util.flipping.AllianceFlipped;
import frc.util.flipping.AllianceFlipUtil.FieldFlipType;

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

    public static final class Goals {
        public static final Distance highGoalCenterHeight = Meters.zero();
        public static final Distance lowGoalCenterHeight = Meters.zero();

        public static final AllianceFlipped<Goal> shutoffHighGoal = AllianceFlipped.fromBlue(new Goal(new Translation3d(), GoalType.High));
        public static final AllianceFlipped<Goal> shutoffLowGoal = AllianceFlipped.fromBlue(new Goal(new Translation3d(), GoalType.Low));
        public static final AllianceFlipped<Goal> permanentHighGoal = AllianceFlipped.fromBlue(new Goal(new Translation3d(), GoalType.High));
        public static final AllianceFlipped<Goal> permanentLowGoal = AllianceFlipped.fromBlue(new Goal(new Translation3d(), GoalType.Low));

        public static final AllianceFlipped<Goal> leftHighGoal = new AllianceFlipped<>(permanentHighGoal.getBlue(), shutoffHighGoal.getRed());
        public static final AllianceFlipped<Goal> rightHighGoal = new AllianceFlipped<>(shutoffHighGoal.getBlue(), permanentHighGoal.getRed());
        public static final AllianceFlipped<Goal> leftLowGoal = new AllianceFlipped<>(permanentLowGoal.getBlue(), shutoffLowGoal.getRed());
        public static final AllianceFlipped<Goal> rightLowGoal = new AllianceFlipped<>(shutoffLowGoal.getBlue(), permanentLowGoal.getRed());

        public static enum GoalType {
            High,
            Low,
            ;
            public <T> T select(T high, T low) {
                return switch (this) {
                    case High -> high;
                    case Low -> low;
                };
            }
        }
        public static final class Goal implements AllianceFlippable<Goal> {
            public final Translation3d centerPoint;
            public final GoalType type;

            private Goal(Translation3d centerPoint, GoalType type) {
                this.centerPoint = centerPoint;
                this.type = type;
            }

            @Override
            public Goal flip(FieldFlipType flipType) {
                return new Goal(
                    AllianceFlipUtil.flip(this.centerPoint, flipType),
                    this.type
                );
            }
        }
    }

    public static final class LunarOutpost {
        public static final Translation2d dimensions = new Translation2d(
            Inches.of(112.250),
            Inches.of(216)
        );
        public static final AllianceFlipped<Pose3d> center = new AllianceFlipped<Pose3d>(
            new Pose3d(new Translation3d(
                dimensions.getMeasureX().div(2),
                dimensions.getMeasureY().div(2),
                Inches.zero()
            ), Rotation3d.kZero),
            new Pose3d(new Translation3d(
                fieldLength.minus(dimensions.getMeasureX().div(2)),
                dimensions.getMeasureY().div(2),
                Inches.zero()
            ), new Rotation3d(0, 0, Math.PI))
        );

        public final CosmicConverter innerGoal;
        public final CosmicConverter outerGoal;

        private LunarOutpost(Alliance alliance){
            if (alliance == Alliance.Blue) {
                this.outerGoal = new CosmicConverter(this, 0, alliance);
                this.innerGoal = new CosmicConverter(this, 1, alliance);
            } else {
                this.outerGoal = new CosmicConverter(this, 1, alliance);
                this.innerGoal = new CosmicConverter(this, 0, alliance);
            }
        }
        

        public static final class CosmicConverter {
            public static final Translation3d dimensions = new Translation3d(
                Inches.of(34),
                Inches.of(41),
                Inches.of(84)
            );
            public static final Distance cosmicConvertersCenterToCenter = Inches.of(175.625);

            public final LunarOutpost parent;
            public final int id;
            public final CosmicConverterSide side;
            public final int apriltagID;
            public final Pose3d center;
            public final Goal highGoal;
            public final Goal lowGoal;
            
            private CosmicConverter(LunarOutpost parent, int id, Alliance alliance) {
                this.parent = parent;
                this.id = id;

                this.side = switch (this.id) {
                    default -> alliance == Alliance.Blue ? CosmicConverterSide.OUTER : CosmicConverterSide.INNER;
                    case 1 -> alliance == Alliance.Blue ? CosmicConverterSide.INNER : CosmicConverterSide.OUTER;
                };

                this.apriltagID = switch (this.id) {
                    default -> alliance == Alliance.Blue ? 7 : 6;
                    case 1 -> alliance == Alliance.Blue ? 5 : 8;
                };

                this.center = switch (this.id) {
                    default -> LunarOutpost.center.get(alliance).transformBy(
                        new Transform3d(
                            new Translation3d(
                                LunarOutpost.dimensions.getMeasureX().div(2).unaryMinus().plus(dimensions.getMeasureX().div(2)),
                                LunarOutpost.dimensions.getMeasureY().div(2).unaryMinus().plus(dimensions.getMeasureY().div(2)),
                                dimensions.getMeasureZ().div(2)
                            ),
                            Rotation3d.kZero
                        )
                    );
                    case 1 -> LunarOutpost.center.get(alliance).transformBy(
                        new Transform3d(
                            new Translation3d(    
                                LunarOutpost.dimensions.getMeasureX().div(2).unaryMinus().plus(dimensions.getMeasureX().div(2)),
                                LunarOutpost.dimensions.getMeasureY().div(2).unaryMinus().plus(dimensions.getMeasureY().div(2)).plus(cosmicConvertersCenterToCenter),
                                dimensions.getMeasureZ().div(2)
                            ),
                            Rotation3d.kZero
                        )
                    );
                };

                this.highGoal = new HighGoal(this, GoalType.HIGH);
                this.lowGoal = new LowGoal(this, GoalType.LOW);
            }
            public static abstract class Goal {
                public final CosmicConverter parent;
                public final Translation3d aimPoint;
                public final GoalType type;

                private Goal(CosmicConverter parent, Transform3d aimOffset, GoalType type) {
                    this.parent = parent;
                    this.aimPoint = parent.center.transformBy(aimOffset).getTranslation();
                    this.type = type;
                }

                public static enum GoalType {
                    HIGH,
                    LOW
                }
            }
            public static final class HighGoal extends Goal{
                public static final Translation2d dimensions = new Translation2d(
                    Inches.of(30),
                    Inches.of(20)
                );
                public static final Distance inset = Inches.of(30);
                public static final Distance toGround = Inches.of(60).plus(dimensions.getMeasureY().div(2));
                
                public static final Transform3d aimOffset = new Transform3d(
                    new Translation3d(
                        inset.minus(CosmicConverter.dimensions.getMeasureX().div(2)).unaryMinus(),
                        Inches.zero(),
                        toGround.minus(CosmicConverter.dimensions.getMeasureZ().div(2))
                    ),
                    Rotation3d.kZero
                );
                
                private HighGoal(CosmicConverter parent, GoalType type) {
                    super(
                        parent,
                        aimOffset,
                        type
                    );
                }
            }
            public static final class LowGoal extends Goal {
                public static final Translation2d dimensions = new Translation2d(
                    Inches.of(29.5),
                    Inches.of(40)
                );
                public static final Distance toGround = Inches.of(22);

                public static final Transform3d aimOffset = new Transform3d(
                    new Translation3d(
                        CosmicConverter.dimensions.getMeasureX().minus(dimensions.getMeasureX().div(2)),
                        Inches.zero(),
                        toGround
                    ),
                    Rotation3d.kZero
                );

                private LowGoal(CosmicConverter parent, GoalType type) {
                    super(
                        parent,
                        aimOffset,
                        type
                    );
                }
            }

            public static enum CosmicConverterSide {
                OUTER,
                INNER
            }
        }

        public static final AllianceFlipped<LunarOutpost> lunarOutposts;
        
        static {
            lunarOutposts = AllianceFlipped.fromFunction((alliance) -> new LunarOutpost(alliance));
        }
    }

    public static final class StarspireZone {
        public static final Translation2d dimensions = new Translation2d(
            Inches.of(112.25),
            Inches.of(84)
        );
        public static final AllianceFlipped<Pose2d> center = new AllianceFlipped<Pose2d>(
            new Pose2d(
                new Translation2d(
                    fieldLength.minus(dimensions.getMeasureX().div(2)),
                    fieldWidth.minus(dimensions.getMeasureY().div(2))
                ),
                Rotation2d.k180deg
            ),
            new Pose2d(
                new Translation2d(
                    dimensions.getMeasureX().div(2),
                    fieldWidth.minus(dimensions.getMeasureY().div(2))
                ),
                Rotation2d.kZero
            )
        );
        
        public final Starspire rearStarspire;
        public final Starspire sideStarspire;

        private StarspireZone(Alliance alliance) {
            this.rearStarspire = new Starspire(this, 0, alliance);
            this.sideStarspire = new Starspire(this, 1, alliance);
        }

        public static final class Starspire {
            public static final Translation2d dimensions = new Translation2d(
                Inches.of(4), // Wall protrusion
                Inches.of(24) // Width
            );

            public final StarspireZone parent;
            public final int id;
            public final int apriltagID;
            public final Pose2d frontCenter;
            public final Pose2d robotPose;
            public final StarspireSide side;

            private Starspire(StarspireZone parent, int id, Alliance alliance) {
                this.parent = parent;
                this.id = id;

                this.side = switch (this.id) {
                    default -> alliance == Alliance.Blue ? StarspireSide.REAR : StarspireSide.SIDE;
                    case 1 -> alliance == Alliance.Blue ? StarspireSide.SIDE : StarspireSide.REAR;
                };

                this.apriltagID = switch (this.side) {
                    default -> alliance == Alliance.Blue ? 4 : 3;
                    case SIDE -> alliance == Alliance.Blue ? 2 : 1;
                };

                this.frontCenter = switch (this.side) {
                    default -> alliance == Alliance.Blue ? 
                        StarspireZone.center.get(alliance).transformBy(new Transform2d(
                            new Translation2d(
                                StarspireZone.dimensions.getMeasureX().div(2).unaryMinus().plus(dimensions.getMeasureX()),
                                StarspireZone.dimensions.getMeasureY().div(2).unaryMinus().plus(Inches.of(18)).plus(dimensions.getMeasureY().div(2)).unaryMinus()
                            ),
                            Rotation2d.kZero
                        )) :
                        StarspireZone.center.get(alliance).transformBy(new Transform2d(
                            new Translation2d(
                                StarspireZone.dimensions.getMeasureX().div(2).unaryMinus().plus(dimensions.getMeasureX()),
                                StarspireZone.dimensions.getMeasureY().div(2).unaryMinus().plus(Inches.of(18)).plus(dimensions.getMeasureY().div(2))
                            ),
                            Rotation2d.kZero
                        ))
                    ;
                    case SIDE -> alliance == Alliance.Blue ? 
                        StarspireZone.center.get(alliance).transformBy(new Transform2d(
                            new Translation2d(
                                StarspireZone.dimensions.getMeasureX().div(2).unaryMinus().plus(Inches.of(60)).plus(dimensions.getMeasureY().div(2)),
                                StarspireZone.dimensions.getMeasureY().div(2).unaryMinus().plus(dimensions.getMeasureX())
                            ),
                            Rotation2d.kCCW_90deg
                        )) :
                        StarspireZone.center.get(alliance).transformBy(new Transform2d(
                            new Translation2d(
                                StarspireZone.dimensions.getMeasureX().div(2).unaryMinus().plus(Inches.of(60)).plus(dimensions.getMeasureY().div(2)),
                                StarspireZone.dimensions.getMeasureY().div(2).unaryMinus().plus(dimensions.getMeasureX()).unaryMinus()
                            ),
                            Rotation2d.kCW_90deg
                        ))
                    ;
                };

                this.robotPose = this.frontCenter.transformBy(new Transform2d(
                    new Translation2d(
                        RobotConstants.robotLength.div(2),
                        Inches.zero()
                    ),
                    Rotation2d.kZero
                ));
            }

            public static enum StarspireSide {
                REAR,
                SIDE
            }
        }

        public static final AllianceFlipped<StarspireZone> starspireZones;
        
        static {
            starspireZones = AllianceFlipped.fromFunction((alliance) -> new StarspireZone(alliance));
        }
    }

    public static final AllianceFlipped<Translation3d> passTargetPoints = new AllianceFlipped<>(
        new Translation3d(
            Inches.of(0),
            Inches.of(0),
            Inches.of(0)
        ),
        new Translation3d(
            Inches.of(0),
            Inches.of(0),
            Inches.of(0)
        )
    );
}