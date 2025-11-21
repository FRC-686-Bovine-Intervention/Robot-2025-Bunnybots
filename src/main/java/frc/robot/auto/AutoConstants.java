package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
    public static final Time allottedAutoTime = Seconds.of(15.3);
    public static final Time disabledTime = Seconds.of(3);

    public static final AllianceFlipped<Distance> taxiLineX = new AllianceFlipped<Distance>(
        Inches.of(88.25),
        Inches.of(559.75)
    );
    public static final AllianceFlipped<Pose2d> startOuterGoal = new AllianceFlipped<Pose2d>(
        new Pose2d(
            taxiLineX.getBlue().minus(RobotConstants.centerToFrontBumper),
            FieldConstants.LunarOutpost.lunarOutposts.getBlue().outerGoal.center.getMeasureY(),
            Rotation2d.k180deg
        ),
        new Pose2d(
            taxiLineX.getRed().plus(RobotConstants.centerToFrontBumper),
            FieldConstants.LunarOutpost.lunarOutposts.getRed().outerGoal.center.getMeasureY(),
            Rotation2d.kZero
        )
    );
    public static final AllianceFlipped<Pose2d> startOuterTaxiLineWallCorner = new AllianceFlipped<Pose2d>(
        new Pose2d(
            taxiLineX.getBlue().minus(RobotConstants.centerToFrontBumper),
            RobotConstants.centerToSideBumper,
            Rotation2d.k180deg
        ),
        new Pose2d(
            taxiLineX.getRed().plus(RobotConstants.centerToFrontBumper),
            RobotConstants.centerToSideBumper,
            Rotation2d.kZero
        )
    );
    public static final AllianceFlipped<Pose2d> startInnerGoal = new AllianceFlipped<Pose2d>(
        new Pose2d(
            taxiLineX.getBlue().minus(RobotConstants.centerToFrontBumper),
            FieldConstants.LunarOutpost.lunarOutposts.getBlue().innerGoal.center.getMeasureY(),
            Rotation2d.k180deg
        ),
        new Pose2d(
            taxiLineX.getRed().plus(RobotConstants.centerToFrontBumper),
            FieldConstants.LunarOutpost.lunarOutposts.getRed().innerGoal.center.getMeasureY(),
            Rotation2d.kZero
        )
    );
}