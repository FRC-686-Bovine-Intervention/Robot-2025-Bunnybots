package frc.robot.auto;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.util.flipping.AllianceFlipped;

public final class AutoConstants {
    public static final Time allottedAutoTime = Seconds.of(15.3);
    public static final Time disabledTime = Seconds.of(3);

    public static final Distance blueTaxiLineX = Inches.of(88.25);
    
    public static final AllianceFlipped<Pose2d> startShutoffGoal = AllianceFlipped.fromBlue(
        new Pose2d(
            new Translation2d(
                blueTaxiLineX.minus(RobotConstants.centerToFrontBumper),
                FieldConstants.LunarOutpost.lunarOutposts.getBlue().outerGoal.center.getMeasureY()
            ),
            Rotation2d.k180deg
        )
    );
    public static final AllianceFlipped<Pose2d> startOuterTaxiLineWallCorner = AllianceFlipped.fromBlue(
        new Pose2d(
            new Translation2d(
                blueTaxiLineX.minus(RobotConstants.centerToFrontBumper),
                RobotConstants.centerToSideBumper
            ),
            Rotation2d.k180deg
        )
    );
    public static final AllianceFlipped<Pose2d> startPermanentGoal = AllianceFlipped.fromBlue(
        new Pose2d(
            new Translation2d(
                blueTaxiLineX.minus(RobotConstants.centerToFrontBumper),
                FieldConstants.LunarOutpost.lunarOutposts.getBlue().innerGoal.center.getMeasureY()
            ),
            Rotation2d.k180deg
        )
    );
    public static final AllianceFlipped<Pose2d> startShutoffGoalSide = AllianceFlipped.fromBlue(
        new Pose2d(
            new Translation2d(
                FieldConstants.LunarOutpost.CosmicConverter.dimensions.getMeasureX().minus(RobotConstants.centerToSideBumper),
                FieldConstants.LunarOutpost.CosmicConverter.dimensions.getMeasureY().plus(RobotConstants.centerToSideBumper)
            ),
            Rotation2d.kCW_90deg
        )
    );
    
}