package frc.robot.auto.routines;

import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine.AutoQuestion;
import frc.robot.constants.FieldConstants.LunarOutpost.CosmicConverter.Goal.GoalType;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.slam.IntakeSlam;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.util.flipping.AllianceFlipped;

public class ScoreLunites {
    private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<AllianceFlipped<Pose2d>>("Starting Position") {
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startOuter = Settings.option("Outer Goal", AutoConstants.startOuterGoal);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startInner = Settings.option("Inner Goal", AutoConstants.startInnerGoal);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startOuterCorner = Settings.option("Outer Corner", AutoConstants.startOuterTaxiLineWallCorner);
        
        @Override
        protected Settings<AllianceFlipped<Pose2d>> generateSettings() {
            return Settings.from(startOuter, startInner, startOuter, startOuterCorner);
        }
    };

    public static final AutoQuestion<Boolean> scoreAfterPreload = new AutoQuestion<Boolean>("Score After Preload") {
        @Override
        protected Settings<Boolean> generateSettings() {
            return Settings.from(
                Settings.option("No", false),
                Settings.option("Yes", true),
                Settings.option("No", false)
            );
        }
    };

    public static final AutoQuestion<GoalType>[] goalHeights = new AutoQuestion[6];

    static {
        for (int i = 0; i < goalHeights.length; i++) {
            goalHeights[i] = new AutoQuestion<GoalType>("Height #" + (i + 1)) {
                @Override
                protected Settings<GoalType> generateSettings() {
                    return Settings.from(
                        Settings.option("High", GoalType.HIGH),
                        Settings.option("High", GoalType.HIGH),
                        Settings.option("Low", GoalType.LOW)
                    );
                }
            };
        }
    }

    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;
    private final Rollers rollers;

    public ScoreLunites(RobotContainer robot) {
        super("Score Lunites", List.of(
            startPosition,
            scoreAfterPreload,
            goalHeights
        ));
        this.drive = robot.drive;
        this.intake = robot.intake;
        this.shooter = robot.shooter;
        this.rollers = robot.rollers;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreLunites.startPosition.getResponse();
        var scoringCount = ScoreLunites.scoreAfterPreload.getResponse() ? 6 : 3;
        var commands = new ArrayList<Command>();
        
        
    }
}
