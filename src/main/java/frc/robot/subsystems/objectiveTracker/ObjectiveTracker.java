package frc.robot.subsystems.objectiveTracker;

import java.util.List;
import java.util.Optional;
import java.util.Set;

import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.objectiveTracker.objectives.IntakeLuniteObjective;
import frc.robot.subsystems.objectiveTracker.objectives.Objective;
import frc.robot.subsystems.objectiveTracker.objectives.Objective.ObjectiveType;
import frc.robot.subsystems.objectiveTracker.objectives.PassObjective;
import frc.robot.subsystems.objectiveTracker.objectives.ScoreLuniteObjective;
import frc.util.VirtualSubsystem;

public class ObjectiveTracker extends VirtualSubsystem{
    public static enum Goal{
        OUTER_HIGH,
        OUTER_LOW,
        INNER_HIGH,
        INNER_LOW
    }

    public static enum Mode {
        Smart,
        Dumb
    }

    private boolean outerClosed = false;

    private Goal selectedGoal = Goal.OUTER_HIGH;
    private final Set<ScoreLuniteObjective> allScoreLuniteObjectives;
    private final Set<ScoreLuniteObjective> availableScoreLuniteObjectives;

    private final Set<PassObjective> allPassObjectives;

    private final Set<IntakeLuniteObjective> allIntakeLuniteObjectives;
    
    private Optional<ObjectiveType> typeOverride = Optional.empty();
    private Optional<Objective> currentObjective = Optional.empty();

    public ObjectiveTracker() {
        System.out.println("[Init ObjectiveTracker] Instantiating ObjectiveTracker");

        // Score Lunite
        this.allScoreLuniteObjectives = Set.of(
            new ScoreLuniteObjective(FieldConstants.outerShootingPose, FieldConstants.outerHighGoalAimPoint)
        );

    }
}
