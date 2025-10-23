package frc.robot.subsystems.objectiveTracker;

import java.lang.reflect.Field;
import java.util.HashSet;
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
        var innerGoal = FieldConstants.LunarOutpost.lunarOutposts.map((zone) -> zone.innerGoal);
        var outerGoal = FieldConstants.LunarOutpost.lunarOutposts.map((zone) -> zone.outerGoal);
        this.allScoreLuniteObjectives = Set.of(
            new ScoreLuniteObjective(outerGoal.map((goal) -> goal.highGoal.aimPoint)),
            new ScoreLuniteObjective(outerGoal.map((goal) -> goal.lowGoal.aimPoint)),
            new ScoreLuniteObjective(innerGoal.map((goal) -> goal.highGoal.aimPoint)),
            new ScoreLuniteObjective(innerGoal.map((goal) -> goal.lowGoal.aimPoint))
        );
        this.availableScoreLuniteObjectives = new HashSet<>(this.allScoreLuniteObjectives.size());

        // Intake
        var sideZone = FieldConstants.StarspireZone.starspireZones.map((zone) -> zone.sideStarspire);
        var rearZone = FieldConstants.StarspireZone.starspireZones.map((zone) -> zone.rearStarspire);
        this.allIntakeLuniteObjectives = Set.of(
            new IntakeLuniteObjective(sideZone.map((starspire) -> starspire.robotPose)),
            new IntakeLuniteObjective(rearZone.map((starspire) -> starspire.robotPose))
        );

        // Pass
        this.allPassObjectives = Set.of(
            
        );
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'periodic'");
    }
}
