package frc.robot.subsystems.objectiveTracker.objectives;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.util.flipping.AllianceFlipped;

public class IntakeLuniteObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;

    public IntakeLuniteObjective(AllianceFlipped<Pose2d> targetRobotPose) {
        this.targetRobotPose = targetRobotPose;
    }
    
    @Override
    public Optional<AllianceFlipped<Pose2d>> getTargetPose() {
        return Optional.of(this.targetRobotPose);
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.IntakeLunite;
    }
}
