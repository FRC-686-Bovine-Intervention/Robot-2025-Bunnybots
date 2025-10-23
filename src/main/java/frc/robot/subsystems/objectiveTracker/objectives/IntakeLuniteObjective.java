package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Pose2d;
import frc.util.flipping.AllianceFlipped;

public class IntakeLuniteObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;

    public IntakeLuniteObjective(AllianceFlipped<Pose2d> targetRobotPose) {
        this.targetRobotPose = targetRobotPose;
    }
    
    public AllianceFlipped<Pose2d> getTargetPose() {
        return this.targetRobotPose;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.IntakeLunite;
    }
}
