package frc.robot.subsystems.objectiveTracker.objectives;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.flipping.AllianceFlipped;

public class ScoreLuniteObjective implements Objective {
    private final AllianceFlipped<Pose2d> targetRobotPose;
    private final AllianceFlipped<Translation3d> aimPoint;

    public ScoreLuniteObjective(AllianceFlipped<Pose2d> targetRobotPose, AllianceFlipped<Translation3d> aimPoint){
        this.targetRobotPose = targetRobotPose;
        this.aimPoint = aimPoint;
    }

    @Override
    public Optional<AllianceFlipped<Pose2d>> getTargetPose() {
        return Optional.of(this.targetRobotPose);
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.ScoreLunite;
    }

    public AllianceFlipped<Translation3d> getAimPoint() {
        return this.aimPoint;
    }
}
