package frc.robot.subsystems.objectiveTracker.objectives;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.flipping.AllianceFlipped;

public class PassObjective implements Objective {
    private final AllianceFlipped<Translation3d> aimPoint;

    public PassObjective(AllianceFlipped<Pose2d> targetRobotPose, AllianceFlipped<Translation3d> aimPoint){
        this.aimPoint = aimPoint;
    }

    @Override
    public ObjectiveType getObjectiveType() {
        return ObjectiveType.PassLunite;
    }

    public AllianceFlipped<Translation3d> getAimPoint() {
        return this.aimPoint;
    }

    @Override
    public Optional<AllianceFlipped<Pose2d>> getTargetPose() {
        return Optional.empty();
    }    
}
