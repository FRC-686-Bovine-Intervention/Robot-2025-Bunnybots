package frc.robot.subsystems.objectiveTracker.objectives;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import frc.util.flipping.AllianceFlipped;

public interface Objective {
    public static enum ObjectiveType {
        IntakeLunite(false),
        PassLunite(true),
        ScoreLunite(true),
        bunnyBundle(false)
        ;
        public final boolean isShootObjective;
        ObjectiveType(boolean isShootObjective) {
            this.isShootObjective = isShootObjective;
        }
    }

    public Optional<AllianceFlipped<Pose2d>> getTargetPose();
    public ObjectiveType getObjectiveType();
}
