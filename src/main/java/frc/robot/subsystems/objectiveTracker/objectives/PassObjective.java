package frc.robot.subsystems.objectiveTracker.objectives;

import edu.wpi.first.math.geometry.Translation3d;
import frc.util.flipping.AllianceFlipped;

public class PassObjective implements Objective {
    private final AllianceFlipped<Translation3d> aimPoint;

    public PassObjective(AllianceFlipped<Translation3d> aimPoint){
        this.aimPoint = aimPoint;
    }

    public ObjectiveType getObjectiveType() {
        return ObjectiveType.PassLunite;
    }

    public AllianceFlipped<Translation3d> getAimPoint() {
        return this.aimPoint;
    }    
}
