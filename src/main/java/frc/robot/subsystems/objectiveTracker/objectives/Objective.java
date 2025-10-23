package frc.robot.subsystems.objectiveTracker.objectives;

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

    public ObjectiveType getObjectiveType();
}
