package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.rollers.IntakeRollers;
import frc.robot.subsystems.intake.slam.IntakeSlam;

public class Intake {
    public final IntakeSlam slam;
    public final IntakeRollers rollers;

    public Intake(IntakeSlam slam, IntakeRollers rollers) {
        this.slam = slam;
        this.rollers = rollers;
    }

    public Command idle() {
        return Commands.parallel(
            this.slam.retract(),
            this.rollers.idle()
        );
    }

    public Command intake() {
        return Commands.parallel(
            this.slam.deploy(),
            this.rollers.intake()
        );
    }
}
