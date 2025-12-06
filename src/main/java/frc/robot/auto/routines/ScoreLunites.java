package frc.robot.auto.routines;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.RobotState;
import frc.robot.auto.AutoCommons;
import frc.robot.auto.AutoConstants;
import frc.robot.auto.AutoRoutine;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Goals.Goal;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.shooter.Shooter;
import frc.util.flipping.AllianceFlipped;

public class ScoreLunites extends AutoRoutine {
    private static final AutoQuestion<AllianceFlipped<Pose2d>> startPosition = new AutoQuestion<AllianceFlipped<Pose2d>>("Starting Position") {
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startShutoff = Settings.option("Shutoff Goal", AutoConstants.startShutoffGoal);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startPermanent = Settings.option("Permanent Goal", AutoConstants.startPermanentGoal);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startOuterCorner = Settings.option("Outer Corner", AutoConstants.startOuterTaxiLineWallCorner);
        private static final Map.Entry<String, AllianceFlipped<Pose2d>> startShutoffSide = Settings.option("Shutoff Side", AutoConstants.startShutoffGoalSide);
        
        @Override
        protected Settings<AllianceFlipped<Pose2d>> generateSettings() {
            return Settings.from(startOuterCorner, startShutoff, startPermanent, startOuterCorner, startShutoffSide);
        }
    };

    private static final AutoQuestion<AllianceFlipped<Goal>> targetGoal = new AutoQuestion<AllianceFlipped<Goal>>("Target Goal") {
        private static final Map.Entry<String, AllianceFlipped<Goal>> shutoffHigh = Settings.option("Shutoff High Goal", FieldConstants.Goals.shutoffHighGoal);
        private static final Map.Entry<String, AllianceFlipped<Goal>> shutoffLow = Settings.option("Shutoff Low Goal", FieldConstants.Goals.shutoffLowGoal);
        private static final Map.Entry<String, AllianceFlipped<Goal>> permanentHigh = Settings.option("Permanent High Goal", FieldConstants.Goals.permanentHighGoal);
        private static final Map.Entry<String, AllianceFlipped<Goal>> permanentLow = Settings.option("Permanent Low Goal", FieldConstants.Goals.permanentLowGoal);
        
        @Override
        protected Settings<AllianceFlipped<Goal>> generateSettings() {
            return Settings.from(shutoffHigh, shutoffHigh, shutoffLow, permanentHigh, permanentLow);
        }
    };

    public static final AutoQuestion<Boolean> taxi = new AutoQuestion<Boolean>("Taxi") {
        private static final Map.Entry<String, Boolean> no = Settings.option("No", false);
        private static final Map.Entry<String, Boolean> yes = Settings.option("Yes", true);
        @Override
        protected Settings<Boolean> generateSettings() {
            return Settings.from(yes, yes, no);
        }
    };

    // static {
    //     for (int i = 0; i < goalHeights.length; i++) {
    //         goalHeights[i] = new AutoQuestion<GoalType>("Height #" + (i + 1)) {
    //             @Override
    //             protected Settings<GoalType> generateSettings() {
    //                 return Settings.from(
    //                     Settings.option("High", GoalType.HIGH),
    //                     Settings.option("High", GoalType.HIGH),
    //                     Settings.option("Low", GoalType.LOW)
    //                 );
    //             }
    //         };
    //     }
    // }

    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;
    private final Rollers rollers;

    public ScoreLunites(RobotContainer robot) {
        super("Score Lunites", List.of(
            startPosition,
            targetGoal,
            taxi
        ));
        this.drive = robot.drive;
        this.intake = robot.intake;
        this.shooter = robot.shooter;
        this.rollers = robot.rollers;
    }

    @Override
    public Command generateCommand() {
        var startPosition = ScoreLunites.startPosition.getResponse();
        var targetGoal = ScoreLunites.targetGoal.getResponse();
        var taxi = ScoreLunites.taxi.getResponse();
        var commands = new ArrayList<Command>();
        
        var aimCommand = Commands.parallel(
            this.shooter.aim(
                () -> RobotState.getInstance().getRobotPoseFromTag(targetGoal.getOurs().apriltagID).orElse(RobotState.getInstance().getEstimatedGlobalPose()),
                this.drive::getFieldMeasuredSpeeds,
                () -> targetGoal.getOurs()
            )
            .repeatedly(),
            this.shooter.aimPivot(),
            this.shooter.aimFlywheel(),
            this.shooter.aimAzimuth(this.drive.rotationalSubsystem)
        ).withName("Aim Target Goal").asProxy();

        var kickCommand = this.rollers.kick().asProxy();

        commands.add(
            Commands.deadline(
                Commands.sequence(
                    Commands.waitSeconds(1.0),
                    kickCommand.withTimeout(5.0)
                ),
                aimCommand
            )
        );

        if (taxi) {
            var taxiSpeeds = AllianceFlipped.fromBlue(new ChassisSpeeds(1,0,0));
    
            var taxiCommand = new Command() {
                {
                    this.setName("Taxi");
                    this.addRequirements(drive.translationSubsystem, drive.rotationalSubsystem);
                }
                @Override
                public void execute() {
                    drive.runFieldSpeeds(taxiSpeeds.getOurs());
                }
    
                @Override
                public void end(boolean interrupted) {
                    drive.stop();
                }
            }.asProxy().withTimeout(5);
    
            commands.add(taxiCommand);
        }


        return Commands.parallel(
            AutoCommons.setOdometryFlipped(startPosition, this.drive),
            Commands.sequence(commands.toArray(Command[]::new))
        );
    }
}
