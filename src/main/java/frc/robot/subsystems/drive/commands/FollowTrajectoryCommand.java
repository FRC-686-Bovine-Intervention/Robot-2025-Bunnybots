package frc.robot.subsystems.drive.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class FollowTrajectoryCommand extends Command {
    private final Drive drive;
    private final Trajectory<SwerveSample> trajectory;
    private final Timer trajectoryTimer = new Timer();
    private final boolean endWhenFinished;

    private final PIDController translationalPID = new PIDController(0, 0, 0);
    private final PIDController rotationalPID = new PIDController(0, 0, 0);

    private static final LoggedTunable<Distance> MAX_ERROR = LoggedTunable.from("Drive/Trajectory Following/Max Error", Inches::of, 24.0);
    
    public FollowTrajectoryCommand(Drive drive, Trajectory<SwerveSample> trajectory, boolean endWhenFinished) {
        this.drive = drive;
        this.trajectory = trajectory;
        this.endWhenFinished = endWhenFinished;
    }

    @Override
    public void initialize() {
        this.trajectoryTimer.reset();
    }

    @Override
    public void execute() {
        var robotPose = RobotState.getInstance().getEstimatedGlobalPose();
        var sample = this.trajectory.sampleAt(this.trajectoryTimer.get(), false).get();
        var errX = sample.x - robotPose.getX();
        var errY = sample.y - robotPose.getY();
        var errTheta = MathUtil.angleModulus(sample.heading - robotPose.getRotation().getRadians());
        var distanceFromSetpointMeters = Math.hypot(errX, errY);

        var errXNorm = errX / distanceFromSetpointMeters;
        var errYNorm = errY / distanceFromSetpointMeters;

        if (distanceFromSetpointMeters > MAX_ERROR.get().in(Meters)) {
            this.trajectoryTimer.stop();
        } else {
            this.trajectoryTimer.start();
        }

        var transPidOut = this.translationalPID.calculate(0.0, distanceFromSetpointMeters);
        var rotPidOut = this.rotationalPID.calculate(0.0, errTheta);

        var pidX = transPidOut * errXNorm;
        var pidY = transPidOut * errYNorm;
        var pidOmega = rotPidOut;

        double ffX;
        double ffY;
        double ffOmega;
        if (this.trajectoryTimer.isRunning()) {
            ffX = sample.vx;
            ffY = sample.vy;
            ffOmega = sample.omega;
        } else {
            ffX = 0.0;
            ffY = 0.0;
            ffOmega = 0.0;
        }

        var fieldX = ffX + pidX;
        var fieldY = ffY + pidY;
        var fieldOmega = ffOmega + pidOmega;

        var robotX = fieldX * +robotPose.getRotation().getCos() - fieldY * -robotPose.getRotation().getSin();
        var robotY = fieldX * -robotPose.getRotation().getSin() + fieldY * +robotPose.getRotation().getCos();
        var robotOmega = fieldOmega;

        this.drive.runRobotSpeeds(robotX, robotY, robotOmega);
    }

    @Override
    public void end(boolean interrupted) {
        this.trajectoryTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return this.endWhenFinished && this.trajectoryTimer.get() > this.trajectory.getTotalTime();
    }
}
