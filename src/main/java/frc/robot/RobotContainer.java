// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.Arrays;
import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOFalcon550;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.OdometryTimestampIO;
import frc.robot.subsystems.drive.OdometryTimestampIO.OdometryTimestampIOOdometryThread;
import frc.robot.subsystems.drive.OdometryTimestampIO.OdometryTimestampIOSim;
import frc.robot.subsystems.drive.commands.WheelRadiusCalibration;
import frc.robot.subsystems.intakePivot.IntakePivot;
import frc.robot.subsystems.intakePivot.IntakePivotIO;
import frc.robot.subsystems.intakePivot.IntakePivotIOSim;
import frc.robot.subsystems.intakePivot.IntakePivotIOTalonFX;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.cameras.Camera;
import frc.robot.subsystems.vision.cameras.CameraIO;
import frc.robot.subsystems.vision.cameras.CameraIOPhoton;
import frc.robot.subsystems.vision.object.ObjectPipeline;
import frc.robot.subsystems.vision.object.ObjectVision;
import frc.util.Perspective;
import frc.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final IntakePivot intakePivot;
    
    // Vision
    public final Camera intakeCamera;
    public final ObjectVision objectVision;

    // Event Loops
    public final EventLoop automationsLoop = new EventLoop();

    // Controllers
    private final XboxController driveController = new XboxController(0);
    @SuppressWarnings("unused")
    private final CommandJoystick simJoystick = new CommandJoystick(5);

    @SuppressWarnings("resource")
    public RobotContainer() {
        System.out.println("[Init RobotContainer] Creating " + RobotType.getMode().name() + " " + RobotType.getRobot().name());

        switch (RobotType.getMode()) {
            case REAL -> {
                this.drive = new Drive(
                    new OdometryTimestampIOOdometryThread(),
                    new GyroIOPigeon2(),
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOFalcon550::new)
                        .toArray(ModuleIO[]::new)
                );
                this.intakeCamera = new Camera(
                    new CameraIOPhoton("Intake"),
                    "Intake",
                    VisionConstants.intakeMount,
                    (isConnected) -> {
                        System.out.println("");
                    }
                );
                this.intakePivot = new IntakePivot(new IntakePivotIOSim());
            }
            case SIM -> {
                this.drive = new Drive(
                    new OdometryTimestampIOSim(),
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                this.intakeCamera = new Camera(
                    new CameraIO() {},
                    "Intake",
                    VisionConstants.intakeMount,
                    (isConnected) -> {}
                );
                this.intakePivot = new IntakePivot(new IntakePivotIOSim());
            }
            default -> {
                this.drive = new Drive(
                    new OdometryTimestampIO() {},
                    new GyroIO() {},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){},
                    new ModuleIO(){}
                );
                this.intakeCamera = new Camera(
                    new CameraIO() {},
                    "Intake",
                    VisionConstants.intakeMount,
                    (isConnected) -> {}
                );
                this.intakePivot = new IntakePivot(new IntakePivotIOSim());
            }
        }

        this.objectVision = new ObjectVision(
            new ObjectPipeline(this.intakeCamera, 0)
        );

        this.drive.structureRoot
            .addChild(intakeCamera.mount);

        System.out.println("[Init RobotContainer] Configuring Commands");
        this.configureCommands();

        System.out.println("[Init RobotContainer] Configuring Notifications");

        System.out.println("[Init RobotContainer] Configuring Autonomous Modes");

        System.out.println("[Init RobotContainer] Configuring System Check");
        SmartDashboard.putData("System Check/Drive/Spin", 
            new Command() {
                private final Drive.Rotational rotationalSubsystem = drive.rotationalSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(this.rotationalSubsystem);
                    setName("TEST Spin");
                }
                public void initialize() {
                    this.timer.restart();
                }
                public void execute() {
                    this.rotationalSubsystem.driveVelocity(Math.sin(this.timer.get()) * 3);
                }
                public void end(boolean interrupted) {
                    this.timer.stop();
                    this.rotationalSubsystem.stop();
                }
            }
        );
        SmartDashboard.putData("System Check/Drive/Circle", 
            new Command() {
                private final Drive.Translational translationSubsystem = drive.translationSubsystem;
                private final Timer timer = new Timer();
                {
                    addRequirements(this.translationSubsystem);
                    setName("TEST Circle");
                }
                public void initialize() {
                    this.timer.restart();
                }
                public void execute() {
                    this.translationSubsystem.driveVelocity(
                        new ChassisSpeeds(
                            Math.cos(this.timer.get()) * 0.01,
                            Math.sin(this.timer.get()) * 0.01,
                            0
                        )
                    );
                }
                public void end(boolean interrupted) {
                    this.timer.stop();
                    this.translationSubsystem.stop();
                }
            }
        );
        
        SmartDashboard.putData("Wheel Calibration", Commands.defer(
            () -> 
                new WheelRadiusCalibration(
                    drive,
                    WheelRadiusCalibration.VOLTAGE_RAMP_RATE.get(),
                    WheelRadiusCalibration.MAX_VOLTAGE.get()
                )
                .withName("Wheel Calibration"),
                Set.of(drive.translationSubsystem, drive.rotationalSubsystem)
            )
        );

        if (RobotConstants.tuningMode) {
            new Alert("Tuning mode active", AlertType.kInfo).set(true);
        }
    }

    private void configureCommands() {
        var driveJoystick = this.driveController.leftStick
            .smoothRadialDeadband(0.1)
            .radialSensitivity(0.75)
            // .radialSlewRateLimit(DriveConstants.joystickSlewRateLimit)
        ;

        this.drive.translationSubsystem.setDefaultCommand(
            new Command() {
                {
                    this.addRequirements(drive.translationSubsystem);
                    this.setName("Driver Control Field Relative");
                }

                @Override
                public void execute() {
                    // Rotate joystick such that up is positive X and left is positive Y
                    var perspectiveX = +driveJoystick.y().getAsDouble();
                    var perspectiveY = -driveJoystick.x().getAsDouble();

                    // Get field relative from perspective relative
                    var perspectiveForwardDirection = Perspective.getCurrent().getForwardDirection();
                    var fieldX = perspectiveX * +perspectiveForwardDirection.getCos() + perspectiveY * -perspectiveForwardDirection.getSin();
                    var fieldY = perspectiveX * +perspectiveForwardDirection.getSin() + perspectiveY * +perspectiveForwardDirection.getCos();

                    var fieldXMetersPerSecond = fieldX * DriveConstants.maxDriveSpeed.in(MetersPerSecond) * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble();
                    var fieldYMetersPerSecond = fieldY * DriveConstants.maxDriveSpeed.in(MetersPerSecond) * DriveConstants.maxDriveSpeedEnvCoef.getAsDouble();

                    // Get robot relative from robot relative
                    var robotRotation = RobotState.getInstance().getEstimatedGlobalPose().getRotation();
                    var robotXMetersPerSecond = fieldXMetersPerSecond * +robotRotation.getCos() + fieldYMetersPerSecond * +robotRotation.getSin();
                    var robotYMetersPerSecond = fieldXMetersPerSecond * -robotRotation.getSin() + fieldYMetersPerSecond * +robotRotation.getCos();

                    //
                    double intakeY;
                    if (intakePivot.hasDeployGoal() && objectVision.hasTarget()) {
                        intakeY = objectVision.getIntakeOffsetSpeedFromRobotSpeeds(new ChassisSpeeds(MetersPerSecond.of(robotXMetersPerSecond), MetersPerSecond.of(robotYMetersPerSecond), RadiansPerSecond.of(0)));
                    } else {
                        intakeY = 0.0;
                    }

                    drive.translationSubsystem.driveVelocity(
                        robotXMetersPerSecond,
                        robotYMetersPerSecond + intakeY
                    );
                }

                @Override
                public void end(boolean interrupted) {
                    drive.translationSubsystem.stop();
                }
            }
        );
        this.drive.rotationalSubsystem.setDefaultCommand(
            this.drive.rotationalSubsystem.spin(this.driveController.rightStick.x().smoothDeadband(0.1).multiply(DriveConstants.maxTurnRate.in(RadiansPerSecond)).multiply(0.5))
                .withName("Robot spin")
        );
        new Trigger(DriverStation::isDisabled).and(() -> driveJoystick.magnitude() > 0).whileTrue(drive.coast());
        //driveController.x().onChange();

        //driveController.rightBumper().and(objectVision::hasTarget).whileTrue(objectVision.autoIntake(objectVision.applyDotProduct(() -> ChassisSpeeds.discretize(0, 0, 0, 0)), () -> true, drive));
    }
}
