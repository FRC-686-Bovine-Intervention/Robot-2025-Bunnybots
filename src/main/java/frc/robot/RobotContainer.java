// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Arrays;
import java.util.Set;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.FieldConstants;
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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.shooter.pivot.Pivot;
import frc.robot.subsystems.shooter.pivot.PivotIO;
import frc.robot.subsystems.shooter.pivot.PivotIOSim;
import frc.robot.subsystems.shooter.pivot.PivotIOTalonFX;
import frc.util.controllers.XboxController;

public class RobotContainer {
    // Subsystems
    public final Drive drive;
    public final Shooter shooter;

    // Vision

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
                this.shooter = new Shooter(
                    new Pivot(new PivotIOTalonFX()),
                    new Flywheel(new FlywheelIOTalonFX())
                );
            }
            case SIM -> {
                this.drive = new Drive(
                    new OdometryTimestampIOSim(),
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                this.shooter = new Shooter(
                    new Pivot(new PivotIOSim()),
                    new Flywheel(new FlywheelIOSim())
                );
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
                this.shooter = new Shooter(
                    new Pivot(new PivotIO() {}),
                    new Flywheel(new FlywheelIO() {})
                );
            }
        }

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
        // Set aim to be locked
        var aimJoystick = this.driveController.rightStick
            .roughRadialDeadband(0.5) // Intentional to make pass selection less error-prone
        ; 
        
        var rightHigh = new Trigger(() -> {
            var aimPos = aimJoystick.radsFromPosXCCW();
            if (aimPos > 0 && aimPos <= Math.PI/2){
                return true;
            } else {
                return false;
            }
        });
        var leftHigh = new Trigger(() -> {
            var aimPos = aimJoystick.radsFromPosXCCW();
            if (aimPos > Math.PI/2 && aimPos <= Math.PI){
                return true;
            } else {
                return false;
            }
        });
        var leftLow = new Trigger(() -> {
            var aimPos = aimJoystick.radsFromPosXCCW();
            if (aimPos > Math.PI && aimPos <= 3*Math.PI/2){
                return true;
            } else {
                return false;
            }
        });
        var rightLow = new Trigger(() -> {
            var aimPos = aimJoystick.radsFromPosXCCW();
            if (aimPos > 3*Math.PI/2 || aimPos <= 0){
                return true;
            } else {
                return false;
            }
        });
        var pass = driveController.rightStickButton();
        
        leftHigh.toggleOnTrue(Commands.parallel(
            this.shooter.aim(
                RobotState.getInstance()::getEstimatedGlobalPose,
                this.drive::getFieldMeasuredSpeeds,
                () -> FieldConstants.Goals.leftHighGoal.getOurs()
            )
            .repeatedly(),
            this.shooter.aimPivot(),
            this.shooter.aimFlywheel()
        ));

        rightHigh.toggleOnTrue(Commands.parallel(
            this.shooter.aim(
                RobotState.getInstance()::getEstimatedGlobalPose,
                this.drive::getFieldMeasuredSpeeds,
                () -> FieldConstants.Goals.rightHighGoal.getOurs()
            )
            .repeatedly(),
            this.shooter.aimPivot(),
            this.shooter.aimFlywheel()
        ));

        leftLow.toggleOnTrue(Commands.parallel(
            this.shooter.aim(
                RobotState.getInstance()::getEstimatedGlobalPose,
                this.drive::getFieldMeasuredSpeeds,
                () -> FieldConstants.Goals.leftLowGoal.getOurs()
            )
            .repeatedly(),
            this.shooter.aimPivot(),
            this.shooter.aimFlywheel()
        ));

        rightLow.toggleOnTrue(Commands.parallel(
            this.shooter.aim(
                RobotState.getInstance()::getEstimatedGlobalPose,
                this.drive::getFieldMeasuredSpeeds,
                () -> FieldConstants.Goals.rightLowGoal.getOurs()
            )
            .repeatedly(),
            this.shooter.aimPivot(),
            this.shooter.aimFlywheel()
        ));

        // pass.onTrue(Commands.runOnce(() -> {
        //     if (RobotState.getInstance().getSelectedShootTarget() != ShootTarget.PASS) {
        //         RobotState.getInstance().setSelectedShootTarget(ShootTarget.PASS);
        //     } else {
        //         RobotState.getInstance().setSelectedShootTarget(ShootTarget.NONE);
        //     }
        // }));

        // driveController.povUp().onTrue(Commands.runOnce(() -> {
        //     AimingParameters.applyPivotOffset(3);
        // }));
        // driveController.povDown().onTrue(Commands.runOnce(() -> {
        //     AimingParameters.applyPivotOffset(-3);
        // }));
        // driveController.povRight().onTrue(Commands.runOnce(() -> {
        //     AimingParameters.applyShooterOffset(0.5);
        // }));
        // driveController.povLeft().onTrue(Commands.runOnce(() -> {
        //     AimingParameters.applyShooterOffset(-0.5);
        // }));

        // final Command autoAimLock = this.drive.rotationalSubsystem.pidControlledHeading(() -> AimingParameters.shotPose().getRotation()).alongWith(pivot.aim()).alongWith(shooter.aimWithoutAutoShoot());
        // this.automationsLoop.bind(() -> {
        //     LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Aim Lock/Before");
        //     var selectedShootTarget = this.RobotState.getInstance().getSelectedShootTarget();
        //     if (selectedShootTarget != ShootTarget.NONE) {
        //         AimingParameters.setFrom(drive);
        //         if (!autoAimLock.isScheduled()) {
        //             autoAimLock.schedule();
        //         }
        //     } else {
        //         if (autoAimLock.isScheduled()) {
        //             autoAimLock.cancel();
        //         }
        //     }
        //     LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Aim Lock");
        // });
    }
}
