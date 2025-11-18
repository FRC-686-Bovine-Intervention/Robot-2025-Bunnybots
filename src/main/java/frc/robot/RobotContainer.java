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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotState.ShootTarget;
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
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.util.LoggedTracer;
import frc.util.controllers.XboxController;

public class RobotContainer {
    private final RobotState robotState;

    // Subsystems
    public final Drive drive;
    public final Shooter shooter;
    public final Pivot pivot;

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
        robotState = RobotState.getInstance();
        switch (RobotType.getMode()) {
            case REAL -> {
                this.drive = new Drive(
                    new OdometryTimestampIOOdometryThread(),
                    new GyroIOPigeon2(),
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOFalcon550::new)
                        .toArray(ModuleIO[]::new)
                );
                this.shooter = new Shooter(new ShooterIOTalonFX());
                this.pivot = new Pivot(new PivotIOTalonFX());
            }
            case SIM -> {
                this.drive = new Drive(
                    new OdometryTimestampIOSim(),
                    new GyroIO() {},
                    Arrays.stream(DriveConstants.moduleConstants)
                        .map(ModuleIOSim::new)
                        .toArray(ModuleIO[]::new)
                );
                this.shooter = new Shooter(new ShooterIOSim());
                this.pivot = new Pivot(new PivotIOSim());
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
                this.shooter = new Shooter(new ShooterIO() {});
                this.pivot = new Pivot(new PivotIO() {});
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
            .smoothRadialDeadband(0.5) // Intentional to make pass selection less error-prone
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

        rightHigh.onTrue(Commands.runOnce(() -> {
            if (robotState.getSelectedShootTarget() != ShootTarget.RIGHT_HIGH){
                robotState.setSelectedShootTarget(ShootTarget.RIGHT_HIGH);
            } else {
                robotState.setSelectedShootTarget(ShootTarget.NONE);
            }
        }));

        leftHigh.onTrue(Commands.runOnce(() -> {
            if (robotState.getSelectedShootTarget() != ShootTarget.LEFT_HIGH){
                robotState.setSelectedShootTarget(ShootTarget.LEFT_HIGH);
            } else {
                robotState.setSelectedShootTarget(ShootTarget.NONE);
            }
        }));

        leftLow.onTrue(Commands.runOnce(() -> {
            if (robotState.getSelectedShootTarget() != ShootTarget.LEFT_LOW){
                robotState.setSelectedShootTarget(ShootTarget.LEFT_LOW);
            } else {
                robotState.setSelectedShootTarget(ShootTarget.NONE);
            }
        }));

        rightLow.onTrue(Commands.runOnce(() -> {
            if (robotState.getSelectedShootTarget() != ShootTarget.RIGHT_LOW){
                robotState.setSelectedShootTarget(ShootTarget.RIGHT_LOW);
            } else {
                robotState.setSelectedShootTarget(ShootTarget.NONE);
            }
        }));

        pass.onTrue(Commands.runOnce(() -> {
            if (robotState.getSelectedShootTarget() != ShootTarget.PASS) {
                robotState.setSelectedShootTarget(ShootTarget.PASS);
            } else {
                robotState.setSelectedShootTarget(ShootTarget.NONE);
            }
        }));

        driveController.povUp().onTrue(Commands.runOnce(() -> {
            AimingParameters.applyPivotOffset(3);
        }));
        driveController.povDown().onTrue(Commands.runOnce(() -> {
            AimingParameters.applyPivotOffset(-3);
        }));
        driveController.povRight().onTrue(Commands.runOnce(() -> {
            AimingParameters.applyShooterOffset(0.5);
        }));
        driveController.povLeft().onTrue(Commands.runOnce(() -> {
            AimingParameters.applyShooterOffset(-0.5);
        }));

        final Command autoAimLock = this.drive.rotationalSubsystem.pidControlledHeading(() -> AimingParameters.shotPose().getRotation()).alongWith(pivot.aim()).alongWith(shooter.aimWithoutAutoShoot());
        this.automationsLoop.bind(() -> {
            LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Aim Lock/Before");
            var selectedShootTarget = this.robotState.getSelectedShootTarget();
            if (selectedShootTarget != ShootTarget.NONE) {
                AimingParameters.setFrom(drive);
                if (!autoAimLock.isScheduled()) {
                    autoAimLock.schedule();
                }
            } else {
                if (autoAimLock.isScheduled()) {
                    autoAimLock.cancel();
                }
            }
            LoggedTracer.logEpoch("CommandScheduler Periodic/Automations/Aim Lock");
        });
    }
}
