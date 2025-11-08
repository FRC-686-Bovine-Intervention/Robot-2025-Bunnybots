package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.drive.DriveConstants.ModuleConstants;
import frc.util.FFConstants;
import frc.util.LoggedTracer;
import frc.util.NeutralMode;
import frc.util.PIDConstants;
import frc.util.loggerUtil.tunables.LoggedTunable;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    public final ModuleConstants config;

    private double wheelAngularPositionRads = 0.0;
    private double wheelAngularVelocityRadsPerSec = 0.0;
    private final SwerveModulePosition modulePosition = new SwerveModulePosition();
    private final SwerveModuleState moduleState = new SwerveModuleState();

    private final SwerveModulePosition[] modulePositionSampleBuffer = new SwerveModulePosition[OdometryThread.MAX_BUFFER_SIZE];
    private SwerveModulePosition[] modulePositionSamples = new SwerveModulePosition[0];

    private static final LoggedTunable<LinearVelocity> brakeModeThreshold = LoggedTunable.from("Drive/Brake Mode Threshold", InchesPerSecond::of, 1); 
    
    private static final LoggedTunable<PIDConstants> drivePIDConsts = LoggedTunable.from(
        "Drive/Module/Drive/PID",
        new PIDConstants(
            0.1,
            0,
            0
        )
    );
    private static final LoggedTunable<FFConstants> driveFFConsts = LoggedTunable.from(
        "Drive/Module/Drive/FF",
        new FFConstants(
            0,
            0,
            2.2,
            0
        )
    );
    private static final LoggedTunable<PIDConstants> azimuthPIDConsts = LoggedTunable.from(
        "Drive/Module/Azimuth/PID",
        new PIDConstants(
            5*2*Math.PI,
            0*2*Math.PI,
            0*2*Math.PI
        )
    );

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0,0,0);

    // private final DeviceFaultAlerts driveMotorActiveFaultsAlert;
    // private final DeviceFaultAlerts driveMotorStickyFaultsAlert;
    // private final DeviceFaultAlerts azimuthMotorActiveFaultsAlert;
    // private final DeviceFaultAlerts azimuthMotorStickyFaultsAlert;
    // private final DeviceFaultClearer driveMotorStickyFaultClearer;
    // private final DeviceFaultClearer azimuthMotorStickyFaultClearer;

    private final Alert driveMotorDisconnectedAlert;
    private final Alert azimuthMotorDisconnectedAlert;
    private final Alert azimuthEncoderDisconnectedAlert;
    private final Alert driveMotorDisconnectedGlobalAlert;
    private final Alert azimuthMotorDisconnectedGlobalAlert;
    private final Alert azimuthEncoderDisconnectedGlobalAlert;

    private static final LoggedTunable<Angle> azimuthStdDevThreshold = LoggedTunable.from("Drive/Module/Azimuth StdDev Threshold", Degrees::of, 5);
    private double azimuthPositionRads = 0.0;

    private double azimuthMotorOffsetRads = 0.0;
    private double azimuthEncoderOffsetRads = 0.0;

    public Module(ModuleIO io, ModuleConstants config) {
        this.io = io;
        this.config = config;

        driveFFConsts.get().update(this.driveFeedforward);
        this.io.configDrivePID(drivePIDConsts.get());
        this.io.configAzimuthPID(azimuthPIDConsts.get());

        for (int i = 0; i < this.modulePositionSampleBuffer.length; i++) {
            this.modulePositionSampleBuffer[i] = new SwerveModulePosition();
        }

        final var alertGroup = "Drive/Module " + this.config.name + "/Alerts";

        // this.driveMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Drive Motor has active faults: ", AlertType.kError));
        // this.driveMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Drive Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
        // this.azimuthMotorActiveFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Azimuth Motor has active faults: ", AlertType.kError));
        // this.azimuthMotorStickyFaultsAlert = new DeviceFaultAlerts(new Alert(alertGroup, "Azimuth Motor has sticky faults: ", AlertType.kWarning), FaultType.StatorCurrentLimit, FaultType.SupplyCurrentLimit);
        // this.driveMotorStickyFaultClearer = new DeviceFaultClearer("Drive/Module " + this.config.name + "/Drive Motor Sticky Faults");
        // this.azimuthMotorStickyFaultClearer = new DeviceFaultClearer("Drive/Module " + this.config.name + "/Azimuth Motor Sticky Faults");

        this.driveMotorDisconnectedAlert = new Alert(alertGroup, "Drive Motor Disconnected", AlertType.kError);
        this.azimuthMotorDisconnectedAlert = new Alert(alertGroup, "Azimuth Motor Disconnected", AlertType.kError);
        this.azimuthEncoderDisconnectedAlert = new Alert(alertGroup, "Azimuth Encoder Disconnected", AlertType.kError);
        this.driveMotorDisconnectedGlobalAlert = new Alert("Module " + this.config.name + " Drive Motor Disconnected!", AlertType.kError);
        this.azimuthMotorDisconnectedGlobalAlert = new Alert("Module " + this.config.name + " Azimuth Motor Disconnected!", AlertType.kError);
        this.azimuthEncoderDisconnectedGlobalAlert = new Alert("Module " + this.config.name + " Azimuth Encoder Disconnected!", AlertType.kError);

        this.io.updateInputs(this.inputs);
        Logger.processInputs("Inpus/Drive/Module " + this.config.name, this.inputs);

        this.resetInternalAzimuthPosition(this.inputs.azimuthEncoder.getPositionRads());
    }

    /** Updates inputs and checks tunable numbers. */
    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Before");
        this.io.updateInputs(this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Update Inputs");
        Logger.processInputs("Inputs/Drive/Module " + this.config.name, this.inputs);
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Process Inputs");

        this.modulePositionSamples = new SwerveModulePosition[this.inputs.odometryDriveRads.length];
        for (int i = 0; i < this.modulePositionSamples.length; i++) {
            var angle = this.config.moduleForwardDirection.plus(
                Rotation2d.fromRadians(
                    DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.odometryAzimuthRads[i])
                )
            );
            var distanceMeters = DriveConstants.wheel.radiansToMeters(DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.odometryDriveRads[i]));
            this.modulePositionSampleBuffer[i].distanceMeters = distanceMeters;
            this.modulePositionSampleBuffer[i].angle = angle;
        }
        System.arraycopy(this.modulePositionSampleBuffer, 0, this.modulePositionSamples, 0, this.modulePositionSamples.length);

        var azimuthEncoderRads = DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.azimuthEncoder.getPositionRads()) + this.azimuthEncoderOffsetRads;
        var azimuthMotorRads = DriveConstants.azimuthMotorToCarriageRatio.applyUnsigned(this.inputs.azimuthMotor.encoder.getPositionRads()) + this.azimuthMotorOffsetRads;
        var measurementCount = 2;
        var azimuthMeanRads = (azimuthEncoderRads + azimuthMotorRads) / measurementCount;
        var azimuthEncoderVarRad2 = (azimuthEncoderRads - azimuthMeanRads) * (azimuthEncoderRads - azimuthMeanRads);
        var azimuthMotorVarRad2 = (azimuthMotorRads - azimuthMeanRads) * (azimuthMotorRads - azimuthMeanRads);
        var azimuthVarianceRad2 = (azimuthEncoderVarRad2 + azimuthMotorVarRad2) / Math.min(measurementCount - 1, 1);
        var azimuthStdDevRad = Math.sqrt(azimuthVarianceRad2);
        while (azimuthStdDevRad >= azimuthStdDevThreshold.get().in(Radians)) {
            measurementCount -= 1;
            if (Math.abs(azimuthEncoderRads - this.azimuthPositionRads) > Math.abs(azimuthMotorRads - this.azimuthPositionRads)) {
                azimuthEncoderRads = 0.0;
                azimuthMeanRads = (azimuthEncoderRads + azimuthMotorRads) / measurementCount;
                azimuthEncoderVarRad2 = 0.0;
                azimuthMotorVarRad2 = (azimuthMotorRads - azimuthMeanRads) * (azimuthMotorRads - azimuthMeanRads);
            } else {
                azimuthMotorRads = 0.0;
                azimuthMeanRads = (azimuthEncoderRads + azimuthMotorRads) / measurementCount;
                azimuthEncoderVarRad2 = (azimuthEncoderRads - azimuthMeanRads) * (azimuthEncoderRads - azimuthMeanRads);
                azimuthMotorVarRad2 = 0.0;
            }
            azimuthVarianceRad2 = (azimuthEncoderVarRad2 + azimuthMotorVarRad2) / Math.min(measurementCount - 1, 1);
            azimuthStdDevRad = Math.sqrt(azimuthVarianceRad2);
        }
        this.resetInternalAzimuthPosition(azimuthMeanRads);

        var angle = this.config.moduleForwardDirection.plus(
            Rotation2d.fromRadians(
                DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.azimuthEncoder.getPositionRads())
            )
        );
        this.modulePosition.angle = angle;
        this.moduleState.angle = angle;

        this.wheelAngularPositionRads = DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.driveMotor.encoder.getPositionRads());
        this.wheelAngularVelocityRadsPerSec = DriveConstants.driveMotorToWheelRatio.applyUnsigned(this.inputs.driveMotor.encoder.getVelocityRadsPerSec());

        this.modulePosition.distanceMeters = DriveConstants.wheel.radiansToMeters(this.wheelAngularPositionRads);
        this.moduleState.speedMetersPerSecond = DriveConstants.wheel.radiansToMeters(this.wheelAngularVelocityRadsPerSec);

        if (driveFFConsts.hasChanged(hashCode())) {
            driveFFConsts.get().update(this.driveFeedforward);
        }
        if (drivePIDConsts.hasChanged(hashCode())) {
            this.io.configDrivePID(drivePIDConsts.get());
        }
        if (azimuthPIDConsts.hasChanged(hashCode())) {
            this.io.configAzimuthPID(azimuthPIDConsts.get());
        }

        // this.driveMotorActiveFaultsAlert.updateFrom(this.inputs.driveMotorFaults.activeFaults);
        // this.driveMotorStickyFaultsAlert.updateFrom(this.inputs.driveMotorFaults.stickyFaults);
        // this.azimuthMotorActiveFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.activeFaults);
        // this.azimuthMotorStickyFaultsAlert.updateFrom(this.inputs.azimuthMotorFaults.stickyFaults);
        // this.driveMotorStickyFaultClearer.clear(this.inputs.driveMotorFaults.stickyFaults, this.io::clearDriveStickyFaults, DeviceFaults.allMask);
        // this.azimuthMotorStickyFaultClearer.clear(this.inputs.azimuthMotorFaults.stickyFaults, this.io::clearAzimuthStickyFaults, DeviceFaults.allMask);

        this.driveMotorDisconnectedAlert.set(!this.inputs.driveMotorConnected);
        this.azimuthMotorDisconnectedAlert.set(!this.inputs.azimuthMotorConnected);
        this.azimuthEncoderDisconnectedAlert.set(!this.inputs.azimuthEncoderConnected);
        this.driveMotorDisconnectedGlobalAlert.set(!this.inputs.driveMotorConnected);
        this.azimuthMotorDisconnectedGlobalAlert.set(!this.inputs.azimuthMotorConnected);
        this.azimuthEncoderDisconnectedGlobalAlert.set(!this.inputs.azimuthEncoderConnected);

        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name + "/Periodic");
        LoggedTracer.logEpoch("CommandScheduler Periodic/VirtualSubsystem Periodic/Drive/Module Periodic/" + this.config.name);
    }

    /**
     * Runs the module with the specified setpoint state. Must be called
     * periodically.
     */
    public void runSetpoint(SwerveModuleState setpoint) {
        setpoint.optimize(this.getAngle());
        
        var turnSetpoint = setpoint.angle;
        this.io.setAzimuthAngle(turnSetpoint.minus(this.config.moduleForwardDirection).getMeasure());

        setpoint.speedMetersPerSecond *= turnSetpoint.minus(this.getAngle()).getCos();

        var velocityRadPerSec = DriveConstants.driveMotorToWheelRatio.inverse().applyUnsigned(DriveConstants.wheel.metersToRadians(setpoint.speedMetersPerSecond));

        var ffout = this.driveFeedforward.calculateWithVelocities(this.moduleState.speedMetersPerSecond, setpoint.speedMetersPerSecond);

        var belowBrakeModeThreshold = Math.abs(setpoint.speedMetersPerSecond) < brakeModeThreshold.get().in(MetersPerSecond);

        this.io.setDriveVelocity(RadiansPerSecond.of(velocityRadPerSec), RadiansPerSecondPerSecond.zero(), Volts.of(ffout), belowBrakeModeThreshold);
    }

    /**
     * Runs the module with the specified voltage
     * Must be called periodically.
     */
    public void runVoltage(Measure<VoltageUnit> volts, Rotation2d moduleAngle) {
        this.io.setAzimuthAngle(moduleAngle.minus(this.config.moduleForwardDirection).getMeasure());
        this.io.setDriveVoltage(volts);
    }

    public void stopDrive(Optional<NeutralMode> neutralMode) {
        this.io.stopDrive(neutralMode);
    }
    public void stopTurn(Optional<NeutralMode> neutralMode) {
        this.io.stopAzimuth(neutralMode);
    }

    public SwerveModulePosition getModulePosition() {
        return this.modulePosition;
    }
    public SwerveModulePosition[] getModulePositionSamples() {
        return this.modulePositionSamples;
    }
    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getModuleState() {
        return this.moduleState;
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return this.getModulePosition().angle;
    }

    /** Returns the current drive position of the module in radians. */
    public double getWheelAngularPositionRads() {
        return this.wheelAngularPositionRads;
    }
    /** Returns the drive velocity in radians/sec. */
    public double getWheelAngularVelocityRadsPerSec() {
        return this.wheelAngularVelocityRadsPerSec;
    }
    /** Returns the current drive position of the module in radians. */
    public double getWheelLinearPositionMeters() {
        return this.getModulePosition().distanceMeters;
    }
    /** Returns the drive velocity in radians/sec. */
    public double getWheelLinearVelocityMetersPerSec() {
        return this.getModuleState().speedMetersPerSecond;
    }

    public double getDriveAppliedVolts() {
        return this.inputs.driveMotor.motor.getAppliedVolts();
    }
    public double getDriveStatorCurrentAmps() {
        return this.inputs.driveMotor.motor.getStatorCurrentAmps();
    }

    private void resetInternalAzimuthPosition(double positionRads) {
        this.azimuthPositionRads = positionRads;
        this.azimuthMotorOffsetRads = this.azimuthPositionRads - DriveConstants.azimuthMotorToCarriageRatio.applyUnsigned(this.inputs.azimuthMotor.encoder.getPositionRads());
        this.azimuthEncoderOffsetRads = this.azimuthPositionRads - DriveConstants.azimuthEncoderToCarriageRatio.applyUnsigned(this.inputs.azimuthEncoder.getPositionRads());
    }
}
