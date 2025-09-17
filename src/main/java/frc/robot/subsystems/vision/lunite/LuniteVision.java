package frc.robot.subsystems.vision.lunite;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.sound.midi.Track;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.lunite.LuniteCamera.LuniteCameraResult;
import frc.robot.subsystems.vision.lunite.LuniteCameraIO.LuniteCameraTarget;
import frc.util.LazyOptional;
import frc.util.VirtualSubsystem;
import frc.util.loggerUtil.LoggerUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.CameraMount;

public class LuniteVision extends VirtualSubsystem {
    private final LuniteCamera[] cameras;

    private static final String loggingKey = "Vision/Lunite/";

    private static final LoggedTunable<Distance> updateDistanceThreshold = LoggedTunable.from(loggingKey + "Updating/Update Distance Threshold", Meters::of, 5);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber(loggingKey + "Updating/Pos Updating Filtering Factor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber(loggingKey + "Confidence/Updating Filtering Factor", 0.5);
    private static final LoggedTunableNumber confidencePerAreaPercent = new LoggedTunableNumber(loggingKey + "Confidence/Per Area Percent", 1);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber(loggingKey + "Confidence/Decay Per Second", 3);
    private static final LoggedTunableNumber priorityPerConfidence = new LoggedTunableNumber(loggingKey + "Priority/Priority Per Confidence", 4);
    private static final LoggedTunableNumber priorityPerDistance = new LoggedTunableNumber(loggingKey + "Priority/Priority Per Distance", -2);
    private static final LoggedTunableNumber acquireConfidenceThreshold = new LoggedTunableNumber(loggingKey + "Target Threshold/Acquire", -2);
    private static final LoggedTunableNumber detargetConfidenceThreshold = new LoggedTunableNumber(loggingKey + "Target Threshold/Detarget", -3);

    private final ArrayList<TrackedLunite> luniteMemories = new ArrayList<>(3);

    private Optional<TrackedLunite> optIntakeTarget = Optional.empty();
    private boolean intakeTargetLocked = false;

    public LuniteVision(LuniteCamera... cameras) {
        System.out.println("[Init LuniteVision] Instantiating LuniteVision");
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("Lunite Cam Poses", Arrays.stream(
            new LuniteVisionConstants.LuniteCameraConstants[]{
                LuniteVisionConstants.luniteCamera
            })
            .map((constants) -> constants.mount.getFieldRelative())
            .toArray(Pose3d[]::new)
        );
        var results = Arrays.stream(cameras).map(LuniteCamera::periodic).toArray(LuniteCameraResult[]::new);
        for (var result : results) {
            var loggingKey = LuniteVision.loggingKey + "Results/" + result.camMeta.hardwareName;
            for (var frame : result.frames) {
                var frameTargets = Arrays
                    .stream(frame.targets)
                    .map((target) -> TrackedLunite.from(result.camMeta.mount, target))
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .toList()
                ;
                var connections = new ArrayList<TargetMemoryConnection>(luniteMemories.size() * frameTargets.size());
                luniteMemories.forEach(
                    (memory) -> frameTargets.forEach(
                        (target) -> {
                            if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.get().in(Meters)){
                                connections.add(new TargetMemoryConnection(memory, target));
                            }
                        }
                    )
                );
                connections.sort((a, b) -> Double.compare(a.getDistance(), b.getDistance()));
                var unusedMemories = new ArrayList<>(luniteMemories);
                var unusedTargets = new ArrayList<>(frameTargets);
                while (!connections.isEmpty()) {
                    var confirmedConnection = connections.get(0);
                    confirmedConnection.memory.updatePosWithFiltering(confirmedConnection.cameraTarget);
                    confirmedConnection.memory.updateConfidence();
                    unusedMemories.remove(confirmedConnection.memory);
                    unusedTargets.remove(confirmedConnection.cameraTarget);
                    connections.removeIf((connection) -> 
                        connection.memory == confirmedConnection.memory
                        || connection.cameraTarget == confirmedConnection.cameraTarget
                    );
                }
                unusedMemories.forEach((memory) -> {
                    if (RobotState.getInstance().getEstimatedGlobalPose().getTranslation().getDistance(memory.fieldPos) > RobotConstants.robotLength.in(Meters) * 0.5) {
                        memory.decayConfidence(1);
                    }
                });
                unusedTargets.forEach(luniteMemories::add);
                luniteMemories.removeIf((memory) -> memory.confidence <= 0);
                luniteMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()) || Double.isNaN(memory.fieldPos.getY()));
                luniteMemories.removeIf((memory) -> RobotState.getInstance().getEstimatedGlobalPose().getTranslation().getDistance(memory.fieldPos) <= 0.07);

                if (
                    optIntakeTarget.isPresent()
                    && (
                        optIntakeTarget.get().confidence < detargetConfidenceThreshold.get()
                        || !luniteMemories.contains(optIntakeTarget.get())
                    )
                ) {
                    optIntakeTarget = Optional.empty();
                }
                if (optIntakeTarget.isEmpty() || !intakeTargetLocked) {
                    optIntakeTarget = luniteMemories
                        .stream()
                        .filter((target) -> target.getPriority() >= acquireConfidenceThreshold.get())
                        .sorted((a, b) -> Double.compare(b.getPriority(), a.getPriority()))
                        .findFirst()
                    ;
                }

                Logger.recordOutput(LuniteVision.loggingKey + "Lunite Memories", luniteMemories.stream().map(TrackedLunite::toASPose).toArray(Pose3d[]::new));
                Logger.recordOutput(LuniteVision.loggingKey + "Lunite Confidence", luniteMemories.stream().mapToDouble((lunite) -> lunite.confidence).toArray());
                Logger.recordOutput(LuniteVision.loggingKey + "Lunite Priority", luniteMemories.stream().mapToDouble(TrackedLunite::getPriority).toArray());
                Logger.recordOutput(LuniteVision.loggingKey + "Target", LoggerUtil.toArray(optIntakeTarget.map(TrackedLunite::toASPose), Pose3d[]::new));
                Logger.recordOutput(LuniteVision.loggingKey + "Locked Target", LoggerUtil.toArray(optIntakeTarget.filter((a) -> intakeTargetLocked).map(TrackedLunite::toASPose).map(Pose3d::getTranslation), Translation3d[]::new));
            }
        }
    }

    public DoubleSupplier applyDotProduct(Supplier<ChassisSpeeds> joystickFieldRelative) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getEstimatedGlobalPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var joystickSpeed = joystickFieldRelative.get();
            var joy = new Translation2d(joystickSpeed.vxMetersPerSecond, joystickSpeed.vyMetersPerSecond);
            var throttle = targetRelRobotNormalized.toVector().dot(joy.toVector());
            return throttle;
        }).orElse(0.0);
    }

    public LazyOptional<ChassisSpeeds> getAutoIntakeTransSpeed(DoubleSupplier throttleSupplier) {
        return () -> optIntakeTarget.map((target) -> {
            var robotTrans = RobotState.getInstance().getEstimatedGlobalPose().getTranslation();
            var targetRelRobot = target.fieldPos.minus(robotTrans);
            var targetRelRobotNormalized = targetRelRobot.div(targetRelRobot.getNorm());
            var finalTrans = targetRelRobotNormalized.times(throttleSupplier.getAsDouble());
            return new ChassisSpeeds(finalTrans.getX(), finalTrans.getY(), 0);
        });
    }

    public LazyOptional<Translation2d> autoIntakeTargetLocation() {
        return () -> optIntakeTarget.map((target) -> target.fieldPos);
    }

    public boolean hasTarget() {
        return optIntakeTarget.isPresent();
    }

    public boolean targetLocked() {
        return intakeTargetLocked;
    }

    public void clearMemory() {
        luniteMemories.clear();
        optIntakeTarget = Optional.empty();
    }

    public Command autoIntake(DoubleSupplier throttle, BooleanSupplier noLunite, Drive drive) {
        return 
            Commands.runOnce(() -> intakeTargetLocked = true)
            .alongWith(
                drive.translationSubsystem.fieldRelative(getAutoIntakeTransSpeed(throttle).orElseGet(ChassisSpeeds::new)),
                drive.rotationalSubsystem.pointTo(autoIntakeTargetLocation(), () -> RobotConstants.intakeForward)
            )
            .onlyWhile(() -> noLunite.getAsBoolean() && optIntakeTarget.isPresent())
            .finallyDo(() -> intakeTargetLocked = false)
            .withName("Auto Intake")
        ;
    }

    private static record TargetMemoryConnection(TrackedLunite memory, TrackedLunite cameraTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(cameraTarget.fieldPos);
        }
    }

    public static class TrackedLunite implements StructSerializable {
        public Translation2d fieldPos;
        public double confidence;

        public TrackedLunite(Translation2d fieldPos, double confidence) {
            this.fieldPos = fieldPos;
            this.confidence = confidence * confUpdatingFilteringFactor.get();
        }

        public static Optional<TrackedLunite> from(CameraMount mount, LuniteCameraTarget target) {
            var camTransform = mount.getRobotRelative();
            // var targetCamViewTransform = camTransform.plus(
            //     new Transform3d(
            //         Translation3d.kZero,
            //         target.cameraPose
            //     )
            // );
            // var distOut = targetCamViewTransform.getTranslation().getZ() / Math.tan(targetCamViewTransform.getRotation().getY());
            // var distOff = distOut * Math.tan(targetCamViewTransform.getRotation().getZ());
            // var camToTargetTranslation = new Translation3d(distOut, distOff, -camTransform.getZ());
            // var fieldPos = new Pose3d(RobotState.getInstance().getPose())
            //     .transformBy(camTransform)
            //     .transformBy(new Transform3d(camToTargetTranslation, Rotation3d.kZero))
            //     .toPose2d()
            //     .getTranslation()
            // ;
            var cameraToTargetVectorRobotRelative = target.cameraToTargetVector.rotateBy(camTransform.getRotation());
            if (cameraToTargetVectorRobotRelative.getZ() >= 0) {
                // Target above horizon, ignore
                return Optional.empty();
            }
            var toFloorScalingFactor = camTransform.getTranslation().getZ() / -cameraToTargetVectorRobotRelative.getZ();
            var robotToTarget = cameraToTargetVectorRobotRelative.times(toFloorScalingFactor).plus(camTransform.getTranslation());
            var fieldPose = RobotState.getInstance().getEstimatedGlobalPose().transformBy(new Transform2d(robotToTarget.toTranslation2d(), Rotation2d.kZero));

            var confidence = Math.sqrt(target.area) * confidencePerAreaPercent.get();

            return Optional.of(new TrackedLunite(fieldPose.getTranslation(), confidence));
        }

        public void updateConfidence() {
            confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        }

        public void updatePosWithFiltering(TrackedLunite newLunite) {
            this.fieldPos = fieldPos.interpolate(newLunite.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newLunite.confidence;
        }

        public void decayConfidence(double rate) {
            this.confidence -= confidenceDecayPerSecond.get() * rate * RobotConstants.rioUpdatePeriodSecs;
        }

        public double getPriority() {
            var pose = RobotState.getInstance().getEstimatedGlobalPose();
            var FORR = fieldPos.minus(pose.getTranslation());
            var rotation = pose.getRotation().minus(RobotConstants.intakeForward);
            return 
                confidence * priorityPerConfidence.get() *
                VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit()) + 
                FORR.getNorm() * priorityPerDistance.get()
            ;
        }

        public Pose3d toASPose() {
            return new Pose3d(
                new Translation3d(
                    fieldPos.getMeasureX(),
                    fieldPos.getMeasureY(),
                    FieldConstants.luniteWidth
                ),
                new Rotation3d(
                    Degrees.of(180),
                    Degrees.zero(),
                    Degrees.zero()
                )
            );
        }

        public static final TrackedLuniteStruct struct = new TrackedLuniteStruct();
        public static class TrackedLuniteStruct implements Struct<TrackedLunite> {
            @Override
            public Class<TrackedLunite> getTypeClass() {
                return TrackedLunite.class;
            }

            @Override
            public String getTypeName() {
                return "TrackedNote";
            }

            @Override
            public int getSize() {
                return Translation2d.struct.getSize() * 1 + kSizeDouble * 1;
            }

            @Override
            public String getSchema() {
                return "Translation2d fieldPos;double confidence";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct<?>[] {Translation2d.struct};
            }

            @Override
            public TrackedLunite unpack(ByteBuffer bb) {
                var fieldPos = Translation2d.struct.unpack(bb);
                var confidence = bb.getDouble();
                return new TrackedLunite(fieldPos, confidence);
            }

            @Override
            public void pack(ByteBuffer bb, TrackedLunite value) {
                Translation2d.struct.pack(bb, value.fieldPos);
                bb.putDouble(value.confidence);
            }
        }
    }
}
