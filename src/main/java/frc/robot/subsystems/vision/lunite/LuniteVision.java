package frc.robot.subsystems.vision.lunite;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.vision.lunite.LuniteCamera.LuniteCameraResult;
import frc.robot.subsystems.vision.lunite.LuniteCameraIO.LuniteCameraTarget;
import frc.util.VirtualSubsystem;
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
                    
                }
            }
        }
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

        public void updatePosWithFiltering(TrackedLunite newNote) {
            this.fieldPos = fieldPos.interpolate(newNote.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newNote.confidence;
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
