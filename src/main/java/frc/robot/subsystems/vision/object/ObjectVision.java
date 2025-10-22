package frc.robot.subsystems.vision.object;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraTarget;
import frc.util.LazyOptional;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.LoggerUtil;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.CameraMount;
import frc.util.rust.iter.Iterator;

public class ObjectVision {
    private final ObjectPipeline[] pipelines;

    private static final String loggingKey = "Vision/Object/";

    private static final LoggedTunable<Distance> updateDistanceThreshold = LoggedTunable.from(loggingKey + "Updating/Update Distance Threshold", Meters::of, 5);
    private static final LoggedTunableNumber posUpdatingFilteringFactor = new LoggedTunableNumber(loggingKey + "Updating/Pos Updating Filtering Factor", 0.8);
    private static final LoggedTunableNumber confUpdatingFilteringFactor = new LoggedTunableNumber(loggingKey + "Confidence/Updating Filtering Factor", 0.5);
    private static final LoggedTunableNumber confidenceDecayPerSecond = new LoggedTunableNumber(loggingKey + "Confidence/Decay Per Second", 3);
    private static final LoggedTunableNumber priorityPerConfidence = new LoggedTunableNumber(loggingKey + "Priority/Priority Per Confidence", 4);
    private static final LoggedTunableNumber priorityPerDistance = new LoggedTunableNumber(loggingKey + "Priority/Priority Per Distance", -2);
    private static final LoggedTunableNumber acquireConfidenceThreshold = new LoggedTunableNumber(loggingKey + "Target Threshold/Acquire", -2);
    private static final LoggedTunableNumber detargetConfidenceThreshold = new LoggedTunableNumber(loggingKey + "Target Threshold/Detarget", -3);
    
    private final ArrayList<TrackedObject> objectMemories = new ArrayList<>(3);

    private Optional<TrackedObject> optIntakeTarget = Optional.empty();
    private boolean intakeTargetLocked = false;

    public ObjectVision(ObjectPipeline... pipelines) {
        System.out.println("[Init ObjectVision] Instantiating ObjectVision");
        this.pipelines = pipelines;
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/ObjectVision/Before");
        List<TrackedObject> allTrackedObjects = new ArrayList<>(this.pipelines.length * 3);
        for (var pipeline : this.pipelines) {
            var frames = pipeline.getFrames();
            var loggingKey = "Vision/Objects/Results/" + pipeline.pipelineIndex;
            var tracingKey = "CommandScheduler Periodic/Objects/Process Results/" + pipeline.pipelineIndex;
            for (var frame : frames) {
                var frameTargets = Iterator.of(frame.targets)
                    .map((target) -> TrackedObject.from(pipeline.camera.mount, target))
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .collect_arraylist()
                ;
                var connections = new ArrayList<TargetMemoryConnection>(objectMemories.size() * frameTargets.size());
                objectMemories.forEach(
                    (memory) -> frameTargets.forEach(
                        (target) -> {
                            if(memory.fieldPos.getDistance(target.fieldPos) < updateDistanceThreshold.get().in(Meters)){
                                connections.add(new TargetMemoryConnection(memory, target));
                            }
                        }
                    )
                );
                connections.sort((a, b) -> Double.compare(a.getDistance(), b.getDistance()));
                var unusedMemories = new ArrayList<>(objectMemories);
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
                unusedTargets.forEach(objectMemories::add);
                objectMemories.removeIf((memory) -> memory.confidence <= 0);
                objectMemories.removeIf((memory) -> Double.isNaN(memory.fieldPos.getX()) || Double.isNaN(memory.fieldPos.getY()));
                objectMemories.removeIf((memory) -> RobotState.getInstance().getEstimatedGlobalPose().getTranslation().getDistance(memory.fieldPos) <= 0.07);

                if (
                    optIntakeTarget.isPresent()
                    && (
                        optIntakeTarget.get().confidence < detargetConfidenceThreshold.get()
                        || !objectMemories.contains(optIntakeTarget.get())
                    )
                ) {
                    optIntakeTarget = Optional.empty();
                }
                if (optIntakeTarget.isEmpty() || !intakeTargetLocked) {
                    optIntakeTarget = objectMemories
                        .stream()
                        .filter((target) -> target.getPriority() >= acquireConfidenceThreshold.get())
                        .sorted((a, b) -> Double.compare(b.getPriority(), a.getPriority()))
                        .findFirst()
                    ;
                }

                Logger.recordOutput(loggingKey + "Object Memories", objectMemories.stream().map(TrackedObject::toASPose).toArray(Pose3d[]::new));
                Logger.recordOutput(loggingKey + "Object Confidence", objectMemories.stream().mapToDouble((object) -> object.confidence).toArray());
                Logger.recordOutput(loggingKey + "Object Priority", objectMemories.stream().mapToDouble(TrackedObject::getPriority).toArray());
                Logger.recordOutput(loggingKey + "Target", LoggerUtil.toArray(optIntakeTarget.map(TrackedObject::toASPose), Pose3d[]::new));
                Logger.recordOutput(loggingKey + "Locked Target", LoggerUtil.toArray(optIntakeTarget.filter((a) -> intakeTargetLocked).map(TrackedObject::toASPose).map(Pose3d::getTranslation), Translation3d[]::new));
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
        objectMemories.clear();
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

    private static record TargetMemoryConnection(TrackedObject memory, TrackedObject cameraTarget) {
        public double getDistance() {
            return memory.fieldPos.getDistance(cameraTarget.fieldPos);
        }
    }

    public static class TrackedObject {
        public int type;
        public Translation2d fieldPos;
        public double confidence;

        public TrackedObject(int type, Translation2d fieldPos, double confidence) {
            this.type = type;
            this.fieldPos = fieldPos;
            this.confidence = confidence;
        }

        public static Optional<TrackedObject> from(CameraMount mount, CameraTarget target) {
            var camTransform = mount.getRobotRelative();
            var cameraToTargetVector = new Translation3d(
                1,
                0,
                0
            ).rotateBy(new Rotation3d(
                0,
                0,
                -target.yawRads
            )).rotateBy(new Rotation3d(
                0,
                -target.pitchRads,
                0
            ));
            var cameraToTargetVectorRobotRelative = cameraToTargetVector.rotateBy(camTransform.getRotation());
            if (cameraToTargetVectorRobotRelative.getZ() >= 0) {
                // Target above horizon, ignore
                return Optional.empty();
            }
            var toFloorScalingFactor = camTransform.getTranslation().getZ() / -cameraToTargetVectorRobotRelative.getZ();
            var robotToTarget = cameraToTargetVectorRobotRelative.times(toFloorScalingFactor).plus(camTransform.getTranslation());
            var fieldPose = RobotState.getInstance().getEstimatedGlobalPose().transformBy(new Transform2d(robotToTarget.toTranslation2d(), Rotation2d.kZero));

            return Optional.of(new TrackedObject(target.objectClassID, fieldPose.getTranslation(), target.objectConfidence));
        }

        public void updateConfidence() {
            confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        }

        public void updatePosWithFiltering(TrackedObject newObject) {
            this.fieldPos = fieldPos.interpolate(newObject.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newObject.confidence;
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
                    FieldConstants.luniteDimensions.getMeasureZ()
                ),
                new Rotation3d(
                    Degrees.of(180),
                    Degrees.zero(),
                    Degrees.zero()
                )
            );
        }

        public static final TrackedObjectStruct struct = new TrackedObjectStruct();
        public static class TrackedObjectStruct implements Struct<TrackedObject> {
            @Override
            public Class<TrackedObject> getTypeClass() {
                return TrackedObject.class;
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
            public TrackedObject unpack(ByteBuffer bb) {
                var type = bb.getInt();
                var fieldPos = Translation2d.struct.unpack(bb);
                var confidence = bb.getDouble();
                return new TrackedObject(type, fieldPos, confidence);
            }

            @Override
            public void pack(ByteBuffer bb, TrackedObject value) {
                Translation2d.struct.pack(bb, value.fieldPos);
                bb.putDouble(value.confidence);
            }
        }
    }

    
}
