package frc.robot.subsystems.vision.object;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.struct.Struct;
import frc.robot.RobotState;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotConstants;
import frc.robot.subsystems.vision.cameras.CameraIO.CameraTarget;
import frc.util.LoggedTracer;
import frc.util.loggerUtil.tunables.LoggedTunable;
import frc.util.loggerUtil.tunables.LoggedTunableNumber;
import frc.util.robotStructure.CameraMount;
import frc.util.rust.iter.Iterator;

public class ObjectVision {
    private final ObjectPipeline[] pipelines;

    private static final LoggedTunable<Distance> updateDistanceThreshold = LoggedTunable.from("Vision/Object/Updating/Update Distance Threshold", Meters::of, 0.1);
    private static final LoggedTunableNumber posUpdatingFilteringFactor =  LoggedTunable.from("Vision/Object/Updating/Pos Updating Filtering Factor", 0.8);
    //private static final LoggedTunableNumber confUpdatingFilteringFactor = LoggedTunable.from("Vision/Object/Confidence/Updating Filtering Factor", 0.5);
    private static final LoggedTunableNumber confidenceDecayPerSecond =    LoggedTunable.from("Vision/Object/Confidence/Decay Per Second", 3);
    private static final LoggedTunableNumber priorityPerConfidence =       LoggedTunable.from("Vision/Object/Priority/Priority Per Confidence", 4);
    private static final LoggedTunableNumber priorityPerDistance =         LoggedTunable.from("Vision/Object/Priority/Priority Per Distance", 2);
    private static final LoggedTunableNumber acquireConfidenceThreshold =  LoggedTunable.from("Vision/Object/Target Threshold/Acquire", 0.75);
    private static final LoggedTunableNumber detargetConfidenceThreshold = LoggedTunable.from("Vision/Object/Target Threshold/Detarget", 0.5);
    
    private List<TrackedObject> trackedObjects = new ArrayList<>(0);
    private Optional<TrackedObject> optIntakeTarget = Optional.empty();
    private boolean intakeTargetLocked = false;
    private ChassisSpeeds desiredRobotRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(0.0, 0.0, 0.0, Rotation2d.kZero);

    public ObjectVision(ObjectPipeline... pipelines) {
        System.out.println("[Init ObjectVision] Instantiating ObjectVision");
        this.pipelines = pipelines;
    }

    public void periodic() {
        LoggedTracer.logEpoch("CommandScheduler Periodic/ObjectVision/Before");
        for (var pipeline : this.pipelines) {
            var frames = pipeline.getFrames();
            var loggingKey = "Vision/Objects/Results/" + pipeline.pipelineIndex + "/";
            for (var frame : frames) {
                var frameTargets = Iterator.of(frame.targets)
                    .map((target) -> TrackedObject.from(pipeline.camera.mount, target))
                    .filter(Optional::isPresent)
                    .map(Optional::get)
                    .collect_arraylist()
                ;

                for (var trackedObject : trackedObjects) {
                    trackedObject.resetUpdated();
                }
                // Update tracked objects
                for (var frameTarget : frameTargets) {
                    for (var trackedObject : trackedObjects) {
                        trackedObject.decayConfidence(confidenceDecayPerSecond.getAsDouble());

                        if (trackedObject.fieldPos.getDistance(frameTarget.fieldPos) < updateDistanceThreshold.get().in(Meters) && !trackedObject.updated) {
                            trackedObject.updateWithFiltering(frameTarget);
                        } else {
                            trackedObjects.add(frameTarget);
                        }

                        if (trackedObject.confidence < detargetConfidenceThreshold.getAsDouble()) {
                            trackedObjects.remove(trackedObject);
                        }
                    }
                }

                if (optIntakeTarget.isPresent()) {
                    optIntakeTarget.get().resetUpdated();
                    optIntakeTarget.get().decayConfidence(confidenceDecayPerSecond.getAsDouble());

                    for (var trackedObject : trackedObjects) {
                        if (trackedObject.fieldPos.getDistance(optIntakeTarget.get().fieldPos) < updateDistanceThreshold.get().in(Meters) && !optIntakeTarget.get().updated) {
                            optIntakeTarget.get().updateWithFiltering(trackedObject);
                        }
                    }

                    if (optIntakeTarget.get().confidence < detargetConfidenceThreshold.get() || !trackedObjects.contains(optIntakeTarget.get())) {
                        optIntakeTarget = Optional.empty();
                    }
                }

                if (optIntakeTarget.isEmpty() || !intakeTargetLocked) {
                    optIntakeTarget = trackedObjects.stream()
                        .filter(target -> target.confidence > acquireConfidenceThreshold.get())
                        .sorted((a, b) -> {
                            return Double.compare(a.getPriority(desiredRobotRelativeSpeeds), b.getPriority(desiredRobotRelativeSpeeds));
                        })
                        .findFirst();
                }                

                Logger.recordOutput(loggingKey + "Targets", Iterator.of(trackedObjects).map((target) -> target.toASPose()).collect_array(Pose3d[]::new));
                Logger.recordOutput(loggingKey + "Locked Target", optIntakeTarget.map(TrackedObject::toASPose).orElse(null));
            }
        }
    }
    
    public void updateTargetRobotRelativeSpeeds(ChassisSpeeds chassisSpeeds) {
        desiredRobotRelativeSpeeds = chassisSpeeds;
    }

    public void unlockIntakeTarget() {
        intakeTargetLocked = false;
    }

    public void lockIntakeTarget() {
        intakeTargetLocked = true;
    }

    public double getIntakeYSpeedFromRobotSpeeds(ChassisSpeeds inputSpeeds) {
        if (optIntakeTarget.isEmpty() || !intakeTargetLocked) {
            return 0.0;
        }
        updateTargetRobotRelativeSpeeds(inputSpeeds);
        var robotPose = RobotState.getInstance().getEstimatedGlobalPose(); //TODO: Replace with intaking position
        var intakeTargetPos = optIntakeTarget.get().fieldPos;
        var objectRelativeToRobot = new Pose2d(intakeTargetPos, Rotation2d.kZero).relativeTo(robotPose).getTranslation();
        return (objectRelativeToRobot.getY() / objectRelativeToRobot.getX()) * desiredRobotRelativeSpeeds.vxMetersPerSecond;
    }

    // private static record TargetMemoryConnection(TrackedObject memory, TrackedObject cameraTarget) {
    //     public double getDistance() {
    //         return memory.fieldPos.getDistance(cameraTarget.fieldPos);
    //     }
    // }

    public static class TrackedObject {
        public int type;
        public Translation2d fieldPos;
        public double confidence;
        public boolean updated;

        public TrackedObject(int type, Translation2d fieldPos, double confidence) {
            this.type = type;
            this.fieldPos = fieldPos;
            this.confidence = confidence;
            this.updated = true;
        }

        public static Optional<TrackedObject> from(CameraMount mount, CameraTarget target) {
            var camRobotPose = mount.getRobotRelative();
            var camFieldPose = mount.getFieldRelative();
            
            var tty = target.pitchRads + camRobotPose.getRotation().getY();
            var ttx = -target.yawRads;

            var height = FieldConstants.luniteDimensions.getZ() / 2.0 - camRobotPose.getTranslation().getZ();
            var h = height / Math.tan(tty);
            var x = Math.cos(ttx) * h;
            var y = Math.sin(ttx) * h;

            var objectFieldPose = camFieldPose.toPose2d().transformBy(new Transform2d(
                new Translation2d(
                    x,
                    y
                ),
                Rotation2d.kZero
            ));
            
            return Optional.of(new TrackedObject(target.objectClassID, objectFieldPose.getTranslation(), target.objectConfidence));
        }

        // public void updateConfidence() {
        //     confidence += confidence * MathUtil.clamp(1 - confUpdatingFilteringFactor.get(), 0, 1); 
        // }

        public void updateWithFiltering(TrackedObject newObject) {
            this.fieldPos = fieldPos.interpolate(newObject.fieldPos, posUpdatingFilteringFactor.get());
            this.confidence = newObject.confidence;
            this.updated = true;
        }

        public void resetUpdated() {
            this.updated = false;
        }

        public void decayConfidence(double rate) {
            this.confidence -= confidenceDecayPerSecond.get() * rate * RobotConstants.rioUpdatePeriodSecs;
        }

        public double getPriority(ChassisSpeeds desiredRobotRelativeSpeeds) {
            // var pose = RobotState.getInstance().getEstimatedGlobalPose();
            // var FORR = fieldPos.minus(pose.getTranslation());
            // var rotation = pose.getRotation().minus(RobotConstants.intakeForward);
            // return 
            //     confidence * priorityPerConfidence.get() *
            //     VecBuilder.fill(rotation.getCos(), rotation.getSin()).dot(FORR.toVector().unit()) + 
            //     FORR.getNorm() * priorityPerDistance.get()
            // ;
            var intakePose = RobotState.getInstance().getEstimatedGlobalPose(); //TODO: Replace with a mount where the intake would be
            var rel = new Pose2d(this.fieldPos, Rotation2d.kZero).relativeTo(intakePose);

            double dist = Math.hypot(rel.getX(), 2 * rel.getY());
            double angle = Math.atan2(rel.getY(), rel.getX());

            double score = 0;

            var chassisSpeedsAngle = Math.atan2(desiredRobotRelativeSpeeds.vxMetersPerSecond, desiredRobotRelativeSpeeds.vyMetersPerSecond);
            if (Math.abs(angle - chassisSpeedsAngle) < Math.PI / 2 && (angle > Math.PI/2 || angle < -Math.PI/2)) {
                score = (1 / (dist * priorityPerDistance.getAsDouble())) * (this.confidence * priorityPerConfidence.getAsDouble());
            }

            return score;
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
