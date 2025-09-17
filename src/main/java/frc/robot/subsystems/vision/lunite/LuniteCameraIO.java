package frc.robot.subsystems.vision.lunite;

import java.nio.ByteBuffer;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;

public interface LuniteCameraIO {
    
    @AutoLog
    public class LuniteCameraIOInputs {
        public boolean isConnected;
        public LuniteCameraFrame[] frames = new LuniteCameraFrame[0];
    }

    public default void updateInputs(LuniteCameraIOInputs inputs) {}

    public static class LuniteCameraFrame {
        public final double timestamp;
        public final LuniteCameraTarget[] targets;

        public LuniteCameraFrame(double timestamp, LuniteCameraTarget[] targets) {
            this.timestamp = timestamp;
            this.targets = targets;
        }
    }

    public static class LuniteCameraTarget implements StructSerializable {
        public final Translation3d cameraToTargetVector;
        public final double area;

        public LuniteCameraTarget(Translation3d cameraToTargetVector, double area) {
            this.cameraToTargetVector = cameraToTargetVector;
            this.area = area;
        }

        public static final LuniteCameraTargetStruct struct = new LuniteCameraTargetStruct();
        public static class LuniteCameraTargetStruct implements Struct<LuniteCameraTarget> {
            @Override
            public Class<LuniteCameraTarget> getTypeClass() {
                return LuniteCameraTarget.class;
            }

            @Override
            public String getTypeName() {
                return "LuniteTarget";
            }

            @Override
            public int getSize() {
                return Translation3d.struct.getSize() * 1 + kSizeDouble * 1;
            }

            @Override
            public String getSchema() {
                return "Translation3d cameraPose;double area";
            }

            @Override
            public Struct<?>[] getNested() {
                return new Struct[]{Translation3d.struct};
            }

            @Override
            public LuniteCameraTarget unpack(ByteBuffer bb) {
                var cameraToTargetVector = Translation3d.struct.unpack(bb);
                var area = bb.getDouble();
                return new LuniteCameraTarget(cameraToTargetVector, area);
            }

            @Override
            public void pack(ByteBuffer bb, LuniteCameraTarget value) {
                Translation3d.struct.pack(bb, value.cameraToTargetVector);
                bb.putDouble(value.area);
            }
        }
    }
}
