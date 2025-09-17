package frc.robot.subsystems.vision.lunite;

import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.robotStructure.CameraMount;

public class LuniteVisionConstants {
    public static class LuniteCameraConstants extends CameraConstants {

        public LuniteCameraConstants(String hardwareName, CameraMount mount) {
            super(hardwareName, mount);
        }
    }

    public static final LuniteCameraConstants luniteCamera = new LuniteCameraConstants(
        "Lunite Cam",
        null //todo
    );
}
