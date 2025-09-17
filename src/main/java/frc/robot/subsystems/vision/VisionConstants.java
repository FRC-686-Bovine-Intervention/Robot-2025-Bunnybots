package frc.robot.subsystems.vision;

import frc.util.robotStructure.CameraMount;

public final class VisionConstants {
     public static class CameraConstants {
        public final String hardwareName;
        public final CameraMount mount;

        public CameraConstants(String hardwareName, CameraMount mount) {
            this.hardwareName = hardwareName;
            this.mount = mount;
        }
    }
}