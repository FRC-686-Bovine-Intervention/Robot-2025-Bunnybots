package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.geometry.GeomUtil;
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

    public static final Transform3d frontLeftMount = new Transform3d(
        new Translation3d(
            Inches.of(+7.953314),
            Inches.of(+11.042047),
            Inches.of(+9.617869)
        ),
        GeomUtil.rotation3dBuilder()
            .pitch(Math.toRadians(27.5))
        .build()
    );
    public static final Transform3d frontRightMount = new Transform3d(
        new Translation3d(
            Inches.of(+7.953314),
            Inches.of(-11.042047),
            Inches.of(+9.617869)
        ),
        GeomUtil.rotation3dBuilder()
            .pitch(Math.toRadians(27.5))
        .build()
    );

    public static final Transform3d intakeMount = new Transform3d(
        new Translation3d(
            Meters.of(+0.1),
            Meters.of(+0),
            Meters.of(+0)
        ),
        GeomUtil.rotation3dBuilder()
        .build()
    );
    public static final CameraMount questNavMount = new CameraMount(new Transform3d(
        new Translation3d(
            // Inches.of(11.3125),
            // Inches.of(9.5),
            // Inches.zero()
            // -0.2431546438240109, -0.22893749281190218,0
            // 0.24807059617995136, 0.22519556696985554, 0
            0.19019748204138284, 0.24143232491706343, 0
        ),
        new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(90))
    ));
}
