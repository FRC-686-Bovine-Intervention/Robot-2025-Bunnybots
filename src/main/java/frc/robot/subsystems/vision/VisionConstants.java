package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.util.geometry.GeomUtil;

public final class VisionConstants {
    public static final Transform3d frontLeftMount = new Transform3d(
        new Translation3d(
            Meters.of(-0.083008),
            Meters.of(+0.244626),
            Meters.of(+0.396240)
        ),
        GeomUtil.rotation3dBuilder()
            .yaw(Degrees.of(-7.5))
        .build()
    );
    public static final Transform3d frontRightMount = new Transform3d(
        new Translation3d(
            Meters.of(-0.083008),
            Meters.of(-0.244626),
            Meters.of(+0.396240)
        ),
        GeomUtil.rotation3dBuilder()
            .yaw(Degrees.of(+7.5))
        .build()
    );
    public static final Transform3d backLeftMount = new Transform3d(
        new Translation3d(
            Meters.of(-0.175207),
            Meters.of(+0.254154),
            Meters.of(+0.403667)
        ),
        GeomUtil.rotation3dBuilder()
            .yaw(Degrees.of(+175))
            .pitch(Degrees.of(+15))
        .build()
    );
    public static final Transform3d backRightMount = new Transform3d(
        new Translation3d(
            Meters.of(-0.175207),
            Meters.of(-0.254154),
            Meters.of(+0.403667)
        ),
        GeomUtil.rotation3dBuilder()
            .yaw(Degrees.of(-175))
            .pitch(Degrees.of(+15))
        .build()
    );

    public static final Transform3d intakeMount = new Transform3d(
        new Translation3d(
            Inches.of(-26),
            Inches.of(+0),
            Inches.of(+6)
        ),
        GeomUtil.rotation3dBuilder()
            .yaw(Degrees.of(180))
        .build()
    );
}
