package frc.robot.subsystems.vision.lunite;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.subsystems.vision.VisionConstants.CameraConstants;
import frc.util.rust.iter.Iterator;

public class LuniteCameraIOPhotonVision implements LuniteCameraIO{
    private final PhotonCamera photonCam;
    private final CameraConstants camMeta;

    public LuniteCameraIOPhotonVision(CameraConstants camMeta) {
        this.camMeta = camMeta;
        photonCam = new PhotonCamera(this.camMeta.hardwareName);
    }

    @Override
    public void updateInputs(LuniteCameraIOInputs inputs) {
        inputs.isConnected = photonCam.isConnected();

        if (!inputs.isConnected) return;
        var results = photonCam.getAllUnreadResults();

        inputs.frames = Iterator.of(results).map(this::frameFromResult).collect_array(LuniteCameraFrame[]::new);
    }

    private LuniteCameraFrame frameFromResult(PhotonPipelineResult result) {
        var timestamp = result.getTimestampSeconds();
        var targets = Iterator.of(result.getTargets()).map(this::targetFromPhotonTarget).collect_array(LuniteCameraTarget[]::new);
        
        return new LuniteCameraFrame(timestamp, targets);
    }

    private LuniteCameraTarget targetFromPhotonTarget(PhotonTrackedTarget photonTarget) {
        return new LuniteCameraTarget(
            new Translation3d(
                1,
                0,
                0
            ).rotateBy(new Rotation3d(
                0,
                0,
                Radians.convertFrom(-photonTarget.getYaw(), Degrees)
            )).rotateBy(new Rotation3d(
                0,
                Radians.convertFrom(-photonTarget.getPitch(), Degrees),
                0
            )),
            photonTarget.getArea()
        );
    }
}
