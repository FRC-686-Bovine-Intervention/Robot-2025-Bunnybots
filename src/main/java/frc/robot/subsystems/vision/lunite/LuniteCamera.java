package frc.robot.subsystems.vision.lunite;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.lunite.LuniteCameraIO.LuniteCameraFrame;
import frc.robot.subsystems.vision.lunite.LuniteCameraIO.LuniteCameraIOInputs;
import frc.robot.subsystems.vision.lunite.LuniteVisionConstants.LuniteCameraConstants;

public class LuniteCamera {
    private final LuniteCameraConstants camMeta;
    private final LuniteCameraIO io;
    private final LuniteCameraIOInputsAutoLogged inputs = new LuniteCameraIOInputsAutoLogged();

    private final Alert notConnectedAlert;

    public LuniteCamera(LuniteCameraConstants camMeta, LuniteCameraIO io) {
        this.camMeta = camMeta;
        this.io = io;

        notConnectedAlert = new Alert("Lunite Camera \"" + camMeta.hardwareName + "\" is not connected", AlertType.kError);
    }

    public Optional<LuniteCameraResult> periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Inputs/LuniteVision/" + camMeta.hardwareName, inputs);

        notConnectedAlert.set(!inputs.isConnected);
        return LuniteCameraResult.from(camMeta, inputs);
    }

    public static class LuniteCameraResult {
        public final LuniteCameraConstants camMeta;
        public final LuniteCameraFrame[] frames;

        private LuniteCameraResult(LuniteCameraConstants camMeta, LuniteCameraFrame[] frames) {
            this.camMeta = camMeta;
            this.frames = frames;
        }

        public static Optional<LuniteCameraResult> from(LuniteCameraConstants camMeta, LuniteCameraIOInputs inputs) {
             if (!inputs.isConnected || inputs.frames.length <= 0) return Optional.empty();

            return Optional.of(new LuniteCameraResult(
                camMeta,
                inputs.frames
            ));
        }
    }
}
