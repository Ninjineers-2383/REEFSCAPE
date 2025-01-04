package frc.robot.subsystems.piece_detection;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.piece_detection.PieceDetectionConstants.PieceDetectionConfig;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PieceDetectionIOPhoton implements PieceDetectionIO {
  private final String name;

  private final PhotonCamera camera;

  private final Alert cameraDisconnected;

  public PieceDetectionIOPhoton(String name, PieceDetectionConfig config) {
    this.name = name;

    camera = new PhotonCamera(name);

    cameraDisconnected = new Alert(name, name + " disconnected!", AlertType.kWarning);
  }

  @Override
  public void updateInputs(PieceDetectionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    List<PhotonPipelineResult> frontResult = camera.getAllUnreadResults();

    if (!frontResult.isEmpty()) {
      PhotonPipelineResult latestResult = frontResult.get(frontResult.size() - 1);
      if (latestResult.hasTargets()) {
        inputs.yaw = latestResult.getBestTarget().getYaw();
        inputs.pitch = latestResult.getBestTarget().getPitch();
        inputs.area = latestResult.getBestTarget().getArea();

        inputs.seesTarget = true;
      } else {
        inputs.yaw = 0.0;
        inputs.pitch = 0.0;
        inputs.area = 0.0;

        inputs.seesTarget = false;
      }
    }

    cameraDisconnected.set(!inputs.connected);
  }

  @Override
  public String getName() {
    return name;
  }
}
