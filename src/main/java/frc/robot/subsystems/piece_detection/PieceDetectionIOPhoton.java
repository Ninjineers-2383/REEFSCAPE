package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.piece_detection.PieceDetectionConstants.PieceDetectionConfig;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class PieceDetectionIOPhoton implements PieceDetectionIO {
  private final String name;

  private final PieceDetectionConfig config;

  private final PhotonCamera camera;

  private final LoggedTunableNumber kDistance;

  private final Alert cameraDisconnected;

  public PieceDetectionIOPhoton(String name, PieceDetectionConfig config, double kDistance) {
    this.name = name;

    this.config = config;

    camera = new PhotonCamera(name);

    this.kDistance = new LoggedTunableNumber(name + "/Gains/kDistance", kDistance);

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

        inputs.distance = kDistance.get() / Math.sqrt(inputs.area);

        inputs.robotToPieceTransform =
            config
                .robotToCameraTransform()
                .plus(
                    new Transform3d(
                        new Translation3d(
                            inputs.distance * Math.cos(Math.toRadians(inputs.pitch)),
                            inputs.distance * Math.sin(Math.toRadians(inputs.yaw)),
                            config.robotToCameraTransform().getZ()
                                - inputs.distance * Math.sin(Math.toRadians(inputs.pitch))),
                        new Rotation3d()));

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
