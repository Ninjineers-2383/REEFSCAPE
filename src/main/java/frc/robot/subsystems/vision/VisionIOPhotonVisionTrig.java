package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVisionTrig implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final Supplier<Rotation2d> gyroRotationSupplier;
  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVisionTrig(
      String name, Transform3d robotToCamera, Supplier<Rotation2d> gyroRotation) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;

    gyroRotationSupplier = gyroRotation;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Integer> tagIds = new HashSet<>();
    List<PoseObservation> poseObservations = new LinkedList<>();

    Set<Double> bestDistances = new HashSet<>();
    Set<Double> worstDistances = new HashSet<>();

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        PhotonTrackedTarget bestTarget = result.getBestTarget();

        double yaw = bestTarget.getYaw();
        double pitch = bestTarget.getPitch();

        inputs.latestTargetObservation =
            new TargetObservation(Rotation2d.fromDegrees(yaw), Rotation2d.fromDegrees(pitch));

        Transform3d bestFieldToCamera = result.getBestTarget().getBestCameraToTarget();
        // Transform3d bestFieldToRobot = bestFieldToCamera.plus(robotToCamera.inverse());

        Transform3d worstFieldToCamera = result.getBestTarget().getAlternateCameraToTarget();
        // Transform3d worstFieldToRobot = worstFieldToCamera.plus(robotToCamera.inverse());

        double bestDistance = bestFieldToCamera.getTranslation().getNorm();
        double worstDistance = worstFieldToCamera.getTranslation().getNorm();

        bestDistances.add(bestDistance);
        worstDistances.add(worstDistance);

        Rotation2d gyro = gyroRotationSupplier.get();

        Transform3d fieldToCameraTrig =
            new Transform3d(
                new Translation3d(
                    -bestDistance * Math.cos(Math.toRadians(yaw - gyro.getDegrees())),
                    -bestDistance * Math.sin(Math.toRadians(yaw - gyro.getDegrees())),
                    bestDistance * Math.sin(Math.toRadians(pitch))),
                new Rotation3d(gyro.plus(robotToCamera.getRotation().toRotation2d())));

        Transform3d fieldToRobot = fieldToCameraTrig.plus(robotToCamera.inverse());

        Pose3d robotPose =
            VisionConstants.aprilTagLayout
                .getTagPose(result.getBestTarget().fiducialId)
                .get()
                .plus(fieldToRobot);

        tagIds.add(result.getBestTarget().fiducialId);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                robotPose,
                result.getBestTarget().getPoseAmbiguity(),
                1,
                bestDistance,
                PoseObservationType.PHOTONVISION));
      } else {
        inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
      }
    }

    // Save pose observations to inputs object
    inputs.poseObservations = new PoseObservation[poseObservations.size()];
    for (int i = 0; i < poseObservations.size(); i++) {
      inputs.poseObservations[i] = poseObservations.get(i);
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }
}
