package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;

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

    for (var result : camera.getAllUnreadResults()) {
      // Update latest target observation
      if (result.hasTargets()) {
        inputs.latestTargetObservation =
            new TargetObservation(
                Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                Rotation2d.fromDegrees(result.getBestTarget().getPitch()));

        Transform3d cameraToTarget = result.getBestTarget().getBestCameraToTarget();
        // Transform3d bestFieldToRobot = bestFieldToCamera.plus(robotToCamera.inverse());

        double distance = cameraToTarget.getTranslation().getNorm();

        double tx = -Units.degreesToRadians(result.getBestTarget().getYaw());
        double ty = Units.degreesToRadians(result.getBestTarget().getPitch());

        Rotation2d gyro = gyroRotationSupplier.get();

        double distance2d = distance * Math.cos(-robotToCamera.getRotation().getY() - ty);
        Rotation2d camToTagRotation =
            gyro.plus(robotToCamera.getRotation().toRotation2d().plus(Rotation2d.fromRadians(-tx)));

        Pose2d tagPose2d =
            VisionConstants.aprilTagLayout
                .getTagPose(result.getBestTarget().fiducialId)
                .get()
                .toPose2d();

        Translation2d fieldToCamTranslation =
            new Pose2d(tagPose2d.getTranslation(), camToTagRotation.plus(Rotation2d.kPi))
                .transformBy(new Transform2d(distance2d, 0.0, Rotation2d.kZero))
                .getTranslation();

        Pose2d robotPose2d =
            new Pose2d(fieldToCamTranslation, gyro.plus(robotToCamera.getRotation().toRotation2d()))
                .transformBy(
                    new Transform2d(
                        new Pose2d(
                            robotToCamera.getTranslation().toTranslation2d(),
                            robotToCamera.getRotation().toRotation2d()),
                        Pose2d.kZero));
        robotPose2d = new Pose2d(robotPose2d.getTranslation(), gyro);

        poseObservations.add(
            new PoseObservation(
                result.getTimestampSeconds(),
                new Pose3d(robotPose2d),
                result.getBestTarget().getPoseAmbiguity(),
                1,
                distance,
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
