package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedFloatArray;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/** IO implementation for real Limelight hardware. */
public class VisionIOQuestNav implements VisionIO {
  public record QuestNavData(
      Pose3d pose,
      double batteryPercent,
      double timestamp,
      float[] translation,
      float[] rotation) {}

  private enum QuestNavResetState {
    RESET_QUEUED,
    RESETTING,
    RESET_COMPLETE
  }

  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();
  private DoubleArrayPublisher resetPublisher = nt4Table.getDoubleArrayTopic("resetpose").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  private QuestNavResetState resetQueue = QuestNavResetState.RESET_COMPLETE;

  // Local heading helper variables
  private Translation3d resetTranslation = new Translation3d();
  private Rotation3d resetRotation = new Rotation3d();

  private final Transform3d robotToCamera;

  private final VisionIO absoluteVisionIO;
  private final VisionIOInputsAutoLogged absoluteInputs = new VisionIOInputsAutoLogged();

  public VisionIOQuestNav(Transform3d robotToCamera, VisionIO absoluteVisionIO) {
    // Initialize the camera to robot transform
    this.robotToCamera = robotToCamera;
    this.absoluteVisionIO = absoluteVisionIO;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    QuestNavData[] questNavData = getQuestNavData();

    // Update the absolute vision IO
    absoluteVisionIO.updateInputs(absoluteInputs);
    Logger.processInputs("QuestNav/absolute", absoluteInputs);

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.poseObservations = new PoseObservation[questNavData.length];
    for (int i = 0; i < questNavData.length; i++) {
      switch (resetQueue) {
        case RESET_QUEUED:
          // if (absoluteInputs.poseObservations.length > 0) {
          Transform2d resetTransform =
              // new Transform2d(
              //     absoluteInputs.poseObservations[0].pose().getX(),
              //     absoluteInputs.poseObservations[0].pose().getY(),
              //     new
              // Rotation2d(absoluteInputs.poseObservations[0].pose().getRotation().getZ()));
              new Transform2d(
                  13.890498 + Units.inchesToMeters(27 / 2.0 + 3.0),
                  4.0259 + Units.inchesToMeters(3),
                  new Rotation2d(Math.PI));
          resetTransform =
              resetTransform.plus(
                  new Transform2d(
                      robotToCamera.getX(),
                      robotToCamera.getY(),
                      new Rotation2d(robotToCamera.getRotation().getZ())));
          resetPublisher.set(
              new double[] {
                resetTransform.getX(),
                resetTransform.getY(),
                resetTransform.getRotation().getDegrees()
              });
          resetQueue = QuestNavResetState.RESETTING;
          questMosi.set(3);
          // }
          break;
        case RESETTING:
          if (questMiso.get() == 97) {
            questMosi.set(2);
          } else if (questMiso.get() == 98) {
            resetQueue = QuestNavResetState.RESET_COMPLETE;
            questMosi.set(0);
          }
          break;
        case RESET_COMPLETE:
          break;
      }
      inputs.poseObservations[i] =
          new PoseObservation(
              questNavData[i].timestamp(),
              questNavData[i].pose(),
              0.0,
              -1,
              0.0,
              PoseObservationType.QUESTNAV);
    }
    inputs.tagIds = new int[0];

    Logger.recordOutput("QuestNav/battery", getBatteryPercent());

    cleanUpQuestNavMessages();
  }

  private QuestNavData[] getQuestNavData() {
    TimestampedDouble[] timestamps = questTimestamp.readQueue();
    TimestampedFloatArray[] positions = questPosition.readQueue();
    TimestampedFloatArray[] angles = questAngles.readQueue();
    // TimestampedDouble[] battery = questBatteryPercent.readQueue();

    double battery = getBatteryPercent();

    int length = Math.min(timestamps.length, Math.min(positions.length, angles.length));

    QuestNavData[] data = new QuestNavData[length];

    for (int i = 0; i < length; i++) {
      data[i] =
          new QuestNavData(
              getPose(positions[i].value, angles[i].value).plus(robotToCamera.inverse()),
              battery,
              timestamps[i].timestamp,
              positions[i].value,
              angles[i].value);
    }

    return data;
  }

  // Gets the Quest's measured position.
  private Pose3d getPose(float[] position, float[] angles) {
    Pose3d pose = getQuestNavPose(position, angles);
    return new Pose3d(
        pose.getTranslation().minus(resetTranslation).rotateBy(resetRotation),
        pose.getRotation().minus(resetRotation));
  }

  // Gets the Rotation of the Quest.
  public Rotation3d getRotation(float[] angles) {
    return new Rotation3d(
        Units.degreesToRadians(-angles[2]),
        Units.degreesToRadians(angles[0]),
        Units.degreesToRadians(-angles[1]));
  }

  // Gets the battery percent of the Quest.
  private double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  private boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Clean up questnav subroutine messages after processing on the headset
  private void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  private Translation3d getQuestNavTranslation(float[] position) {
    return new Translation3d(position[2], -position[0], position[1]);
  }

  private Pose3d getQuestNavPose(float[] position, float[] angles) {
    var oculousPositionCompensated =
        getQuestNavTranslation(position).minus(new Translation3d(0, 0.1651, 0)); // 6.5
    return new Pose3d(oculousPositionCompensated, getRotation(angles));
  }

  public void resetPose() {
    questMosi.set(0);
    resetQueue = QuestNavResetState.RESET_QUEUED;
  }
}
