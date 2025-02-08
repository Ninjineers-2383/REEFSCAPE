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
import edu.wpi.first.networktables.PubSubOption;
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
    RESET_POSE_QUEUED,
    RESET_HEADING_QUEUED,
    RESETTING,
    RESET_COMPLETE
  }

  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi =
      nt4Table
          .getIntegerTopic("mosi")
          .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));
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

  private final Transform3d robotToCamera;

  private final VisionIO absoluteVisionIO;
  private final VisionIOInputsAutoLogged absoluteInputs = new VisionIOInputsAutoLogged();

  private int delay = 0;

  private Transform3d[] questNavRawToFieldCoordinateSystemQueue = new Transform3d[5];
  private Transform3d questNavRawToFieldCoordinateSystem = new Transform3d();

  int count = 0;
  int idx = 0;

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

    delay++;

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.poseObservations = new PoseObservation[questNavData.length];

    if (absoluteInputs.poseObservations.length > 0 && questNavData.length > 0) {
      questNavRawToFieldCoordinateSystemQueue[idx] =
          new Transform3d(questNavData[0].pose, absoluteInputs.poseObservations[0].pose());
      questNavRawToFieldCoordinateSystemQueue[idx++] =
          new Transform3d(questNavRawToFieldCoordinateSystem.getTranslation(), new Rotation3d());
      count++;
      if (idx == 5) {
        idx = 0;
      }
      questNavRawToFieldCoordinateSystem = new Transform3d();
      for (int i = 0; i < Math.min(count, 5); i++) {
        questNavRawToFieldCoordinateSystem =
            questNavRawToFieldCoordinateSystem.plus(questNavRawToFieldCoordinateSystemQueue[i]);
      }
      questNavRawToFieldCoordinateSystem.div(Math.min(count, 5));
    }

    switch (resetQueue) {
      case RESET_POSE_QUEUED:
      case RESET_HEADING_QUEUED:
        if (delay > 2) {
          Transform2d resetTransform;
          if (resetQueue == QuestNavResetState.RESET_POSE_QUEUED
              && absoluteInputs.poseObservations.length > 0) {
            resetTransform =
                new Transform2d(
                    absoluteInputs.poseObservations[0].pose().getX(),
                    absoluteInputs.poseObservations[0].pose().getY(),
                    new Rotation2d(absoluteInputs.poseObservations[0].pose().getRotation().getZ()));
          } else if (resetQueue == QuestNavResetState.RESET_HEADING_QUEUED
              && questNavData.length > 0) {
            resetTransform =
                new Transform2d(
                    questNavData[0].pose().getTranslation().toTranslation2d(), Rotation2d.kPi);
          } else {
            break;
          }

          resetTransform =
              resetTransform.plus(
                  new Transform2d(
                      robotToCamera.getX(),
                      robotToCamera.getY(),
                      new Rotation2d(robotToCamera.getRotation().getZ())));

          if (resetTransform.getX() < 0 || resetTransform.getY() < 0) {
            resetTransform = new Transform2d(0, 0, resetTransform.getRotation());
          }
          resetPublisher.set(
              new double[] {
                resetTransform.getX(),
                resetTransform.getY(),
                resetTransform.getRotation().getDegrees()
              });
          resetQueue = QuestNavResetState.RESETTING;
        }
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
    for (int i = 0; i < questNavData.length; i++) {
      inputs.poseObservations[i] =
          new PoseObservation(
              questNavData[i].timestamp(),
              questNavData[i].pose().plus(questNavRawToFieldCoordinateSystem),
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
              getQuestNavPose(positions[i].value, angles[i].value).plus(robotToCamera.inverse()),
              battery,
              timestamps[i].timestamp,
              positions[i].value,
              angles[i].value);
    }

    return data;
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

  // Gets the Rotation of the Quest.
  public Rotation3d getQuestNavRotation(float[] angles) {
    return new Rotation3d(
        Units.degreesToRadians(-angles[2]),
        Units.degreesToRadians(angles[0]),
        Units.degreesToRadians(-angles[1]));
  }

  private Pose3d getQuestNavPose(float[] position, float[] angles) {
    var oculousPositionCompensated = getQuestNavTranslation(position); // 6.5
    return new Pose3d(oculousPositionCompensated, getQuestNavRotation(angles));
  }

  public void resetPose() {
    questMosi.accept(3);
    delay = 0;
    resetQueue = QuestNavResetState.RESET_POSE_QUEUED;
  }

  public void resetHeading() {
    questMosi.accept(3);
    delay = 0;
    resetQueue = QuestNavResetState.RESET_HEADING_QUEUED;
  }
}
