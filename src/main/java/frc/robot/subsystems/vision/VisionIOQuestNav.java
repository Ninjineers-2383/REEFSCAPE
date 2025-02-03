package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
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
  public record QuestNavData(Pose3d pose, double batteryPercent, double timestamp) {}

  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("batteryPercent").subscribe(0.0f);

  private boolean resetQueue = false;

  // Local heading helper variables
  private Transform3d resetTransform = new Transform3d();

  public VisionIOQuestNav() {}

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    QuestNavData[] questNavData = getQuestNavData();

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.poseObservations = new PoseObservation[questNavData.length];
    for (int i = 0; i < questNavData.length; i++) {
      if (resetQueue) {
        resetTransform = new Pose3d().minus(questNavData[i].pose());
        resetQueue = false;
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
              getPose(positions[i].value, angles[i].value), battery, timestamps[i].timestamp);
    }

    return data;
  }

  // Gets the Quest's measured position.
  private Pose3d getPose(float[] position, float[] angles) {
    return getQuestNavPose(position, angles).transformBy(resetTransform);
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
    resetQueue = true;
  }
}
