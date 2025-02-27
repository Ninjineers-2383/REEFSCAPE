package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class REEFLocations {
  protected static REEFLocations instance = new REEFLocations();
  protected LoggedTunableNumber[] x = new LoggedTunableNumber[24];
  protected LoggedTunableNumber[] y = new LoggedTunableNumber[24];

  protected final LoggedTunableNumber GLOBAL_L_X =
      new LoggedTunableNumber("REEFLocations/GLOBAL/L/X", -0.39);
  protected final LoggedTunableNumber GLOBAL_L_Y =
      new LoggedTunableNumber("REEFLocations/GLOBAL/L/Y", 0.25);

  protected final LoggedTunableNumber GLOBAL_R_X =
      new LoggedTunableNumber("REEFLocations/GLOBAL/R/X", -0.42);
  protected final LoggedTunableNumber GLOBAL_R_Y =
      new LoggedTunableNumber("REEFLocations/GLOBAL/R/Y", -0.1);

  protected final AprilTagFieldLayout fieldLayout;

  protected REEFLocations() {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    for (int i = 0; i < 24; i++) {
      boolean blue = i < 12;
      x[i] =
          new LoggedTunableNumber("REEFLocations/" + (blue ? "BLUE" : "RED") + "/X/" + i % 12, 0.0);
      y[i] =
          new LoggedTunableNumber("REEFLocations/" + (blue ? "BLUE" : "RED") + "/Y/" + i % 12, 0.0);
    }
  }

  public static REEFLocations getInstance() {
    return instance;
  }

  protected Pose2d getAprilTagLocationByReefSide(int side, boolean isBlue) {

    int aprilTag;
    if (isBlue) {
      switch (side) {
        case 0:
          aprilTag = 18;
          break;
        case 1:
          aprilTag = 17;
          break;
        case 2:
          aprilTag = 22;
          break;
        case 3:
          aprilTag = 21;
          break;
        case 4:
          aprilTag = 20;
          break;
        case 5:
          aprilTag = 19;
          break;
        default:
          aprilTag = 0;
      }
    } else {
      switch (side) {
        case 0:
          aprilTag = 7;
          break;
        case 1:
          aprilTag = 8;
          break;
        case 2:
          aprilTag = 9;
          break;
        case 3:
          aprilTag = 10;
          break;
        case 4:
          aprilTag = 11;
          break;
        case 5:
          aprilTag = 6;
          break;
        default:
          aprilTag = 0;
      }
    }
    return fieldLayout.getTagPose(aprilTag).get().toPose2d();
  }

  protected boolean isBlue() {
    return DriverStation.getAlliance().isPresent()
        && DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
  }

  /*
   * Returns the pose of the reef at the given location.
   *
   * @param location The location of the branch [0, 11]
   */
  public Pose2d getBranchScorePose(int location) {
    boolean isBlue = isBlue();

    boolean isLeft = location % 2 == 0;

    Pose2d aprilTagPose = getAprilTagLocationByReefSide(location >> 1, isBlue);

    Transform2d globalTransform =
        isLeft
            ? new Transform2d(GLOBAL_L_X.get(), GLOBAL_L_Y.get(), Rotation2d.kZero)
            : new Transform2d(GLOBAL_R_X.get(), GLOBAL_R_Y.get(), Rotation2d.kZero);

    return aprilTagPose
        .transformBy(new Transform2d(0, 0, Rotation2d.kPi))
        .transformBy(globalTransform)
        .transformBy(
            new Transform2d(
                x[location + (isBlue ? 0 : 12)].get(),
                y[location + (isBlue ? 0 : 12)].get(),
                Rotation2d.kZero));
  }

  public Pose2d getAlgaePickupPose(int location) {
    boolean isBlue = isBlue();

    Pose2d aprilTagPose = getAprilTagLocationByReefSide(location, isBlue);

    Transform2d globalTransform = new Transform2d(GLOBAL_L_X.get(), 0.15, Rotation2d.kZero);

    return aprilTagPose
        .transformBy(new Transform2d(0, 0, Rotation2d.kPi))
        .transformBy(globalTransform);
  }
}
