package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Pose3d;
import java.util.function.Supplier;

public class PieceDetectionIOSim implements PieceDetectionIO {
  Supplier<Pose3d[]> notePoses;
  Supplier<Pose3d> drivePose;
  String name;

  private double smallestDistance = Integer.MAX_VALUE;

  public PieceDetectionIOSim(
      String name, Supplier<Pose3d[]> notePoses, Supplier<Pose3d> drivePose) {
    this.notePoses = notePoses;
    this.drivePose = drivePose;
    this.name = name;
  }

  @Override
  public void updateInputs(PieceDetectionIOInputs inputs) {
    inputs.connected = true;

    for (int i = 0; i < notePoses.get().length; i++) {
      if (notePoses.get()[i].getTranslation().getDistance(drivePose.get().getTranslation())
          < smallestDistance) {
        smallestDistance =
            notePoses.get()[i].getTranslation().getDistance(drivePose.get().getTranslation());
      }
    }

    // inputs.frontSeesTarget = true;
    // inputs.frontYaw = frontResult.getBestTarget().getYaw();
    // inputs.frontPitch = frontResult.getBestTarget().getPitch();
  }

  @Override
  public String getName() {
    return name;
  }
}
