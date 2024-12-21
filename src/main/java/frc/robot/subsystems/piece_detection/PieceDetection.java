package frc.robot.subsystems.piece_detection;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class PieceDetection extends SubsystemBase {
  private PieceDetectionIO pieceDetection;
  private PieceDetectionIOInputsAutoLogged inputs = new PieceDetectionIOInputsAutoLogged();

  private final String name;

  public PieceDetection(PieceDetectionIO io) {
    pieceDetection = io;

    name = pieceDetection.getName();
  }

  @Override
  public void periodic() {
    pieceDetection.updateInputs(inputs);

    Logger.processInputs(name, inputs);
  }

  public double getPitch() {
    return inputs.pitch;
  }

  public double getYaw() {
    return inputs.yaw;
  }

  public double getArea() {
    return inputs.area;
  }

  public boolean pieceDetected() {
    return inputs.seesTarget;
  }
}
