package frc.robot.subsystems.piece_detection;

public class PieceDetectionIOReplay implements PieceDetectionIO {
  private final String name;

  public PieceDetectionIOReplay(String name) {
    this.name = name;
  }

  @Override
  public String getName() {
    return name;
  }
}
