package frc.robot.subsystems.beam_break;

public class BeamBreakConstants {
  public record BeamBreakConfig(int id, boolean invert) {}

  public static final BeamBreakConfig OUTTAKE_BREAK_CONFIG = new BeamBreakConfig(0, false);
}
