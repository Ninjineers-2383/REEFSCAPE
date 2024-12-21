package frc.robot.subsystems.beam_break;

import org.littletonrobotics.junction.AutoLog;

public interface BeamBreakIO {
  @AutoLog
  public class BeamBreakIOInputs {
    public boolean beamBreakTripped = false;
  }

  public default void updateInputs(BeamBreakIOInputs inputs) {}

  public default String getName() {
    return "Beam Break";
  }
}
