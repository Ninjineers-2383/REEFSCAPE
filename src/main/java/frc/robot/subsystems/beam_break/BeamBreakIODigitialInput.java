package frc.robot.subsystems.beam_break;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.beam_break.BeamBreakConstants.BeamBreakConfig;

public class BeamBreakIODigitialInput {
  private final DigitalInput beamBreak;

  private final boolean invert;

  public BeamBreakIODigitialInput(BeamBreakConfig config) {
    beamBreak = new DigitalInput(config.id());
    invert = config.invert();
  }

  public boolean beamBreakTripped() {
    return beamBreak.get() ^ invert; // XOR
  }
}
