package frc.robot.subsystems.beam_break;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.beam_break.BeamBreakConstants.BeamBreakConfig;

public class BeamBreakIODigitialInput implements BeamBreakIO {
  private final String name;

  private final DigitalInput beamBreak;

  private final boolean invert;

  public BeamBreakIODigitialInput(String name, BeamBreakConfig config) {
    this.name = name;

    beamBreak = new DigitalInput(config.id());
    invert = config.invert();
  }

  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.beamBreakTripped = beamBreak.get() ^ invert; // XOR
  }

  @Override
  public String getName() {
    return name;
  }
}
