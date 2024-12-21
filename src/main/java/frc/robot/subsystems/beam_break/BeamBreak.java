package frc.robot.subsystems.beam_break;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class BeamBreak extends SubsystemBase {
  private final BeamBreakIO beamBreak;
  private final BeamBreakIOInputsAutoLogged inputs = new BeamBreakIOInputsAutoLogged();

  private final String name;

  public BeamBreak(BeamBreakIO io) {
    beamBreak = io;

    name = beamBreak.getName();
  }

  @Override
  public void periodic() {
    beamBreak.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public boolean beamBreakTripped() {
    return inputs.beamBreakTripped;
  }

  public Trigger getTrigger() {
    return new Trigger(this::beamBreakTripped);
  }
}
