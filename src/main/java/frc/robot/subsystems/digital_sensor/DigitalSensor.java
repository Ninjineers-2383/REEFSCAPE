package frc.robot.subsystems.digital_sensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class DigitalSensor extends SubsystemBase {
  private final DigitalSensorIO digitalSensor;
  private final DigitalSensorIOInputsAutoLogged inputs = new DigitalSensorIOInputsAutoLogged();

  private final String name;

  public DigitalSensor(DigitalSensorIO io) {
    digitalSensor = io;

    name = digitalSensor.getName();
  }

  @Override
  public void periodic() {
    digitalSensor.updateInputs(inputs);
    Logger.processInputs(name, inputs);
  }

  public boolean sensorActive() {
    return inputs.sensorActive;
  }

  public Trigger getTrigger() {
    return new Trigger(this::sensorActive);
  }
}
