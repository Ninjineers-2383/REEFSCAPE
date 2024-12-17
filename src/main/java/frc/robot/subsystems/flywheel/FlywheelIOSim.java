package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelHardwareConfig;

public class FlywheelIOSim implements FlywheelIO {
  private final String name;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;

  private double velocitySetpoint;

  public FlywheelIOSim(String name, FlywheelHardwareConfig config) {
    this.name = name;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    gearBox = DCMotor.getKrakenX60Foc(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.025, config.gearRatio()), gearBox);

    controller = new PIDController(0, 0, 0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    sim.setInputVoltage(controller.calculate(sim.getAngularVelocityRadPerSec(), velocitySetpoint));

    inputs.velocity = sim.getAngularVelocity().magnitude();
    inputs.desiredVelocity = velocitySetpoint;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;
  }

  @Override
  public void setGains(FlywheelGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());

    System.out.println(name + " gains set to " + gains);
  }
}
