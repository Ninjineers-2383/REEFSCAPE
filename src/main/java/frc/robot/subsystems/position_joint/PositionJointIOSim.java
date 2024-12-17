package frc.robot.subsystems.position_joint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;

public class PositionJointIOSim implements PositionJointIO {
  private final String name;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;

  private double positionSetpoint;

  public PositionJointIOSim(String name, PositionJointHardwareConfig config) {
    this.name = name;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    gearBox = DCMotor.getKrakenX60Foc(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.025, config.gearRatio()), gearBox);

    controller = new PIDController(0, 0, 0);
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    sim.setInputVoltage(controller.calculate(sim.getAngularPositionRad(), positionSetpoint));

    inputs.position = sim.getAngularVelocity().magnitude();
    inputs.desiredPosition = positionSetpoint;
  }

  @Override
  public void setPosition(double position, double velocity) {
    positionSetpoint = position;
  }

  @Override
  public void setGains(PositionJointGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());

    System.out.println(name + " gains set to " + gains);
  }
}
