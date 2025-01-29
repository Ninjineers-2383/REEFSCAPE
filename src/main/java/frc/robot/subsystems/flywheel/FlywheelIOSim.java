package frc.robot.subsystems.flywheel;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

public class FlywheelIOSim implements FlywheelIO {
  private final String name;

  private final FlywheelHardwareConfig config;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;
  private final TunableSimpleMotorFeedforward feedforward;

  private final double[] motorPositions;
  private final double[] motorVelocities;
  private final double[] motorAccelerations;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double velocitySetpoint = 0;

  public FlywheelIOSim(String name, FlywheelHardwareConfig config) {
    this.name = name;

    this.config = config;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorAccelerations = new double[config.canIds().length];

    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];

    gearBox = DCMotor.getKrakenX60Foc(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.025, config.gearRatio()), gearBox);

    controller = new PIDController(0, 0, 0);
    feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    double inputVoltage =
        controller.calculate(sim.getAngularVelocityRPM(), velocitySetpoint)
            + feedforward.calculateWithVelocities(sim.getAngularVelocityRPM(), velocitySetpoint);
    sim.setInputVoltage(inputVoltage);
    sim.update(0.02);

    inputs.velocity = sim.getAngularVelocityRPM();
    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < config.canIds().length; i++) {
      motorPositions[i] = sim.getAngularPositionRotations();
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);
      motorAccelerations[i] = sim.getAngularAcceleration().in(RotationsPerSecondPerSecond);

      motorVoltages[i] = inputVoltage;
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;
    inputs.motorAccelerations = motorAccelerations;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;
  }

  @Override
  public void setGains(FlywheelGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kV(), gains.kA());

    System.out.println(name + " gains set to " + gains);
  }

  @Override
  public String getName() {
    return name;
  }
}
