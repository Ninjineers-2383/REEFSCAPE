package frc.robot.subsystems.drive.azimuth_motor;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorGains;
import frc.robot.subsystems.drive.azimuth_motor.AzimuthMotorConstants.AzimuthMotorHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class AzimuthMotorIOSim implements AzimuthMotorIO {
  private final String name;

  private final AzimuthMotorHardwareConfig config;

  private final DCMotor gearBox;

  private final DCMotorSim sim;

  private final PIDController controller;
  private final TunableSimpleMotorFeedforward feedforward;

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private double appliedVolts = 0.0;
  private double ffVolts = 0.0;

  private double positionSetpoint = 0;
  private double velocitySetpoint = 0;

  private boolean closedLoop = false;

  public AzimuthMotorIOSim(String name, AzimuthMotorHardwareConfig config) {
    this.name = name;

    this.config = config;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    motorsConnected = new boolean[config.canIds().length];
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];

    gearBox = DCMotor.getKrakenX60Foc(config.canIds().length);

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(gearBox, 0.01, config.gearRatio()), gearBox);

    controller = new PIDController(0, 0, 0);
    feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);

    controller.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(AzimuthMotorIOInputs inputs) {
    // Run closed-loop control
    if (closedLoop) {
      appliedVolts =
          controller.calculate(sim.getAngularPositionRotations(), positionSetpoint) + ffVolts;
    } else {
      controller.reset();
    }

    // Update simulation state
    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    // Update drive inputs
    inputs.velocityRotationsPerSecond = sim.getAngularVelocity().in(RotationsPerSecond);
    inputs.desiredVelocityRotationsPerSecond = velocitySetpoint;

    inputs.outputPositionRotations = sim.getAngularPositionRotations();
    inputs.desiredPositionRotations = positionSetpoint;

    for (int i = 0; i < config.canIds().length; i++) {
      motorsConnected[i] = true;
      motorPositions[i] = sim.getAngularPositionRotations();
      motorVelocities[i] = sim.getAngularVelocity().in(RotationsPerSecond);
      motorVoltages[i] = appliedVolts;
      motorCurrents[i] = sim.getCurrentDrawAmps();
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;

    // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't matter)
    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryTurnPositions =
        new Rotation2d[] {Rotation2d.fromRotations(inputs.outputPositionRotations)};
  }

  @Override
  public void setVoltage(double output) {
    closedLoop = false;
    appliedVolts = output;

    ffVolts = feedforward.calculateWithVelocities(velocitySetpoint, output);
  }

  @Override
  public void setPosition(double position, double velocity) {
    closedLoop = true;

    ffVolts = feedforward.calculateWithVelocities(velocitySetpoint, velocity);

    positionSetpoint = position;
    velocitySetpoint = velocity;
  }

  @Override
  public void setGains(AzimuthMotorGains gains) {
    controller.setPID(gains.kP(), gains.kI(), gains.kD());
    feedforward.setGains(gains.kS(), gains.kV(), gains.kA());
  }

  @Override
  public String getName() {
    return name;
  }
}
