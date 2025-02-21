package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.mechanical_advantage.LoggedTunableNumber;

public class QuarrelPresets {
  public QuarrelPresets() {}

  public static final LoggedTunableNumber L1_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/L1/Pivot", 0.0);
  ;
  public static final LoggedTunableNumber L1_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/L1/Elevator", 0.2);

  public static final LoggedTunableNumber L2_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/L2/Pivot", 90.0);
  public static final LoggedTunableNumber L2_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/L2/Elevator", 0.60);

  public static final LoggedTunableNumber L3_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/L3/Pivot", 90);
  public static final LoggedTunableNumber L3_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/L3/Elevator", 1);

  public static final LoggedTunableNumber L4_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/L4/Pivot", 280.0);
  public static final LoggedTunableNumber L4_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/L4/Elevator", 1.1);

  public static final LoggedTunableNumber ZERO_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/Zero/Pivot", 90.0);
  public static final LoggedTunableNumber ZERO_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/Zero/Elevator", 0.0);

  public static final LoggedTunableNumber LOWBALL_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/LOWBALL/Pivot", 70);
  public static final LoggedTunableNumber LOWBALL_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/LOWBALL/Elevator", 0.3);

  public static final LoggedTunableNumber HIGHBALL_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/HIGHBALL/Pivot", 70);
  public static final LoggedTunableNumber HIGHBALL_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/HIGHBALL/Elevator", 1);

  public static final LoggedTunableNumber SCORE_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/Score/Pivot", 30.0);

  public static final LoggedTunableNumber TRANSFER_DOWN_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/TransferDown/Pivot", 95);

  public static final LoggedTunableNumber TRANSFER_DOWN_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/TransferDown/Elevator", 0.14);

  public static final LoggedTunableNumber Processor_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/Processor/Pivot", 70);

  public static final LoggedTunableNumber Processor_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/Processor/Elevator", 0);

  public static final LoggedTunableNumber Barge_LOW_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/Barge/LOW/Pivot", 0);

  public static final LoggedTunableNumber Barge_LOW_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/Barge/LOW/Elevator", 1.3);

  public static final LoggedTunableNumber Barge_HIGH_PIVOT =
      new LoggedTunableNumber("QuarrelPresets/Barge/HIGH/Pivot", -80);

  public static final LoggedTunableNumber Barge_HIGH_ELEVATOR =
      new LoggedTunableNumber("QuarrelPresets/Barge/HIGH/Elevator", 1.3);

  public static QuarrelPosition getL1() {
    return new QuarrelPosition(L1_ELEVATOR.get(), Rotation2d.fromDegrees(L1_PIVOT.get()));
  }

  public static QuarrelPosition getL2() {
    return new QuarrelPosition(L2_ELEVATOR.get(), Rotation2d.fromDegrees(L2_PIVOT.get()));
  }

  public static QuarrelPosition getL3() {
    return new QuarrelPosition(L3_ELEVATOR.get(), Rotation2d.fromDegrees(L3_PIVOT.get()));
  }

  public static QuarrelPosition getL4() {
    return new QuarrelPosition(L4_ELEVATOR.get(), Rotation2d.fromDegrees(L4_PIVOT.get()));
  }

  public static QuarrelPosition getZero() {
    return new QuarrelPosition(ZERO_ELEVATOR.get(), Rotation2d.fromDegrees(ZERO_PIVOT.get()));
  }

  public static QuarrelPosition getHighball() {
    return new QuarrelPosition(
        HIGHBALL_ELEVATOR.get(), Rotation2d.fromDegrees(HIGHBALL_PIVOT.get()));
  }

  public static QuarrelPosition getLowball() {
    return new QuarrelPosition(LOWBALL_ELEVATOR.get(), Rotation2d.fromDegrees(LOWBALL_PIVOT.get()));
  }

  public static QuarrelPosition getTransferDown() {
    return new QuarrelPosition(
        TRANSFER_DOWN_ELEVATOR.get(), Rotation2d.fromDegrees(TRANSFER_DOWN_PIVOT.get()));
  }

  public static QuarrelPosition getProcessor() {
    return new QuarrelPosition(
        Processor_ELEVATOR.get(), Rotation2d.fromDegrees(Processor_PIVOT.get()));
  }

  public static QuarrelPosition getBargeLow() {
    return new QuarrelPosition(
        Barge_LOW_ELEVATOR.get(), Rotation2d.fromDegrees(Barge_LOW_PIVOT.get()));
  }

  public static QuarrelPosition getBargeHigh() {
    return new QuarrelPosition(
        Barge_HIGH_ELEVATOR.get(), Rotation2d.fromDegrees(Barge_HIGH_PIVOT.get()));
  }

  public static Rotation2d getScore() {
    return Rotation2d.fromDegrees(SCORE_PIVOT.get());
  }

  public static record QuarrelPosition(double elevatorPositionMeters, Rotation2d pivotRotation) {}
}
