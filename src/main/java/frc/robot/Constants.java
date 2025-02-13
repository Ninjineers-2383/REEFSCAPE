package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static boolean tuningMode = true;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class OperatorButtons {
    public static final class LEFT {
      public static final int REEF_BACK = 1;
      public static final int REEF_FRONT_RIGHT = 2;
      public static final int REEF_BACK_RIGHT = 3;
      public static final int REEF_FRONT = 4;
      public static final int REEF_FRONT_LEFT = 5;
      public static final int REEF_BACK_LEFT = 6;
      public static final int HUMAN_RIGHT = 7;
      public static final int HUMAN_LEFT = 8;
      public static final int EMERGENCY_ELEVATOR_RESET = 9;

      public static final int EMERGENCY_DRIVE = 11;
      public static final int EMERGENCY_ELEVATOR = 12;
    }

    public static final class RIGHT {
      public static final int LEFT_L1 = 11;
      public static final int LEFT_L2 = 8;
      public static final int LEFT_L3 = 5;
      public static final int LEFT_L4 = 2;
      public static final int RIGHT_L1 = 12;
      public static final int RIGHT_L2 = 9;
      public static final int RIGHT_L3 = 6;
      public static final int RIGHT_L4 = 3;
      public static final int ALGAE_HIGH = 4;
      public static final int ALGAE_LOW = 7;
      public static final int BARGE = 1;
      public static final int PROCESSOR = 10;
    }
  }
}
