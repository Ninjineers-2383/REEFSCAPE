package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveToPose extends Command {
  private final Drive drive;
  private final Supplier<Pose2d> endPoseSupplier;
  private TrapezoidProfile profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(500, 0.5));

  private Pose2d endPose;

  private TrapezoidProfile.State goalX;
  private TrapezoidProfile.State goalY;
  private TrapezoidProfile.State goalTheta;

  private TrapezoidProfile.State currentX;
  private TrapezoidProfile.State currentY;
  private TrapezoidProfile.State currentTheta;

  private PIDController xController = new PIDController(5, 0, 0);
  private PIDController yController = new PIDController(5, 0, 0);
  private PIDController thetaController = new PIDController(5, 0, 0);

  Timer timer = new Timer();

  public DriveToPose(Drive drive, Supplier<Pose2d> pose) {
    this.drive = drive;
    this.endPoseSupplier = pose;

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    endPose = endPoseSupplier.get();
    Pose2d currentPose = drive.getPose();
    ChassisSpeeds speeds = drive.getChassisSpeeds();
    speeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, currentPose.getRotation());

    goalX = new TrapezoidProfile.State(endPose.getTranslation().getX(), 0);
    goalY = new TrapezoidProfile.State(endPose.getTranslation().getY(), 0);
    goalTheta = new TrapezoidProfile.State(endPose.getRotation().getRadians(), 0);

    currentX =
        new TrapezoidProfile.State(currentPose.getTranslation().getX(), speeds.vxMetersPerSecond);
    currentY =
        new TrapezoidProfile.State(currentPose.getTranslation().getY(), speeds.vyMetersPerSecond);
    currentTheta =
        new TrapezoidProfile.State(
            currentPose.getRotation().getRadians(), speeds.omegaRadiansPerSecond);

    Logger.recordOutput("DriveToPose/Goal", endPose);
  }

  @Override
  public void execute() {
    boolean done = true;
    currentX = profile.calculate(0.02, currentX, goalX);
    done &= profile.isFinished(0.02);
    currentY = profile.calculate(0.02, currentY, goalY);
    done &= profile.isFinished(0.02);
    currentTheta = profile.calculate(0.02, currentTheta, goalTheta);
    done &= profile.isFinished(0.02);

    double xPower =
        xController.calculate(drive.getPose().getTranslation().getX(), currentX.position);
    double yPower =
        yController.calculate(drive.getPose().getTranslation().getY(), currentY.position);
    double thetaPower =
        thetaController.calculate(
            drive.getPose().getRotation().getRadians(), currentTheta.position);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xPower + currentX.velocity,
            yPower + currentY.velocity,
            thetaPower + currentTheta.velocity,
            drive.getRotation()));

    if (done) {
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return drive.getPose().getTranslation().getDistance(endPose.getTranslation()) < 0.04
        || timer.hasElapsed(0.5);
  }
}
