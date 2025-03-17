package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ForkCommand extends Command {
  Command command;

  public ForkCommand(Command command) {
    this.command = command;
  }

  @Override
  public void initialize() {
    CommandScheduler.getInstance().schedule(command);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
