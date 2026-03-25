package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveIntake extends Command {
  public double pos;

  public MoveIntake(double position) {
    pos = position;
  }

  @Override
  public void initialize() {
    addRequirements(RobotContainer.intake);
    RobotContainer.intake.setIntakePosition(pos);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {}
}
