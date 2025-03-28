package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;

public class GoToLevelCommand extends Command {
  private final ElevatorSubsystem elevator;
  private final double target_level;

  public GoToLevelCommand(ElevatorSubsystem el, double trgt) {
    this.elevator = el;
    this.target_level = trgt;

    addRequirements(elevator);
  }

  @Override
  public void initialize() {
    elevator.goToLevel(target_level);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return elevator.isAtLevel(target_level);
  }
}
