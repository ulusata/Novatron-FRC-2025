package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SetLevels extends Command {

    private IntakeSubsystem intake;

  public SetLevels(IntakeSubsystem in) {
    intake = in;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setDropLevel();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return intake.isAtDropAngle();
  }
}
