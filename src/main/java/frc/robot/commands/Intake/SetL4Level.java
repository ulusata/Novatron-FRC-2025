package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SetL4Level extends Command {

    private IntakeSubsystem intake;

  public SetL4Level(IntakeSubsystem in) {
    intake = in;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setL4Level();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return intake.isAtL4Level();
  }
}
