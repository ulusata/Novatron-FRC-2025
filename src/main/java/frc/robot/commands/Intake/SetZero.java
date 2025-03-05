package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SetZero extends Command {
  private final IntakeSubsystem intake;
  public SetZero(IntakeSubsystem in) {
    intake = in;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setPivotBaseLevel();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return intake.isAtBaseAngle();
  }
}
