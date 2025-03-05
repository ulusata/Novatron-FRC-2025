package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class SetAlgae extends Command {

    private IntakeSubsystem intake;

  public SetAlgae(IntakeSubsystem in) {
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
  public void end(boolean interrupted) {
    System.out.println("end");
  }

  @Override
  public boolean isFinished() {
    System.out.println("sa");
    return intake.isAtBaseAngle();
  }
}
