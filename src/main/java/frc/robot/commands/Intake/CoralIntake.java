package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CoralIntake extends Command {
  private final IntakeSubsystem intake;

  public CoralIntake(IntakeSubsystem in) {
    this.intake = in;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeSpeed(-0.5);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return intake.getCoralSensor() == true && intake.getCoralSensor2() == true;
  }
}
