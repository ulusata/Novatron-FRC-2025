package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState.IntakeState;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class CoralAdjust extends Command {
  private IntakeSubsystem intake;
  public CoralAdjust(IntakeSubsystem in) {
    intake = in;

    addRequirements(intake);
  }

  @Override
  public void initialize() {
    intake.setIntakeSpeed(-0.3);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    intake.setIntakeSpeed(0);
    IntakeState.setIsEmpty(false);
  }

  @Override
  public boolean isFinished() {
    return intake.getCoralSensor() == false;
  }
}
