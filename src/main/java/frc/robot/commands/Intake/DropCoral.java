package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState.IntakeState;
import frc.robot.constants.intakeConstants;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class DropCoral extends SequentialCommandGroup {
  IntakeSubsystem intake;
  public DropCoral(IntakeSubsystem in) {

    intake = in;

    addCommands(Commands.runOnce(() -> intake.setIntakeSpeed(intakeConstants.CoralIntakeSpeed), intake),
                Commands.runOnce(() -> IntakeState.setIsEmpty(true), intake),
                new WaitCommand(2),
                Commands.runOnce(() -> intake.setIntakeSpeed(0), intake));
  }

}
