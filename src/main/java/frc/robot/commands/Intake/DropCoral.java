// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState.IntakeState;
import frc.robot.constants.elevatorConstant;
import frc.robot.constants.intakeConstants;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;

public class DropCoral extends SequentialCommandGroup {
  IntakeSubsystem intake;
  ElevatorSubsystem elevator;
  public DropCoral(IntakeSubsystem intake, ElevatorSubsystem elevator) {
    this.intake = intake;
    this.elevator = elevator;

    addCommands(Commands.runOnce(() -> intake.setIntakeSpeed(intakeConstants.lastSpeed), intake),
    Commands.runOnce(() -> IntakeState.setIsEmpty(true), intake),
    new WaitCommand(0.7),
    Commands.runOnce(() -> intake.setIntakeSpeed(0), intake));
  }

}
