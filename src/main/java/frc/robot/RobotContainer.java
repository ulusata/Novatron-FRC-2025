// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.abstracts.BaseSubsystem;
import frc.robot.RobotState.IntakeState;
import frc.robot.commands.Elevator.GoToLevelCommand;
import frc.robot.commands.Intake.CoralAdjust;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.constants.elevatorConstant;
import frc.robot.constants.intakeConstants;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final CommandXboxController m_driverController = new CommandXboxController(
            0);
    private final CommandXboxController m_driverAsisstant = new CommandXboxController(
            1);

    private List<BaseSubsystem> m_allSubsystems = new ArrayList<>();

   private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
   private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
   // private final VisionSubsystem vision = VisionSubsystem.getInstance();
    private final IntakeSubsystem intake = IntakeSubsystem.getInstance();

    private Command fastDrive;
    private Command preciseDrive;
    
    
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {

       SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.4)
                .scaleRotation(-1)
                .allianceRelativeControl(false);

        fastDrive = drivebase.driveFieldOriented(driveAngularVelocity);

        SwerveInputStream driveAngularVelocityPrecise = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.07)
                .scaleRotation(-0.3)
                .allianceRelativeControl(false);

        preciseDrive = drivebase.driveFieldOriented(driveAngularVelocityPrecise);

        drivebase.setDefaultCommand(fastDrive); 
        
        configureSubsystems();
        configureBindings();

    }

    public void configureBindings(){

       m_driverController.leftBumper().whileTrue(preciseDrive);

        //For test purposes
     Command upElevator = new RunCommand(() -> elevator.goToLevel(elevator.getPosition() + 1), elevator);
    m_driverAsisstant.povUp().whileTrue(upElevator);

    Command downElevator = new RunCommand(() -> elevator.goToLevel(elevator.getPosition() + -1), elevator);
    m_driverAsisstant.povDown().whileTrue(downElevator);

        //Game Score Manipulation
        m_driverController.rightTrigger().onTrue(new CoralIntake(intake).andThen(new CoralAdjust(intake)));
        m_driverController.leftTrigger().onTrue(Commands.runOnce(() -> intake.setIntakeSpeed(intakeConstants.AlgeaIntakeSpeed)).andThen(Commands.runOnce(() -> IntakeState.setIsEmpty(true), intake)));

        m_driverController.rightBumper().onTrue(Commands.runOnce(() -> intake.setIntakeSpeed(0), intake));

        //Elevator Levels
       m_driverController.povLeft().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL1));
       m_driverController.povDown().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL2));
       m_driverController.povRight().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL3));
        m_driverController.povUp().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL4));

        //Intake
        Command pivotUp = new StartEndCommand(() -> intake.setIntakePivotVoltage(3), () -> intake.setIntakePivotVoltage(0), intake);
        Command pivotDown = new StartEndCommand(() -> intake.setIntakePivotVoltage(-5), () -> intake.setIntakePivotVoltage(0), intake);

        m_driverController.a().whileTrue(pivotUp);

        //Allignment
        // m_driverController.povRight().onTrue(new DeferredCommand(
         //                       () -> drivebase.driveToReefRight(), Set.of(drivebase)));
         //m_driverController.povLeft().onTrue(new DeferredCommand(
        //                        () -> drivebase.driveToReefLeft(), Set.of(drivebase)));

       //Odometry Reset
        m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometryAtStart()));

      // m_driverController.x().onTrue(Commands.runOnce(() -> this.drivebase.zeroGyro(), this.drivebase));


    }

    public void configureSubsystems(){
       m_allSubsystems.add(drivebase);
       m_allSubsystems.add(elevator);
        //m_allSubsystems.add(vision);
        m_allSubsystems.add(intake);
    }


    public Command getAutonomousCommand() {
        // System.out.println("Auto Command");
        // PathPlannerPath path;
        // try {
        //     path = PathPlannerPath.fromPathFile("straight5m");
        //     Pose2d startingPose = path.getStartingHolonomicPose().get();
        //     System.out.println(startingPose);
        //     drivebase.zeroGyroWithAlliance();
        //     drivebase.resetOdometry(startingPose);
        //     return AutoBuilder.followPath(path);
        // } catch (FileVersionException | IOException | ParseException e) {
        //     e.printStackTrace();
        // }
        return null;

    }

    public List<BaseSubsystem> getSubsystems() {
        return m_allSubsystems;
    }

}
