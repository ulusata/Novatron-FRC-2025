// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.abstracts.BaseSubsystem;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import frc.robot.subsystems.Vision.VisionSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {

    private final CommandPS5Controller m_driverController = new CommandPS5Controller(
            0);
    private final CommandXboxController m_driverAsisstant = new CommandXboxController(
            1);

    private List<BaseSubsystem> m_allSubsystems = new ArrayList<>();

    private final SwerveSubsystem drivebase = SwerveSubsystem.getInstance();
    private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
    private final VisionSubsystem vision = VisionSubsystem.getInstance();
    
    
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

        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
        
        configureSubsystems();
        configureBindings();

    }

    public void configureBindings(){
        m_driverController.square().onTrue(Commands.runOnce(() -> this.drivebase.zeroGyro(), this.drivebase));

       SwerveInputStream driveAngularVelocityPrecise = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.07)
                .scaleRotation(-0.3)
                .allianceRelativeControl(false);

        Command driveFieldOrientedAnglularVelocityPrecise = drivebase.driveFieldOriented(driveAngularVelocityPrecise);

        m_driverController.triangle().whileTrue(driveFieldOrientedAnglularVelocityPrecise);

        Command upElevator = new StartEndCommand(() -> elevator.setElevatorVoltage(2), () -> elevator.setElevatorVoltage(0), elevator);
       m_driverController.triangle().whileTrue(upElevator);

        Command downElevator = new StartEndCommand(() -> elevator.setElevatorVoltage(-2), () -> elevator.setElevatorVoltage(0), elevator);
        m_driverController.cross().whileTrue(downElevator);

       SwerveInputStream alignToAprilTag = SwerveInputStream.of(drivebase.getSwerveDrive(), 
        () -> vision.autoTranslateY(),
        () -> vision.autoTranslateX())
        .withControllerRotationAxis(() -> vision.autoRotate())
        .deadband(0.1)
        .scaleTranslation(0.3)
        .scaleRotation(-0.3)
        .allianceRelativeControl(false);

        Command alignToApriltagCommand = drivebase.drive(alignToAprilTag);
        m_driverController.cross().whileTrue(alignToApriltagCommand);

       /*  SwerveInputStream alingLeft = SwerveInputStream.of(drivebase.getSwerveDrive(), 
        () -> vision.autoTranslateY(),
        () -> 0)
        .withControllerRotationAxis(() -> 0)
        .deadband(0.1)
        .scaleTranslation(0.1)
        .scaleRotation(-0.3)
        .allianceRelativeControl(false); */

        // Command alignLeftCommand = drivebase.driveFieldOriented(alingLeft);
       // m_driverController.circle().whileTrue(alignLeftCommand);
    }

    public void configureSubsystems(){
        m_allSubsystems.add(drivebase);
        m_allSubsystems.add(elevator);
        m_allSubsystems.add(vision);
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
