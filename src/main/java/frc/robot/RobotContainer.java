package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import org.json.simple.parser.ParseException;
//import org.opencv.core.Mat;

import com.pathplanner.lib.auto.NamedCommands;

//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.abstracts.BaseSubsystem;
import frc.robot.RobotState.IntakeState;
import frc.robot.commands.Autos.DownL4;
import frc.robot.commands.Autos.DownOneL4;
import frc.robot.commands.Autos.L4Up;
import frc.robot.commands.Autos.Middle;
import frc.robot.commands.Autos.UpOneL4;
import frc.robot.commands.Elevator.GoToLevelCommand;
import frc.robot.commands.Intake.CoralAdjust;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.commands.Intake.DropCoral;
import frc.robot.constants.elevatorConstant;
import frc.robot.constants.intakeConstants;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
//import frc.robot.subsystems.Vision.VisionSubsystem;
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

    private SendableChooser<Command> m_commands = new SendableChooser<>();

    private Command fastDrive;
    private Command preciseDrive;
    private Command preciseDrive2;
    
    private double elevatorLevel = 0;
    
    SendableChooser<Command> m_chooser = new SendableChooser<>();

    public RobotContainer() {
        
       SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.75)//0.5
                .scaleRotation(-0.8)
                .allianceRelativeControl(true);

        fastDrive = drivebase.driveFieldOriented(driveAngularVelocity);

        SwerveInputStream driveAngularVelocityPrecise = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.095)//0.07
                .scaleRotation(-0.3)
                .allianceRelativeControl(true);

        preciseDrive = drivebase.driveFieldOriented(driveAngularVelocityPrecise);

        SwerveInputStream driveAngularVelocityPrecise2 = SwerveInputStream.of(drivebase.getSwerveDrive(),
                () -> m_driverController.getLeftY() * -1,
                () -> m_driverController.getLeftX() * -1)
                .withControllerRotationAxis(m_driverController::getRightX)
                .deadband(0.1)
                .scaleTranslation(0.38)//0.07
                .scaleRotation(-0.4)
                .allianceRelativeControl(true);

        preciseDrive2 = drivebase.driveFieldOriented(driveAngularVelocityPrecise2);

        drivebase.setDefaultCommand(fastDrive); 
        
        configureSubsystems();
        configureNamedCommands();
        configureAutonomous();
        configureBindings();

    }

    public void configureBindings(){

        //Precise Drive
        m_driverController.leftBumper().whileTrue(preciseDrive);
        m_driverController.rightBumper().whileTrue(preciseDrive2);

        //Game Score Manipulation
        m_driverController.leftTrigger().onTrue(
                        new CoralIntake(intake)
                        .andThen(new CoralAdjust(intake)));

        // m_driverController.rightTrigger().onTrue(
        //         Commands.runOnce(() -> intake.setIntakeSpeed(intakeConstants.CoralIntakeSpeed), intake)
        //         .andThen(Commands.runOnce(() -> IntakeState.setIsEmpty(true), intake))
        //         .andThen(new WaitCommand(2))
        //         .andThen(Commands.runOnce(() -> intake.setIntakeSpeed(0), intake)));
        
        m_driverController.rightTrigger().onTrue(new DropCoral(intake, elevator));

       // m_driverController.rightBumper().onTrue(Commands.runOnce(() -> intake.setIntakeSpeed(0), intake));

        //Algae Intake
       // m_driverController.y().onTrue(Commands.runOnce(() -> intake.setIntakeSpeed(intakeConstants.AlgeaIntakeSpeed), intake));


        // //Elevator Levels
        // m_driverAsisstant.povLeft().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kBase));

        // m_driverAsisstant.povDown().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL2));

        // m_driverAsisstant.povRight().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL3));

        // m_driverAsisstant.povUp().onTrue(new GoToLevelCommand(elevator,elevatorConstant.kElevatorL4));

        //Lazy Elevator Commands
        m_driverAsisstant.povLeft().onTrue(Commands.runOnce(() -> this.elevatorLevel = elevatorConstant.kBase));
        m_driverAsisstant.povDown().onTrue(Commands.runOnce(() -> this.elevatorLevel = elevatorConstant.kElevatorL2));
        m_driverAsisstant.povRight().onTrue(Commands.runOnce(() -> this.elevatorLevel = elevatorConstant.kElevatorL3));
        m_driverAsisstant.povUp().onTrue(Commands.runOnce(() -> this.elevatorLevel = elevatorConstant.kElevatorL4));

        m_driverController.x().onTrue(new GoToLevelCommand(elevator, this.elevatorLevel));


        //m_driverAsisstant.a().onTrue(new GoToLevelCommand(elevator, elevatorConstant.kElevatorAlgeaLeveL1));

        //m_driverAsisstant.y().onTrue(new GoToLevelCommand(elevator, elevatorConstant.kElevatorAlgeaLeveL2));


        //Manual elevator
        Command upElevator = new RunCommand(() -> elevator.goToLevel(elevator.getPosition() + 1.5), elevator);
        m_driverAsisstant.rightTrigger().whileTrue(upElevator);

        Command downElevator = new RunCommand(() -> elevator.goToLevel(elevator.getPosition() + -1.5), elevator);
        m_driverAsisstant.leftTrigger().whileTrue(downElevator);

        //For test purposes
        //Command pivotUp = new RunCommand(() -> intake.setPivotBaseLevel(), intake);
        //Command pivotDown = new RunCommand(() -> intake.setDropLevel() ,intake);

        //m_driverController.a().onTrue(pivotUp);
        //m_driverController.b().onTrue(pivotDown);


        // //Allignment
        // m_driverController.povRight().onTrue(new DeferredCommand(
        //                        () -> drivebase.driveToReefRight(), Set.of(drivebase)));
        //  m_driverController.povLeft().onTrue(new DeferredCommand(
        //                        () -> drivebase.driveToReefLeft(), Set.of(drivebase)));

       //Odometry Reset
        // m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometryAtStart()));

       m_driverController.a().onTrue(Commands.runOnce(() -> this.drivebase.zeroGyroWithAlliance(), this.drivebase));


    }

    public void configureSubsystems(){
        m_allSubsystems.add(drivebase);
        m_allSubsystems.add(elevator);
        //m_allSubsystems.add(vision);
        m_allSubsystems.add(intake);
    }

    public void configureNamedCommands(){
        // NamedCommands.registerCommand("GoToLevel1", 
        //         new SetZero(intake)
        //         .andThen(new GoToLevelCommand(elevator, elevatorConstant.kBase))
        //         .andThen(new SetLevels(intake)));
        // NamedCommands.registerCommand("GoToLevel2", 
        //         new SetZero(intake)
        //         .andThen(new GoToLevelCommand(elevator, elevatorConstant.kElevatorL2))
        //         .andThen(new SetBase(intake)));
        // NamedCommands.registerCommand("GoToLevel3", 
        //         new SetZero(intake)
        //         .andThen(new GoToLevelCommand(elevator, elevatorConstant.kElevatorL3))
        //         .andThen(new SetBase(intake)));
        // NamedCommands.registerCommand("GoToLevel4", 
        //         new SetZero(intake)
        //         .andThen(new GoToLevelCommand(elevator, elevatorConstant.kElevatorL4))
        //         .andThen(new SetBase(intake)));

        NamedCommands.registerCommand("Intake", 
                new CoralIntake(intake)
                .andThen(new CoralAdjust(intake)));

        NamedCommands.registerCommand("Drop", 
                Commands.runOnce(() -> intake.setIntakeSpeed(intakeConstants.CoralIntakeSpeed), intake)
                .andThen(Commands.runOnce(() -> IntakeState.setIsEmpty(true), intake))
                .andThen(new WaitCommand(2))
                .andThen(Commands.runOnce(() -> intake.setIntakeSpeed(0), intake)));
    }

    public void configureAutonomous(){
        try {
                m_commands.addOption("L4 Up", new L4Up(drivebase, elevator, intake));
                m_commands.addOption("L4 Down", new DownL4(drivebase, elevator, intake));


                m_commands.addOption("Down One L4", new DownOneL4(drivebase, elevator, intake));
                m_commands.addOption("Up One L4", new UpOneL4(drivebase, elevator, intake));
                m_commands.addOption("Middle", new Middle(drivebase, elevator, intake));


                SmartDashboard.putData("Autos", m_commands);

        } catch (ParseException | edu.wpi.first.util.struct.parser.ParseException e) {
                e.printStackTrace();
        }
    }


    public Command getAutonomousCommand()  {
        return m_commands.getSelected();
    }

    public List<BaseSubsystem> getSubsystems() {
        return m_allSubsystems;
    }

}
