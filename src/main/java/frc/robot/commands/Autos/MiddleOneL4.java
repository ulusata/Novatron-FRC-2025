package frc.robot.commands.Autos;

import java.io.IOException;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.GoToLevelCommand;
import frc.robot.commands.Intake.CoralAdjust;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.commands.Intake.DropCoral;
import frc.robot.commands.Intake.SetLevels;
import frc.robot.commands.Intake.SetZero;
import frc.robot.constants.elevatorConstant;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class MiddleOneL4 extends SequentialCommandGroup {
  SwerveSubsystem swerve;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;

  List<PathPlannerPath> pathGroup;

  public MiddleOneL4(SwerveSubsystem swerve,
    ElevatorSubsystem el,
    IntakeSubsystem in) throws org.json.simple.parser.ParseException, ParseException {
      this.swerve = swerve;
      elevator = el;
      intake = in;

      try{
        pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Middle");
      }

      catch (IOException e) {
        e.printStackTrace();
    }     

    addCommands(
      AutoBuilder.resetOdom(pathGroup.get(0).getStartingHolonomicPose().get()),
      new SequentialCommandGroup(
        AutoBuilder.followPath(pathGroup.get(0)),
        new GoToLevelCommand(elevator, elevatorConstant.kElevatorL4),
        new DropCoral(intake, elevator),
        new WaitCommand(0.2),
        new GoToLevelCommand(elevator, elevatorConstant.kElevatorL1)
      )
    );
      
  }
}
