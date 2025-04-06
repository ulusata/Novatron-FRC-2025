package frc.robot.commands.Autos;

import java.io.IOException;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.util.struct.parser.ParseException;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Elevator.GoToLevelCommand;
import frc.robot.commands.Intake.CoralAdjust;
import frc.robot.commands.Intake.CoralIntake;
import frc.robot.commands.Intake.DropCoral;
import frc.robot.constants.elevatorConstant;
import frc.robot.subsystems.Elavator.ElevatorSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.Swerve.SwerveSubsystem;

public class L4Up extends SequentialCommandGroup {
  SwerveSubsystem swerve;
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;

  List<PathPlannerPath> pathGroup;

  public L4Up(SwerveSubsystem swerve,
    ElevatorSubsystem el,
    IntakeSubsystem in) throws org.json.simple.parser.ParseException, ParseException {
      this.swerve = swerve;
      elevator = el;
      intake = in;

      try{
        pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("Up");
      }

      catch (IOException e) {
        e.printStackTrace();
    }     

    addCommands(
      Commands.runOnce(() -> swerve.zeroGyroWithAlliance()),
      AutoBuilder.resetOdom(pathGroup.get(0).getStartingHolonomicPose().get()),
      new SequentialCommandGroup(
         new ParallelCommandGroup(
          AutoBuilder.followPath(pathGroup.get(0)),
          new CoralIntake(intake)
        ),
          new GoToLevelCommand(elevator, elevatorConstant.kElevatorL4),
          new DropCoral(intake, elevator),
          new WaitCommand(0.2),
          new GoToLevelCommand(elevator, elevatorConstant.kElevatorL1)
      ),
      new SequentialCommandGroup(
        AutoBuilder.followPath(pathGroup.get(1)),
        new CoralIntake(intake),
        new CoralAdjust(intake)
      ),
      new SequentialCommandGroup(
        AutoBuilder.followPath(pathGroup.get(2)),
        new GoToLevelCommand(elevator, elevatorConstant.kElevatorL4),
        new DropCoral(intake, elevator),
        new WaitCommand(0.2),
        new GoToLevelCommand(elevator, elevatorConstant.kElevatorL1)
      ),
      new SequentialCommandGroup(
        AutoBuilder.followPath(pathGroup.get(3))
      )
    );
      
  }
}
