package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.SwerveState;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class SmoothDrive extends Command {
  SwerveSubsystem swerve;
  double targetLevel;
  double start = 0.75;
  SwerveInputStream stream;
  CommandXboxController driver;
  Command smoothCommand;
  Timer timer = new Timer();
  public SmoothDrive(SwerveSubsystem sw, double targetLevel, CommandXboxController driver1) {

    stream = SwerveInputStream.of(swerve.getSwerveDrive(),
    () -> driver.getLeftY() * -1,
    () -> driver.getLeftX() * -1)
    .withControllerRotationAxis(driver::getRightX)
    .deadband(0.1)
    .scaleTranslation(0.75)//0.5
    .scaleRotation(-0.8)
    .allianceRelativeControl(true);

    smoothCommand = swerve.driveFieldOriented(stream);
    SwerveState.setCommand(smoothCommand);

    start = 0.75;

    swerve = sw;
    driver = driver1;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    goToTheLevel();
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }

  public Command goToTheLevel(){
    for(double i = start; i <= targetLevel; i--){
      stream = SwerveInputStream.of(swerve.getSwerveDrive(),
      () -> driver.getLeftY() * -1,
      () -> driver.getLeftX() * -1)
      .withControllerRotationAxis(driver::getRightX)
      .deadband(0.1)
      .scaleTranslation(start)//0.5
      .scaleRotation(-0.8)
      .allianceRelativeControl(true);

      smoothCommand = swerve.driveFieldOriented(stream);
      SwerveState.setCommand(smoothCommand);

      Commands.waitSeconds(0.02);
      start--;
    }

    return swerve.driveFieldOriented(stream);
  }

}
