package frc.robot.commands.Swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotState.SwerveState;
import frc.robot.subsystems.Swerve.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class SmoothDrive extends Command {
  private final SwerveSubsystem swerve;
    private final CommandXboxController driver;

    private double currentScale = 0.75;
    private final double targetScale;
    private final double scaleStep = 0.01;

  public SmoothDrive(SwerveSubsystem swerve, double targetScale, CommandXboxController driver) {

        this.swerve = swerve;
        this.targetScale = targetScale;
        this.driver = driver;
  }

  @Override
  public void initialize() {
    currentScale = 0.75;
  }

  @Override
  public void execute() {
      if (Math.abs(currentScale - targetScale) > scaleStep) {
        currentScale += (targetScale > currentScale) ? scaleStep : -scaleStep;
    } else {
        currentScale = targetScale;
    }

    // Build updated input stream with interpolated scaling
    SwerveInputStream stream = SwerveInputStream.of(swerve.getSwerveDrive(),
            () -> driver.getLeftY() * -1,
            () -> driver.getLeftX() * -1)
            .withControllerRotationAxis(driver::getRightX)
            .deadband(0.1)
            .scaleTranslation(currentScale)
            .scaleRotation(-0.8)
            .allianceRelativeControl(true);

    swerve.driveFieldOriented(stream).execute(); // Run it every tick
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
