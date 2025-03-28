// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Meter;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import frc.lib.abstracts.BaseSubsystem;
import frc.lib.utils.FieldUtils;
//import frc.robot.subsystems.Vision.VisionSubsystem;

import java.io.File;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends BaseSubsystem {
  private static SwerveSubsystem mInstance;

  //private VisionSubsystem vision;

  private Field2d field = new Field2d();

  public static SwerveSubsystem getInstance() {
    if (mInstance == null) {
      mInstance = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/talon"));
    }
    return mInstance;
  }

  /** Swerve drive object. */
  private final SwerveDrive swerveDrive;

  //Initial positions for both side
    private final Pose2d blueInitalPosition = new Pose2d(new Translation2d(Meter.of(7.380), Meter.of(4.112)),
      Rotation2d.fromDegrees(180));
    private static final Pose2d redInitalPosition = new Pose2d(new Translation2d(Meter.of(10.1), Meter.of(4.112)),
      Rotation2d.fromDegrees(0));

  /**
   * Initialize {@link SwerveDrive} with the directory provided.
   *
   * @param directory Directory of swerve drive config files.
   */
  public SwerveSubsystem(File directory) {
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      swerveDrive =
          new SwerveParser(directory).createSwerveDrive(5, new Pose2d());
    } catch (Exception e) {
      throw new RuntimeException(e);
    }
    swerveDrive.setHeadingCorrection(false);
    swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

    swerveDrive.stopOdometryThread();

    setupPathPlanner();

    //vision = VisionSubsystem.getInstance();
    //vision.setPoseSupplier(this::getPose);
  }

  public Pose2d getReefPoseLeft() {
    Pose2d nearestTagPose1 = this.getPose().nearest(FieldUtils.getReefAprilTags());
    double xOffset1 = (0.71 / 2) + 0.11;
    double yOffset1 = 0;
    yOffset1 += 0.165;

    Pose2d targetPose1 = nearestTagPose1.plus(new Transform2d(xOffset1, yOffset1, Rotation2d.k180deg));
    return targetPose1;
  }

  public Pose2d getReefPoseRight() {
    Pose2d nearestTagPose2 = this.getPose().nearest(FieldUtils.getReefAprilTags());
    double xOffset2 = (0.71 / 2) + 0.11;
    double yOffset2 = 0;
    yOffset2 -= 0.165;

    Pose2d targetPose2 = nearestTagPose2.plus(new Transform2d(xOffset2, yOffset2, Rotation2d.k180deg));
    return targetPose2;
  }

  @Override
  public void periodic() {
    // addVisionReading();
    swerveDrive.updateOdometry();
    //SmartDashboard.putString("Left Reef", this.getReefPoseLeft().toString());
    //SmartDashboard.putString("Right Reef", this.getReefPoseRight().toString());

    // SmartDashboard.putData("Field", field);

    // field.setRobotPose(swerveDrive.getPose());

    // SmartDashboard.putString("Left Reef", getReefPoseLeft().toString());

  }

  @Override
  public void simulationPeriodic() {}

  public void setupPathPlanner() {
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = false;
      AutoBuilder.configure(
          this::getPose,
          this::resetOdometry,
          this::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            if (enableFeedforward) {
              swerveDrive.drive(
                  speedsRobotRelative,
                  swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(speedsRobotRelative);
            }
          },
          new PPHolonomicDriveController(
              new PIDConstants(8.0, 0.001, 0.1), // Translation PID constants
              new PIDConstants(4.2, 0.0, 0.0) // Rotation PID constants
              ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return true;
          },
          this);

    } catch (Exception e) {
      e.printStackTrace();
    }
  }

  public PathPlannerPath getPathPlannerPath(String pathName) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
      return path;
    } catch (Exception e) {
      e.printStackTrace();
      return null;
    }
  }

  public Command getAutonomousCommand(String pathName) {
    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return new PathPlannerAuto(pathName);
  }

  public Command getPathFollowCommand(String pathName) {
    PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
      return AutoBuilder.followPath(path);
    } catch (Exception e) {
      e.printStackTrace();
      return Commands.run(
          () -> {
            System.out.println("Path not found: " + pathName);
          });
    }
  }
  /**
   * The primary method for controlling the drivebase. Takes a {@link Translation2d} and a rotation
   * rate, and calculates and commands module states accordingly. Can use either open-loop or
   * closed-loop velocity control for the wheel velocities. Also has field- and robot-relative
   * modes, which affect how the translation vector is used.
   *
   * @param translation {@link Translation2d} that is the commanded linear velocity of the robot, in
   *     meters per second. In robot-relative mode, positive x is torwards the bow (front) and
   *     positive y is torwards port (left). In field-relative mode, positive x is away from the
   *     alliance wall (field North) and positive y is torwards the left wall when looking through
   *     the driver station glass (field West).
   * @param rotation Robot angular rate, in radians per second. CCW positive. Unaffected by
   *     field/robot relativity.
   * @param fieldRelative Drive mode. True for field-relative, false for robot-relative.
   */
  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    swerveDrive.drive(
        translation,
        rotation,
        fieldRelative,
        isOpenLoop); // Open loop is disabled since it shouldn't be used most of the time.
  }

  

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {

    swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {

    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  /**
   * Drive according to the chassis robot oriented velocity.
   *
   * @param velocity Robot oriented {@link ChassisSpeeds}
   */
  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public Command drive(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
      swerveDrive.drive(velocity.get());
    });
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when
   * calling this method. However, if either gyro angle or module position is reset, this must be
   * called in order for odometry to keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {
    swerveDrive.resetOdometry(initialHolonomicPose);
  }

  /**
   * Gets the current pose (position and rotation) of the robot, as reported by odometry.
   *
   * @return The robot's pose
   */
  public Pose2d getPose() {
    return swerveDrive.getPose();
  }

  /**
   * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
   */
  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance() {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }

  /**
   * This will zero (calibrate) the robot to assume the current position is facing forward
   *
   * <p>If red alliance rotate the robot 180 after the drviebase zero command
   */
  public void zeroGyroWithAlliance() {
    if (isRedAlliance()) {
      zeroGyro();
      // Set the pose 180 degrees
      resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
    } else {
      zeroGyro();
    }
  }

  public void resetOdometryAtStart(){
    this.zeroGyroWithAlliance();
    if (isRedAlliance()) {
      resetOdometry(this.redInitalPosition);
    } else {
      resetOdometry(this.blueInitalPosition);
    }
  }

  /**
   * Gets the current yaw angle of the robot, as reported by the swerve pose estimator in the
   * underlying drivebase. Note, this is not the raw gyro reading, this may be corrected from calls
   * to resetOdometry().
   *
   * @return The yaw angle
   */
  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  /**
   * Get the chassis speeds based on controller input of 1 joystick and one angle. Control the robot
   * at an offset of 90deg.
   *
   * @param xInput X joystick input for the robot to move in the X direction.
   * @param yInput Y joystick input for the robot to move in the Y direction.
   * @param angle The angle in as a {@link Rotation2d}.
   * @return {@link ChassisSpeeds} which can be sent to the Swerve Drive.
   */
  public ChassisSpeeds getTargetSpeeds(double xInput, double yInput, Rotation2d angle) {
    Translation2d scaledInputs = SwerveMath.cubeTranslation(new Translation2d(xInput, yInput));

    return swerveDrive.swerveController.getTargetSpeeds(
        scaledInputs.getX(),
        scaledInputs.getY(),
        angle.getRadians(),
        getHeading().getRadians(),
        5);
  }

  /**
   * Gets the current field-relative velocity (x, y and omega) of the robot
   *
   * @return A ChassisSpeeds object of the current field-relative velocity
   */
  public ChassisSpeeds getFieldVelocity() {
    return swerveDrive.getFieldVelocity();
  }

  /**
   * Gets the current velocity (x, y and omega) of the robot
   *
   * @return A {@link ChassisSpeeds} object of the current velocity
   */
  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  /**
   * Get the {@link SwerveController} in the swerve drive.
   *
   * @return {@link SwerveController} from the {@link SwerveDrive}.
   */
  public SwerveController getSwerveController() {
    return swerveDrive.swerveController;
  }

  /** Add a fake vision reading for testing purposes. */
  public void addFakeVisionReading() {
    swerveDrive.addVisionMeasurement(
        new Pose2d(4, 7, Rotation2d.fromDegrees(0)), Timer.getFPGATimestamp());
  }

  // public void addVisionReading(){
  //   var visionEst = vision.getEstimatedGlobalPose();

  //       visionEst.ifPresent(
  //               est -> {
  //                   // Change our trust in the measurement based on the tags we can see
  //                   var estStdDevs = vision.getEstimationStdDevs();

  //                   swerveDrive.addVisionMeasurement(
  //                           est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
  //               });
  // }

  /**
   * Gets the swerve drive object.
   *
   * @return {@link SwerveDrive}
   */
  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }


  //Methods for allignment

  
  
  public Command driveToReefRight() {
    PathConstraints constraints = new PathConstraints(
        1, 1,
        Units.degreesToRadians(180), Units.degreesToRadians(180));
    Pose2d targetPose2 = getReefPoseLeft();

    return (Commands.runOnce(() -> {
      System.out.println("Drive to reef right is finished");
    })).andThen(generateCommand());
  }

  public Command driveToReefLeft() {
    PathConstraints constraints = new PathConstraints(
        1, 1,
        Units.degreesToRadians(180), Units.degreesToRadians(180));
    Pose2d targetPose2 = getReefPoseLeft();

    return (Commands.runOnce(() -> {
      System.out.println("Drive to reef left is finished");
    })).andThen(generateCommand());
  }

  public Command generateCommand() {
    return Commands.defer(() -> {
      return getPathFromWaypoint(getReefPoseLeft());
    }, Set.of());
  }

  private Command getPathFromWaypoint(Pose2d waypoint) {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(swerveDrive.getPose().getTranslation(), getPathVelocityHeading(swerveDrive.getFieldVelocity(), waypoint)),
        waypoint);
    
   
    if (waypoints.get(0).anchor().getDistance(waypoints.get(1).anchor()) < 0.01) {
      return Commands.none();
    }
    PathConstraints kPathConstraints = new PathConstraints(1.75, 1.75, 1/2 * Math.PI, 1 * Math.PI);
    PathPlannerPath path = new PathPlannerPath(
        waypoints,
        kPathConstraints,
        new IdealStartingState(getVelocityMagnitude(swerveDrive.getFieldVelocity()), swerveDrive.getOdometryHeading()),
        new GoalEndState(0.0, getReefPoseLeft().getRotation()));

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  private Rotation2d getPathVelocityHeading(ChassisSpeeds cs, Pose2d target) {
    if (getVelocityMagnitude(cs).in(MetersPerSecond) < 0.01) {
      var diff = swerveDrive.getPose().minus(target).getTranslation();
      return (diff.getNorm() < 0.01) ? target.getRotation() : diff.getAngle();
    }
    return new Rotation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond);
  }

  private LinearVelocity getVelocityMagnitude(ChassisSpeeds cs) {
    return MetersPerSecond.of(new Translation2d(cs.vxMetersPerSecond, cs.vyMetersPerSecond).getNorm());
  }


  @Override
  public void reset() {}

  @Override
  public void writePeriodicOutputs() {}

  @Override
  public void stop() {}

  @Override
  protected void outputLowTelemetry() {}

  @Override
  protected void outputHighTelemetry() {}
}
