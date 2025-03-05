package frc.robot.subsystems.Vision;

import java.io.File;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.abstracts.BaseSubsystem;
import frc.lib.enums.TelemetryVerbosityLevel;

public class VisionSubsystem extends BaseSubsystem {

    private static VisionSubsystem mInstance;

    public static VisionSubsystem getInstance() {

        if (mInstance == null) {
          mInstance = new VisionSubsystem();
        }
        return mInstance;
      }

    private String CAMERANAME = "10202-raspberry";
    private final AprilTagFieldLayout APRILTAGFIELDLAYOUT = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

    //Robot Mounting 3d
    private final Transform3d ROBOTTOCAM = 
        new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0, 0, 0)
        );

    
    private Matrix<N3, N1> singleTagStdDevs;

    private Matrix<N3, N1> multiTagStdDevs;

    public Matrix<N3, N1> curStdDevs;

    private final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    private final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);


    public Optional<EstimatedRobotPose> estimatedRobotPose = Optional.empty();

    // Mounting Constants
    private final double CAMERA_HEIGHT = 0.25; // will be changed.
    private final double REEF_TARGET = 0.22;
    private final double CAMERA_PITCH = 0;

    PhotonCamera camera = new PhotonCamera("10202-raspberry");

    private PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(APRILTAGFIELDLAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, ROBOTTOCAM);

    PhotonTrackedTarget lastTarget;

    private Supplier<Pose2d> currentPose;

    private Field2d field;

    //Default values for poseEstimator
    private double[] def1 = {0, 0, 0, 0, 0, 0};
    private double defVal = 0;

      // Networktable IO Inputs
    private NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(String.format("photonvision/%s", CAMERANAME));
    private DoubleArraySubscriber poseSub = cameraTable.getDoubleArrayTopic("targetPose").subscribe(def1);
    private DoubleSubscriber yaw = cameraTable.getDoubleTopic("targetYaw").subscribe(defVal);
    private DoubleSubscriber x = cameraTable.getDoubleTopic("targetPixelsX").subscribe(defVal);
    private DoubleSubscriber y = cameraTable.getDoubleTopic("targetArea").subscribe(defVal);
    private BooleanSubscriber hasTarget = cameraTable.getBooleanTopic("hasTarget").subscribe(false);
    private DoubleSubscriber pitch = cameraTable.getDoubleTopic("targetPitch").subscribe(defVal);

    // PID Controllers
    PIDController rotatePid = new PIDController(0.125, 0, 0);
    PIDController xPid = new PIDController(1, 0, 0.005);
    PIDController yPid = new PIDController(0.06, 0, 0.005);

    public VisionSubsystem(Supplier<Pose2d> poseSub){
        this.currentPose = poseSub;
        
        setTelemetryVerbosity(TelemetryVerbosityLevel.HIGH);
    }

    public VisionSubsystem(){
      setTelemetryVerbosity(TelemetryVerbosityLevel.HIGH);
    }

    @Override
    public void reset(){

    }

    @Override
    public void periodic(){
      getEstimatedGlobalPose();
    }

    @Override
    public void writePeriodicOutputs() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void simulationPeriodic() {
    }

    @Override
    protected void outputLowTelemetry() {

    }

    @Override
    protected void outputHighTelemetry() {
        SmartDashboard.putNumber("X Pixels", x.getAsDouble());
        SmartDashboard.putNumber("Y Pixels", y.getAsDouble());
    }

    public Optional<EstimatedRobotPose> updatePoseEstimator(PhotonPipelineResult result) {
        return poseEstimator.update(result);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
      Optional<EstimatedRobotPose> visionEst = Optional.empty();
      for (var change : camera.getAllUnreadResults()) {
          visionEst = poseEstimator.update(change);
          updateEstimationStdDevs(visionEst, change.getTargets());
      }
      return visionEst;
    }

    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = poseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }}
        }

      public Matrix<N3, N1> getEstimationStdDevs() {
          return curStdDevs;
      }

    public Pose2d getRobotPose2d(PhotonPipelineResult result){
      return poseEstimator.getReferencePose().toPose2d();
    }

    public void setPoseSupplier(Supplier<Pose2d> pose){
      this.currentPose = pose;
    }

    public Transform3d getCamToTag(PhotonPipelineResult result) {
        Transform3d camToTag = new Transform3d();
        if (result.hasTargets()) {
          camToTag = result.getBestTarget().getBestCameraToTarget();
        }
        return camToTag;
      }
    
      public double getXDistance() {
        double distance =
            (REEF_TARGET - CAMERA_HEIGHT)
                / Math.tan(Units.degreesToRadians(CAMERA_PITCH + pitch.get()));
        double x = distance * Units.degreesToRadians(yaw.get());
        System.out.println("x: " + x);
        return x;
      }
    
      public double autoRotate() {
        System.out.println("R: " + rotatePid.calculate(yaw.get(), 0) * -0.2);
        return rotatePid.calculate(yaw.get(), 0) * -0.2;
      }
    
      public double autoTranslateX() {
        System.out.println("X: " + -xPid.calculate(getXDistance(), 0));
        if (hasTarget.get()) {
          return xPid.calculate(getXDistance(), 0);
        } else {
          return 0;
        }
      }
    
      public double autoTranslateY() {
        System.out.println("Y: " + -yPid.calculate(y.get(), 10));
        if (hasTarget.get()) {
          return -yPid.calculate(y.get(), 5);
        } else {
          return 0;
        }
      }
    
      public boolean hasTarget() {
        return hasTarget.get();
      }
    
      public double getArea() {
        return y.get();
      }
    
      public double autoAlignToLeftReef() {
        if (hasTarget.get()) {
          return yPid.calculate(y.get(), 0) * 1.5;
        } else {
          return 0;
        }
      }
    
      public double autoAlignToRightReef() {
        if (hasTarget.get()) {
          return -yPid.calculate(y.get(), 0) * 1.5;
        } else {
          return 0;
        }
      }

}
