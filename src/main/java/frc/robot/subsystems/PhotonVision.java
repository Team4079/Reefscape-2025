package frc.robot.subsystems;

import static frc.robot.utils.Dash.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.*;
import java.util.*;
import org.photonvision.*;
import org.photonvision.targeting.*;

/**
 * The PhotonVision class is a subsystem that interfaces with multiple PhotonVision cameras to
 * provide vision tracking and pose estimation capabilities. This subsystem is a Singleton that
 * manages multiple CameraModules and selects the best result based on pose ambiguity.
 *
 * <p>This subsystem provides methods to get the estimated global pose of the robot, the distance to
 * targets, and the yaw of detected AprilTags. It also provides methods to check if a tag is visible
 * and get the pivot position based on distance calculations.
 */
public class PhotonVision extends SubsystemBase {
  private final List<PhotonModule> cameras = new ArrayList<>();
  private PhotonModule bestCamera;
  private PhotonPipelineResult currentResult;
  private PhotonTrackedTarget currentTarget;
  private double yaw = -15.0;
  private double y = 0.0;
  private double dist = 0.0;

  private double targetPoseAmbiguity = 7157.0;

  // Singleton instance
  private static final PhotonVision INSTANCE = new PhotonVision();

  /**
   * Returns the Singleton instance of this PhotonVision subsystem. This static method should be
   * used, rather than the constructor, to get the single instance of this class. For example:
   * {@code PhotonVision.getInstance();}
   *
   * @return The Singleton instance of PhotonVision
   */
  public static PhotonVision getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this PhotonVision subsystem. This constructor is private since this
   * class is a Singleton. Code should use the {@link #getInstance()} method to get the singleton
   * instance.
   */
  private PhotonVision() {
    // Initialize cameras with their positions
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // First camera setup
    Transform3d camera1Pos =
        new Transform3d(
            new Translation3d(0.31, 0.0, PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER),
            new Rotation3d(
                0.0,
                Math.toRadians(360 - PhotonVisionConstants.CAMERA_ONE_ANGLE_DEG),
                Math.toRadians(180.0)));
    cameras.add(new PhotonModule("Camera", camera1Pos, fieldLayout));

    // Add additional cameras here as needed
  }

  /**
   * This method is called periodically by the CommandScheduler. It updates the tracked targets,
   * selects the best camera based on pose ambiguity, and updates dashboard information.
   */
  @Override
  public void periodic() {
    updateBestCamera();
    if (bestCamera == null) return;

    List<PhotonPipelineResult> results = bestCamera.getAllUnreadResults();
    currentResult = results.isEmpty() ? null : results.get(0);

    if (currentResult == null) return;

    currentTarget = currentResult.getBestTarget();
    targetPoseAmbiguity = currentTarget != null ? currentTarget.getPoseAmbiguity() : 7157.0;

    for (PhotonTrackedTarget tag : currentResult.getTargets()) {
      yaw = tag.getYaw();
      y = tag.getBestCameraToTarget().getX();
      dist = tag.getBestCameraToTarget().getZ();
    }

    // Update dashboard
    log("yaw to target", yaw);
    log("cam ambiguity", targetPoseAmbiguity);
    log("_targets", currentResult.hasTargets());
  }

  /** Updates the best camera selection based on pose ambiguity of detected targets. */
  private void updateBestCamera() {
    bestCamera = getCameraWithLeastAmbiguity();
  }

  /**
   * Selects the camera with the least pose ambiguity from all available cameras.
   *
   * @return The CameraModule with the lowest pose ambiguity, or null if no cameras have valid
   *     targets
   */
  private PhotonModule getCameraWithLeastAmbiguity() {
    PhotonModule bestCam = null;
    double bestAmbiguity = Double.MAX_VALUE;

    for (PhotonModule camera : cameras) {
      List<PhotonPipelineResult> results = camera.getAllUnreadResults();
      for (PhotonPipelineResult result : results) {
        if (result.hasTargets()) {
          PhotonTrackedTarget target = result.getBestTarget();
          if (target != null && target.getPoseAmbiguity() < bestAmbiguity) {
            bestAmbiguity = target.getPoseAmbiguity();
            bestCam = camera;
          }
        }
      }
    }

    return bestCam;
  }

  /**
   * Checks if there is a visible AprilTag.
   *
   * <p>This method is useful to avoid NullPointerExceptions when trying to access specific info
   * based on vision.
   *
   * @return true if there is a visible tag, false otherwise
   */
  public boolean hasTag() {
    return currentResult != null && currentResult.hasTargets();
  }

  /**
   * Gets the estimated global pose of the robot using the best available camera.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot
   * @return The estimated robot pose, or null if no pose could be estimated
   */
  public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (bestCamera == null) return null;

    PhotonPoseEstimator estimator = bestCamera.getPoseEstimator();
    estimator.setReferencePose(prevEstimatedRobotPose);
    return currentResult != null ? estimator.update(currentResult).orElse(null) : null;
  }

  /**
   * Gets the estimated global pose of the robot as a Transform3d.
   *
   * @return The estimated global pose as a Transform3d
   */
  @SuppressWarnings("java:S3655")
  public Transform3d getEstimatedGlobalPose() {
    if (currentResult == null || currentResult.getMultiTagResult().isEmpty()) {
      return new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    }
    return currentResult.getMultiTagResult().get().estimatedPose.best;
  }

  /**
   * Calculates the straight-line distance to the currently tracked AprilTag.
   *
   * @return The distance to the AprilTag in meters
   */
  public double getDistanceAprilTag() {
    Transform3d pose = getEstimatedGlobalPose();
    return Math.sqrt(
        Math.pow(pose.getTranslation().getX(), 2) + Math.pow(pose.getTranslation().getY(), 2));
  }

  /**
   * Calculates the pivot position based on the distance to the AprilTag. Uses a polynomial function
   * tuned for optimal positioning.
   *
   * @return The calculated pivot position
   */
  public double getPivotPosition() {
    // 10/14/2024 outside tuning
    // Desmos: https://www.desmos.com/calculator/naalukjxze
    double r = getDistanceAprilTag() + 0.6;
    double f = -1.39223; // power 5
    double e = 20.9711; // power 4
    double d = -122.485; // power 3
    double c = 342.783; // power 2
    double b = -447.743; // power 1
    double a = 230.409; // constant

    return (f * Math.pow(r, 5.0))
        + (e * Math.pow(r, 4.0))
        + (d * Math.pow(r, 3.0))
        + (c * Math.pow(r, 2.0))
        + (b * r)
        + a;
  }

  /**
   * Gets the current yaw angle to the target.
   *
   * @return The yaw angle in degrees
   */
  public double getYaw() {
    return yaw;
  }

  public double getDist() {
    return dist;
  }

  public double getY() {
    return y;
  }

  /**
   * Gets the current target pose ambiguity.
   *
   * @return The target pose ambiguity value
   */
  public double getTargetPoseAmbiguity() {
    return targetPoseAmbiguity;
  }

  /**
   * Gets the current tracked target.
   *
   * @return The current PhotonTrackedTarget, or null if no target is tracked
   */
  public PhotonTrackedTarget getCurrentTarget() {
    return currentTarget;
  }
}
