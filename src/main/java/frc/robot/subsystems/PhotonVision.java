package frc.robot.subsystems;

import static frc.robot.utils.Register.Dash.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.*;
import java.util.*;
import java.util.function.*;
import kotlin.*;
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
  private double yaw = -15.0;
  private double y = 0.0;
  private double dist = 0.0;
  private Supplier<Pair<PhotonModule, PhotonPipelineResult>> bestResultPair =
      () -> PhotonModuleListKt.getBestResultPair(cameras);

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
    AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

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

    logs(
        log("does camera exist", cameras.get(0) != null),
        log("does best camera exist", bestResultPair.get() != null));

    if (bestResultPair.get() != null) {
      log("has current target", bestResultPair.get().getSecond() != null);

      // REMEMBER: MOVEMENT IS BOUND TO A! DON'T FORGET NERD
      if (bestResultPair.get().getSecond() != null) {
        for (PhotonTrackedTarget tag : bestResultPair.get().getSecond().getTargets()) {
          yaw = tag.getYaw();
          y = tag.getBestCameraToTarget().getX();
          dist = tag.getBestCameraToTarget().getZ();
        }

        logs(
            log("yaw to target", yaw),
            log("_targets", bestResultPair.get().getSecond().hasTargets()));
      }
    }
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
    return bestResultPair.get().getSecond() != null
        && bestResultPair.get().getSecond().hasTargets();
  }

  /**
   * Gets the estimated global pose of the robot using the best available camera.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot
   * @return The estimated robot pose, or null if no pose could be estimated
   */
  public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (bestResultPair.get() == null) return null;

    PhotonPoseEstimator estimator = bestResultPair.get().getFirst().getPoseEstimator();
    estimator.setReferencePose(prevEstimatedRobotPose);
    return bestResultPair.get().getSecond() != null
        ? estimator.update(bestResultPair.get().getSecond()).orElse(null)
        : null;
  }

  /**
   * Gets the estimated global pose of the robot as a Transform3d.
   *
   * @return The estimated global pose as a Transform3d
   */
  @SuppressWarnings("java:S3655")
  public Transform3d getEstimatedGlobalPose() {
    if (bestResultPair.get().getSecond() == null
        || bestResultPair.get().getSecond().getMultiTagResult().isEmpty()) {
      return new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    }
    return bestResultPair.get().getSecond().getMultiTagResult().get().estimatedPose.best;
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
   * Gets the current tracked target.
   *
   * @return The current PhotonTrackedTarget, or null if no target is tracked
   */
  public PhotonTrackedTarget getCurrentTarget() {
    return bestResultPair.get().getSecond().getBestTarget();
  }

  /**
   * Logs the current standard deviations for each camera to the console.
   */
  public void logStdDev() {
    for (PhotonModule camera : cameras) {
      camera.getEstimatedRobotPose();
      logs(
          log(String.format("Camera [%s]", camera.getCameraName()), camera.getCurrentStdDevs()));
    }
  }
}
