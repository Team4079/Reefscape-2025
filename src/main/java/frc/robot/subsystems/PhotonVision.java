package frc.robot.subsystems;

import static frc.robot.utils.PhotonModuleListKt.*;
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
  private final Supplier<List<Pair<PhotonModule, PhotonPipelineResult>>> results =
      () -> PhotonModuleListKt.getDecentResultPairs(cameras);

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
    logs("decent result pairs exist", results.get() != null);

    if (results.get() != null) {
      log("best target list is empty", results.get().isEmpty());

      // REMEMBER: MOVEMENT IS BOUND TO A! DON'T FORGET NERD
      if (!results.get().isEmpty()) {
        PhotonTrackedTarget bestTarget = results.get().getFirst().getSecond().getBestTarget();
        yaw = bestTarget.getYaw();
        y = bestTarget.getBestCameraToTarget().getX();
        dist = bestTarget.getBestCameraToTarget().getZ();

        logs(log("yaw to target", yaw), log("_targets", hasTargets(results.get())));
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
    return results.get() != null && hasTargets(results.get());
  }

  /**
   * Gets the estimated global pose of the robot using the best available camera.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot
   * @return The estimated robot pose, or null if no pose could be estimated
   */
  public List<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    if (results.get() == null) return Collections.emptyList();

    List<EstimatedRobotPose> poses = new ArrayList<>();

    for (Pair<PhotonModule, PhotonPipelineResult> pair : results.get()) {
      PhotonPoseEstimator estimator = pair.getFirst().getPoseEstimator();
      estimator.setReferencePose(prevEstimatedRobotPose);
      estimator.update(pair.getSecond()).ifPresent(poses::add);
    }

    return poses;
  }

  /**
   * Gets the current yaw angle to the target.
   *
   * @return The yaw angle in degrees
   */
  public double getYaw() {
    return yaw;
  }

  /**
   * Gets the current distance to the target.
   *
   * @return The distance in meters
   */
  public double getDist() {
    return dist;
  }

  /**
   * Gets the current Y position of the target.
   *
   * @return The Y position in meters
   */
  public double getY() {
    return y;
  }
}
