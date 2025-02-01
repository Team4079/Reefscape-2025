package frc.robot.subsystems;

import static frc.robot.utils.ExtensionsKt.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.PhotonVisionConstants.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import java.util.*;
import java.util.function.*;
import kotlin.*;
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
  public Supplier<List<Pair<PhotonModule, PhotonPipelineResult>>> resultPairs =
      () -> ExtensionsKt.getDecentResultPairs(cameras);

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
    Transform3d c1pos = createCameraPos(0.31, 0.0, CAMERA_ONE_HEIGHT_METER, CAMERA_ONE_ANGLE_DEG);
    Transform3d c2pos = createCameraPos(0.31, 0.0, CAMERA_TWO_HEIGHT_METER, CAMERA_TWO_ANGLE_DEG);
    cameras.add(new PhotonModule("Camera1", c1pos, fieldLayout));
    cameras.add(new PhotonModule("Camera2", c2pos, fieldLayout));
  }

  /**
   * This method is called periodically by the CommandScheduler. It updates the tracked targets,
   * selects the best camera based on pose ambiguity, and updates logged information.
   */
  @Override
  public void periodic() {
    resultPairs =
            () -> ExtensionsKt.getDecentResultPairs(cameras);
    List<Pair<PhotonModule, PhotonPipelineResult>> currentResultPair = resultPairs.get();

    logs(
        () -> {
          log("Does any camera exist", cameras.get(0) != null);
          log("Does any result pair exist", currentResultPair != null);
          log("Has tag", hasTag());
          if (currentResultPair != null) {
            log("Result pairs have targets", hasTargets(currentResultPair));
          }
        });

    if (currentResultPair != null) {
      logs("Best target list is empty", currentResultPair.isEmpty());

      if (!currentResultPair.isEmpty()) {
        PhotonTrackedTarget bestTarget = currentResultPair.get(0).getSecond().getBestTarget();
        yaw = bestTarget.getYaw();
        y = bestTarget.getBestCameraToTarget().getX();
        dist = bestTarget.getBestCameraToTarget().getZ();

        logs("Yaw", yaw);
        logStdDev();
      }
    }
  }

  /**
   * Checks if there is a visible tag.
   *
   * @return true if there is a visible tag and the current result pair is not null
   */
  public boolean hasTag() {
    List<Pair<PhotonModule, PhotonPipelineResult>> currentResultPair = resultPairs.get();

    logs(
            () -> {
              log("resultPairs get", resultPairs.get().isEmpty());
              log("currentResultPair not null", currentResultPair != null);
              log("hasTargets currentResultPair", hasTargets(currentResultPair));
            });

    return currentResultPair != null && hasTargets(currentResultPair);
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

  /**
   * Logs the standard deviation norm for each camera. This method filters out cameras with null
   * standard deviations and logs the normF value of the standard deviations for each camera.
   */
  public void logStdDev() {
    cameras.stream()
        .filter(camera -> camera.getCurrentStdDevs() != null)
        .forEach(
            camera ->
                logs(
                    "Camera %s Std Dev NormF".formatted(camera.getCameraName()),
                    camera.getCurrentStdDevs().normF()));
  }

  /**
   * Creates a new {@link Transform3d} object representing the position and orientation of a camera.
   *
   * @param x The X position of the camera in meters.
   * @param y The Y position of the camera in meters.
   * @param height The height of the camera in meters.
   * @param angleDeg The angle of the camera in degrees.
   * @return A {@link Transform3d} object representing the camera's position and orientation.
   */
  public static Transform3d createCameraPos(double x, double y, double height, double angleDeg) {
    return new Transform3d(
        new Translation3d(x, y, height),
        new Rotation3d(0.0, Math.toRadians(360 - angleDeg), Math.toRadians(180.0)));
  }
}
