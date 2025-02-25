package frc.robot.subsystems;

import static frc.robot.utils.ExtensionsKt.*;
import static frc.robot.utils.RobotParameters.PhotonVisionConstants.*;
import static frc.robot.utils.pingu.LogPingu.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;
import kotlin.*;
import org.photonvision.PhotonCamera;
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
  private double yaw = 0.0;
  private double y = 0.0;
  private double dist = 0.0;
  private int logCount = 0;
  private List<Pair<PhotonModule, PhotonPipelineResult>> currentResultPair;

  // Singleton instance
  private static final PhotonVision INSTANCE;

  static {
    INSTANCE = new PhotonVision();
  }

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

  // x = 9.5
  // y = 12
  private PhotonVision() {
    cameras.add(
        new PhotonModule(
            "RightCamera",
            new Transform3d(
                new Translation3d(0.27305, -0.2985, CAMERA_ONE_HEIGHT_METER),
                new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(45))),
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)));
    // well calibrated camera is left camera
    cameras.add(
        new PhotonModule(
            "LeftCamera",
            new Transform3d(
                new Translation3d(0.27305, 0.2985, CAMERA_ONE_HEIGHT_METER),
                new Rotation3d(0.0, Math.toRadians(-25), Math.toRadians(-45))),
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded)));

    currentResultPair = new ArrayList<>();

    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  /**
   * This method is called periodically by the CommandScheduler. It updates the tracked targets,
   * selects the best camera based on pose ambiguity, and updates logged information.
   */
  @Override
  public void periodic() {
    currentResultPair = getDecentResultPairs(cameras);

    logs(
        () -> {
          log("Photonvision/Does any camera exist", cameras.get(0) != null);
          log("Photonvision/Does any result pair exist", currentResultPair != null);
          log("Photonvision/Has tag", hasTag());
          log("Photonvision/resultCamera List length", currentResultPair.size());
          if (currentResultPair != null) {
            log("Photonvision/Result pairs have targets", hasTargets(currentResultPair));
          }
        });

    if (currentResultPair != null) {
      logs("Photonvision/Best target list is empty", currentResultPair.isEmpty());

      if (!currentResultPair.isEmpty()) {
        logCount++;
        logs("Photonvision/BestTarget updated counter", logCount);
        PhotonTrackedTarget bestTarget = currentResultPair.get(0).getSecond().getBestTarget();
        logs("Photonvision/BestTarget is not null", bestTarget != null);

        logs("Photonvision/Best Target is not null", bestTarget != null);
        logs("Photonvision/Best Target is not null", bestTarget != null);

        if (bestTarget != null) {
          yaw = bestTarget.getYaw();
          y = bestTarget.getBestCameraToTarget().getX();
          dist = bestTarget.getBestCameraToTarget().getZ();
        }

        if (bestTarget != null) {
          yaw = bestTarget.getYaw();
          y = bestTarget.getBestCameraToTarget().getX();
          dist = bestTarget.getBestCameraToTarget().getZ();
        }

        logs("Yaw", yaw);
      }

      logStdDev();
    }
  }

  /**
   * Checks if there is a visible tag.
   *
   * @return true if there is a visible tag and the current result pair is not null
   */
  public boolean hasTag() {
    logs("Photonvision/currentResultPair not null", currentResultPair != null);

    if (currentResultPair != null) {
      logs("Photonvision/hasTargets currentResultPair", hasTargets(currentResultPair));
    }

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

  public PhotonCamera requestCamera(String cameraName) {
    for (PhotonModule camera : cameras) {
      if (camera.getCameraName().equals(cameraName)) {
        return camera.getCamera();
      }
    }
    return null;
  }

  public double fetchYaw(PhotonCamera camera) {
    for (Pair<PhotonModule, PhotonPipelineResult> pair : currentResultPair) {
      if (pair.getFirst().getCamera().equals(camera)) {
        return pair.getSecond().getBestTarget().getYaw();
      }
    }
    return 0.0;
  }

  public double fetchDist(PhotonCamera camera) {
    for (Pair<PhotonModule, PhotonPipelineResult> pair : currentResultPair) {
      if (pair.getFirst().getCamera().equals(camera)) {
        return pair.getSecond().getBestTarget().getBestCameraToTarget().getX();
      }
    }
    return 0.0;
  }

  public double fetchY(PhotonCamera camera) {
    for (Pair<PhotonModule, PhotonPipelineResult> pair : currentResultPair) {
      if (pair.getFirst().getCamera().equals(camera) && currentResultPair != null) {
        return pair.getSecond().getBestTarget().getBestCameraToTarget().getY();
      }
    }
    return 7157;
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
                    "Photonvision/Camera %s Std Dev NormF".formatted(camera.getCameraName()),
                    camera.getCurrentStdDevs().normF()));
  }

  public List<Pair<PhotonModule, PhotonPipelineResult>> getResultPairs() {
    return currentResultPair;
  }
}
