package frc.robot.subsystems;

import static frc.robot.utils.ExtensionsKt.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.PhotonVisionConstants.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
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
  private double yaw = 0.0;
  private double y = 0.0;
  private double dist = 0.0;
  private int logCount = 0;
  public Supplier<List<Pair<PhotonModule, PhotonPipelineResult>>> resultPairs =
      () -> getDecentResultPairs(cameras);
  public List<PhotonPipelineResult> resultCamera;
  private List<Pair<PhotonModule, PhotonPipelineResult>> currentResultPair;
  private Timer timer;

  // Singleton instance
  private static final PhotonVision INSTANCE;

  static {
    try {
      INSTANCE = new PhotonVision();
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
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
  private PhotonVision() throws IOException {
    //    AprilTagFieldLayout fieldLayout =
    // AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    //      ^ FUCK THIS LINE OF CODE IT BREAKS THE MEMORY ERRORS AND EVEYRTHING IT SUCKS
    //
    //    _____                                    _                    _
    //   |_   _|                                  | |                  | |
    //     | |    _ __ ___    _ __    ___   _ __  | |_   __ _  _ ___   | |_
    //     | |   | '_ ` _ \  | '_ \  / _ \ | ___| | __/ / _` | | '_  \ | __|
    //     |_|   |_| | | | | | | |_) | (_) | |    | |  |  (_|  | | | | | |_
    //   |_____| |_| |_| |_| | .__/  \___/ |_|     \__  \__,_| |_| |_| \___|
    //                       | |
    //                       |_|

    //  AprilTagFieldLayout fieldLayout =

    // First camera setup
    Transform3d c1pos = createCameraPos(0.31, 0.0, CAMERA_ONE_HEIGHT_METER, CAMERA_ONE_ANGLE_DEG);
    //    Transform3d c2pos = createCameraPos(0.31, 0.0, CAMERA_TWO_HEIGHT_METER,
    // CAMERA_TWO_ANGLE_DEG);
    cameras.add(
        new PhotonModule(
            "RightCamera", c1pos, AprilTagFieldLayout.loadFromResource("2025-reefscape.json")));
    //    cameras.add(new PhotonModule("Camera2", c2pos,
    // AprilTagFieldLayout.loadFromResource("2025-reefscape.json")));

    timer = new Timer();
    timer.start();

    currentResultPair = new ArrayList<>();

    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  /**
   * This method is called periodically by the CommandScheduler. It updates the tracked targets,
   * selects the best camera based on pose ambiguity, and updates logged information.
   */
  @Override
  public void periodic() {
    if (timer.advanceIfElapsed(0.02)) currentResultPair = resultPairs.get();
    //    currentResultPair = resultPairs.get();

    logs(
        () -> {
          log("/Photonvision/Does any camera exist", cameras.get(0) != null);
          log("/Photonvision/Does any result pair exist", currentResultPair != null);
          log("/Photonvision/Has tag", hasTag());
          log("/Photonvision/resultCamera List length", currentResultPair.size());
          if (currentResultPair != null) {
            log("/Photonvision/Result pairs have targets", hasTargets(currentResultPair));
          }
        });

    logs("/Photonvision/is current result pair null", currentResultPair != null);

    if (currentResultPair != null) {
      logs("/Photonvision/Best target list is empty", currentResultPair.isEmpty());

      if (!currentResultPair.isEmpty()) {
        logCount++;
        logs("/Photonvision/BestTarget updated counter", logCount);
        PhotonTrackedTarget bestTarget = currentResultPair.get(0).getSecond().getBestTarget();
        logs("/Photonvision/BestTarget is not null", bestTarget != null);

        logs("/Photonvision/Best Target is not null", bestTarget != null);
        logs("/Photonvision/Best Target is not null", bestTarget != null);

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
    //    List<Pair<PhotonModule, PhotonPipelineResult>> currentResultPair = resultPairs.get();

    logs(
        () -> {
          log("resultPairs get", resultPairs.get().isEmpty());
          log("resultPairs length", resultPairs.get().size());
          log("currentResultPair not null", currentResultPair != null);
        });

    if (currentResultPair != null) {
      logs("hasTargets currentResultPair", hasTargets(currentResultPair));
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

  public List<Pair<PhotonModule, PhotonPipelineResult>> getResultPairs() {
    return currentResultPair;
  }
}
