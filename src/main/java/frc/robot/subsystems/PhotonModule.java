package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.geometry.*;
import java.util.*;
import org.photonvision.*;
import org.photonvision.targeting.*;

/**
 * The CameraModule class represents a single Photonvision camera setup with its associated pose
 * estimator and position information. This class encapsulates all the functionality needed for a
 * single camera to track AprilTags and estimate robot pose.
 */
public class PhotonModule {
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonPoseEstimator;
  private final Transform3d cameraPos;

  /**
   * Creates a new CameraModule with the specified parameters.git p
   *
   * @param cameraName The name of the camera in the Photonvision interface
   * @param cameraPos The 3D transform representing the camera's position relative to the robot
   * @param fieldLayout The AprilTag field layout used for pose estimation
   */
  public PhotonModule(String cameraName, Transform3d cameraPos, AprilTagFieldLayout fieldLayout) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraPos = cameraPos;
    this.photonPoseEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPos);
  }

  /**
   * Gets all unread pipeline results from the camera.
   *
   * @return A list of PhotonPipelineResult objects containing the latest vision processing results
   */
  public List<PhotonPipelineResult> getAllUnreadResults() {
    return camera.getAllUnreadResults();
  }

  /**
   * Gets the pose estimator associated with this camera.
   *
   * @return PhotonPoseEstimator, The PhotonPoseEstimator object used for robot pose estimation
   */
  public PhotonPoseEstimator getPoseEstimator() {
    return photonPoseEstimator;
  }

  /**
   * Gets the camera's position relative to the robot.
   *
   * @return Transform3d, The Transform3d representing the camera's position
   */
  public Transform3d getCameraPosition() {
    return cameraPos;
  }
}
