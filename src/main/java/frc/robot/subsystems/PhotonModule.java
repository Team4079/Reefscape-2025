package frc.robot.subsystems;

import static edu.wpi.first.math.VecBuilder.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.*;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.*;
import frc.robot.utils.RobotParameters.*;
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
  private Matrix<N3, N1> currentStdDev;
  private Matrix<N4, N1> currentStdDev3d;

  /**
   * Creates a new CameraModule with the specified parameters.
   *
   * @param cameraName The name of the camera in the Photonvision interface
   * @param cameraPos The 3D transform representing the camera's position relative to the robot
   * @param fieldLayout The AprilTag field layout used for pose estimation
   */
  public PhotonModule(String cameraName, Transform3d cameraPos, AprilTagFieldLayout fieldLayout) {
    this.camera = new PhotonCamera(cameraName);
    this.cameraPos = cameraPos;
    this.photonPoseEstimator =
        new PhotonPoseEstimator(fieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, cameraPos);
    photonPoseEstimator.setMultiTagFallbackStrategy(
        PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
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
   * @return {@link Transform3d}, The {@link Transform3d} representing the camera's position
   */
  public Transform3d getCameraPosition() {
    return cameraPos;
  }

  /**
   * Updates the estimated standard deviations based on the provided estimated pose and list of
   * tracked targets.
   *
   * <p>This method calculates the number of visible tags and their average distance to the
   * estimated pose. It then uses this information to adjust the standard deviations used for robot
   * pose estimation.
   *
   * @param estimatedPose An Optional containing the estimated robot pose.
   * @param targets A list of PhotonTrackedTarget objects representing the tracked targets.
   */
  public void updateEstimatedStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      currentStdDev = PhotonVisionConstants.SINGLE_TARGET_STD_DEV;
      return;
    }
    int numTags = 0;
    double totalDistance = 0;

    // Calculate the number of visible tags and their average distance to the estimated pose
    for (var target : targets) {
      var tagPoseOptional = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPoseOptional.isEmpty()) continue;

      numTags++;
      var tagPose = tagPoseOptional.get().toPose2d().getTranslation();
      var estimatedTranslation = estimatedPose.get().estimatedPose.toPose2d().getTranslation();
      totalDistance += tagPose.getDistance(estimatedTranslation);
    }

    if (numTags == 0) {
      currentStdDev = PhotonVisionConstants.SINGLE_TARGET_STD_DEV;
      return;
    }

    double avgDistance = totalDistance / numTags;
    var stdDevs =
        (numTags > 1)
            ? PhotonVisionConstants.MULTI_TARGET_STD_DEV
            : PhotonVisionConstants.SINGLE_TARGET_STD_DEV;

    if (numTags == 1 && avgDistance > 4) {
      currentStdDev = fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      currentStdDev = stdDevs.times(1 + (avgDistance * avgDistance / 30));
    }
  }

  /**
   * Updates the estimated standard deviations based on the provided estimated pose and list of
   * tracked targets. This method calculates the number of visible tags and their average 3D
   * distance to the estimated pose. It then uses this information to adjust the standard deviations
   * used for robot pose estimation.
   *
   * @param estimatedPose
   * @param targets
   */
  public void updateEstimatedStdDevs3d(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      currentStdDev3d = PhotonVisionConstants.SINGLE_TARGET_STD_DEV_3D;
      return;
    }
    int numTags = 0;
    double totalDistance = 0;

    // Calculate the number of visible tags and their average 3D distance to the estimated pose
    for (var target : targets) {
      var tagPoseOptional = photonPoseEstimator.getFieldTags().getTagPose(target.getFiducialId());
      if (tagPoseOptional.isEmpty()) continue;

      numTags++;
      var tagPose = tagPoseOptional.get().getTranslation();
      var estimatedTranslation = estimatedPose.get().estimatedPose.getTranslation();

      // Calculate 3D Euclidean distance
      double deltaX = tagPose.getX() - estimatedTranslation.getX();
      double deltaY = tagPose.getY() - estimatedTranslation.getY();
      double deltaZ = tagPose.getZ() - estimatedTranslation.getZ();
      totalDistance += Math.sqrt(deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ);
    }

    if (numTags == 0) {
      currentStdDev3d = PhotonVisionConstants.SINGLE_TARGET_STD_DEV_3D;
      return;
    }

    double avgDistance = totalDistance / numTags;
    var stdDevs =
        (numTags > 1)
            ? PhotonVisionConstants.MULTI_TARGET_STD_DEV_3D
            : PhotonVisionConstants.SINGLE_TARGET_STD_DEV_3D;

    if (numTags == 1 && avgDistance > 4) {
      currentStdDev3d =
          fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
    } else {
      currentStdDev3d = stdDevs.times(1 + (avgDistance * avgDistance / 30));
    }
  }

  /**
   * Gets the current standard deviations used for robot pose estimation.
   *
   * @return Matrix<N3, N1> The current standard deviations as a Matrix object
   */
  public Matrix<N3, N1> getCurrentStdDevs() {
    return currentStdDev;
  }

  /**
   * Gets the current standard deviations used for 3D robot pose estimation.
   *
   * @return Matrix<N4, N1> The current standard deviations as a Matrix object
   */
  public Matrix<N4, N1> getCurrentStdDevs3d() {
    return currentStdDev3d;
  }

  /**
   * Gets the name of the camera associated with this module.
   *
   * @return String The name of the camera
   */
  public String getCameraName() {
    return camera.getName();
  }

  public PhotonCamera getCamera() {
    return camera;
  }
}
