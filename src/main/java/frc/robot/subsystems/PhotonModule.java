package frc.robot.subsystems;

import edu.wpi.first.apriltag.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.*;
import java.util.*;

import edu.wpi.first.math.numbers.*;
import frc.robot.utils.RobotParameters.*;
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
  private Matrix<N3, N1> estimatedStdDev;

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
        new PhotonPoseEstimator(
            fieldLayout, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPos);
    photonPoseEstimator.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
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

  /**
   * Gets the estimated robot pose based on the latest vision processing results.
   *
   * <p>This method retrieves all unread pipeline results from the camera and checks if there is a
   * multi-tag result available. If a multi-tag result is present, it extracts and returns the
   * translation component of the estimated pose. If no multi-tag result is available, it returns a
   * default Translation3d object.
   *
   * @return Translation3d The estimated robot pose as a Translation3d object. If no multi-tag
   *     result is available, returns a default Translation3d object.
   */
  public Translation3d getEstimatedRobotPose() {
    List<PhotonPipelineResult> currentResult = camera.getAllUnreadResults();
    if (currentResult.get(0).multitagResult.isPresent()) {
      return currentResult.get(0).getMultiTagResult().get().estimatedPose.best.getTranslation();
    }
    return new Translation3d();
  }

  public void updateEstimatedStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
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
    var stdDevs = (numTags > 1) ? PhotonVisionConstants.MULTI_TARGET_STD_DEV : PhotonVisionConstants.SINGLE_TARGET_STD_DEV;

    if (numTags == 1 && avgDistance > 4) {
      currentStdDev = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
      System.out.println("I blame Om!!!");
    } else {
      currentStdDev = stdDevs.times(1 + (avgDistance * avgDistance / 30));
    }
  }

}
