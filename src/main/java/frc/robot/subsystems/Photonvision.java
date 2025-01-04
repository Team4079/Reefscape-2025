package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Dash;
import frc.robot.utils.Extensions;
import frc.robot.utils.RobotParameters.PhotonVisionConstants;
import java.util.List;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** The PhotonVision subsystem handles vision processing using PhotonVision cameras. */
public class Photonvision extends SubsystemBase {
  // PhotonVision cameras
  private final PhotonCamera camera = new PhotonCamera("Camera");

  // Pose estimator for determining the robot's position on the field
  private final PhotonPoseEstimator photonPoseEstimator;

  private final Translation2d cameraTrans = new Translation2d(0.31, 0.0);

  // AprilTag field layout for the 2024 Crescendo field
  private AprilTagFieldLayout aprilTagFieldLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);

  // Transformation from the robot to the camera
  private Transform3d cameraPos =
      new Transform3d(
          Extensions.dimensionIncrease(cameraTrans, PhotonVisionConstants.CAMERA_ONE_HEIGHT_METER),
          new Rotation3d(
              0.0,
              Math.toRadians(360 - PhotonVisionConstants.CAMERA_ONE_ANGLE_DEG),
              Math.toRadians(180.0)));

  private PhotonTrackedTarget target = null;
  private boolean targetVisible = false;
  private double yaw = -15.0;
  private double targetPoseAmbiguity = 7157.0;
  private double range = 0.0;
  private double rangeToTarget = 0.0;
  private List<PhotonPipelineResult> result;
  private PhotonPipelineResult currentResult = null;
  private boolean camTag = false;
  private Translation3d currentPose = null;

  /** Constructs a new PhotonVision subsystem. */
  public Photonvision() {
    // Initialize result
    result = camera.getAllUnreadResults();
    photonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPos);
  }

  /**
   * This method is called periodically by the scheduler. It updates the tracked targets and
   * displays relevant information on the SmartDashboard.
   */
  @Override
  public void periodic() {
    result = camera.getAllUnreadResults();
    currentResult = result.isEmpty() ? null : result.get(0);

    if (currentResult == null) return;

    photonPoseEstimator.update(currentResult);

    target = currentResult.getBestTarget();
    targetPoseAmbiguity = target != null ? target.getPoseAmbiguity() : 7157.0;

    for (PhotonTrackedTarget tag : currentResult.getTargets()) {
      if (tag.getFiducialId() == 7 || tag.getFiducialId() == 4) {
        yaw = tag.getYaw();
      }
    }

    Dash.dash(
        Dash.pairOf("yaw to target", yaw),
        Dash.pairOf("range target", rangeToTarget),
        Dash.pairOf("april tag distance", getDistanceSubwoofer()),
        Dash.pairOf("april tag yaw", getSubwooferYaw()),
        Dash.pairOf("cam ambiguity", targetPoseAmbiguity),
        Dash.pairOf("_targets", currentResult.hasTargets()));
  }

  /**
   * Checks if there is a tag.
   *
   * <p>This method is useful to avoid NullPointerExceptions when trying to access specific info
   * based on vision.
   *
   * @return true if there is a tag, false otherwise.
   */
  public boolean hasTag() {
    return currentResult != null && currentResult.hasTargets();
  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @param prevEstimatedRobotPose The previous estimated pose of the robot.
   * @return The estimated robot pose, or null if no pose could be estimated.
   */
  public EstimatedRobotPose getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return currentResult != null ? photonPoseEstimator.update(currentResult).orElse(null) : null;
  }

  /**
   * Gets the estimated global pose of the robot.
   *
   * @return The estimated global pose of the robot.
   */
  public Transform3d getEstimatedGlobalPose() {
    return currentResult != null && currentResult.getMultiTagResult().isPresent()
        ? currentResult.getMultiTagResult().get().estimatedPose.best
        : new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
  }

  /**
   * Gets the forward distance to the target.
   *
   * @return The forward distance to the target.
   */
  public double getPivotPosition() {
    // 10/14/2024 outside tuning
    // Desmos: https://www.desmos.com/calculator/naalukjxze
    double r = getDistanceSubwoofer() + 0.6;
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
   * Calculates and returns the distance to the subwoofer for the 2024 Crescendo game.
   *
   * <p>This method computes the Euclidean distance (distance formula) from the robot's current
   * position to the location of the subwoofer. The calculation varies based on the alliance:
   *
   * <ul>
   *   <li>If the alliance is {@code RED} or not specified, the subwoofer is assumed to be at
   *       coordinates (16.5, 5.5).
   *   <li>If the alliance is {@code BLUE}, the subwoofer is assumed to be at (0, 5.5).
   * </ul>
   *
   * If the current pose is not available, the method returns a default value of {@code 687.0}. This
   * value reflects our team's tradition of thanking teams for their help to us. :D
   *
   * @return The calculated distance to the subwoofer, or {@code 687.0} if the current pose is
   *     unavailable.
   */
  public double getDistanceSubwoofer() {
    currentPose = getEstimatedGlobalPose().getTranslation();
    if (currentPose != null) {
      if (DriverStation.getAlliance().isEmpty()
          || DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
        return Math.sqrt(
            Math.pow(currentPose.getX() - 16.5, 2.0) + Math.pow(currentPose.getY() - 5.5, 2.0));
      } else {
        return Math.sqrt(
            Math.pow(currentPose.getX(), 2.0) + Math.pow(currentPose.getY() - 5.5, 2.0));
      }
    } else {
      return 687.0;
    }
  }

  /**
   * Gets the yaw of the subwoofer.
   *
   * @return The yaw of the subwoofer.
   */
  public double getSubwooferYaw() {
    return 180 - Math.toDegrees(getEstimatedGlobalPose().getRotation().getAngle());
  }
}
