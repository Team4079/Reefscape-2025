package frc.robot.utils

import edu.wpi.first.math.geometry.Pose2d
import frc.robot.subsystems.PhotonModule
import org.photonvision.EstimatedRobotPose
import org.photonvision.targeting.PhotonPipelineResult
import java.util.Optional

/**
 * Extension function for a list of PhotonModule objects to get the best PhotonPipelineResult.
 *
 * This function iterates through each PhotonModule in the list, retrieves the latest result,
 * and checks if it has targets. If it does, it compares the pose ambiguity of the target
 * with the current best ambiguity. If the current target's ambiguity is lower, it updates
 * the best result.
 *
 * @receiver List<PhotonModule> The list of PhotonModule objects to search through.
 * @return List<Pair<PhotonModule, PhotonPipelineResult>> The list of PhotonModule and PhotonPipelineResult pairs ordered by pose ambiguity.
 */
fun List<PhotonModule>.getDecentResultPairs(): List<Pair<PhotonModule, PhotonPipelineResult>> =
    this
        .mapNotNull { module ->
            // logs("PhotonModule allUnreadResults size", module.allUnreadResults.size)
            module.allUnreadResults
                .getOrNull(0)
                ?.takeIf { it.hasTargets() } // && it.bestTarget.poseAmbiguity < 0.2
                ?.let { module to it }
        }.sortedBy { it.second.bestTarget.poseAmbiguity }

/**
 * Extension function for a list of Pair<PhotonModule, PhotonPipelineResult> objects to check if any have targets.
 *
 * This function iterates through each pair in the list and checks if the PhotonPipelineResult has targets.
 *
 * @receiver List<Pair<PhotonModule, PhotonPipelineResult>> The list of pairs to check.
 * @return Boolean True if any pair has targets, false otherwise.
 */
fun List<Pair<PhotonModule, PhotonPipelineResult>>.hasTargets(): Boolean = this.any { it.second.hasTargets() }

/**
 * Extension function for a list of Pair<PhotonModule, PhotonPipelineResult> objects to check if any have multi-tag results.
 *
 * This function iterates through each pair in the list and checks if the PhotonPipelineResult has multi-tag results.
 *
 * @receiver List<Pair<PhotonModule, PhotonPipelineResult>> The list of pairs to check.
 * @return Boolean True if any pair has multi-tag results, false otherwise.
 */
fun List<Pair<PhotonModule, PhotonPipelineResult>>.hasMultiTag(): Boolean = this.any { it.second.multiTagResult.isPresent }

/**
 * Extension function for a Pair of PhotonModule and PhotonPipelineResult to get estimated poses.
 *
 * This function sets the reference pose for the pose estimator of the PhotonModule and updates it
 * with the PhotonPipelineResult. If an estimated robot pose is present, it adds it to the list of poses.
 *
 * @receiver Pair<PhotonModule, PhotonPipelineResult> The pair of PhotonModule and PhotonPipelineResult.
 * @param prevEstimatedRobotPose Pose2d? The previous estimated robot pose to set as reference.
 * @return List<EstimatedRobotPose> The list of estimated robot poses.
 */
fun Pair<PhotonModule, PhotonPipelineResult>.getEstimatedPose(prevEstimatedRobotPose: Pose2d?): EstimatedRobotPose? {
    first.poseEstimator.apply {
        setReferencePose(prevEstimatedRobotPose)
        return update(second).orElse(null)
    }
//    return null;
}

/**
 * Extension function for a Pair of PhotonModule and PhotonPipelineResult to update the standard deviations of the estimated robot pose.
 *
 * This function updates the estimated standard deviations of the robot pose using the provided estimated robot pose
 * and the targets from the PhotonPipelineResult.
 *
 * @receiver Pair<PhotonModule, PhotonPipelineResult> The pair of PhotonModule and PhotonPipelineResult.
 * @param estimatedRobotPose Optional<EstimatedRobotPose> The estimated robot pose to use for updating the standard deviations.
 */
fun Pair<PhotonModule, PhotonPipelineResult>.updateStdDev(estimatedRobotPose: Optional<EstimatedRobotPose>) =
    first.updateEstimatedStdDevs(estimatedRobotPose, second.getTargets())
