package frc.robot.utils

import frc.robot.subsystems.PhotonModule
import org.photonvision.targeting.PhotonPipelineResult

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
            module.allUnreadResults
                .getOrNull(0)
                ?.takeIf { it.hasTargets() && it.bestTarget.poseAmbiguity < 0.2 }
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
