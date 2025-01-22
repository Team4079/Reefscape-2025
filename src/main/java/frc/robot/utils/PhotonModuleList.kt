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
 * @return PhotonPipelineResult? The best PhotonPipelineResult found, or null if no valid result is found.
 */
fun List<PhotonModule>.getBestResultPair(): Pair<PhotonModule, PhotonPipelineResult>? =
    this
        .mapNotNull { module ->
            module.latestResult?.takeIf { it.hasTargets() }?.let { module to it }
        }.minByOrNull { it.second.bestTarget.poseAmbiguity }
