package frc.robot.utils.pingu

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand.createPathfindingCmd
import frc.robot.utils.emu.Direction

/**
 * Type alias for a list of scoring positions, each defined by an Translation2d AprilTag location,
 * a reef scoring Pose2d location for the left coral, and a Pose2d location for the right coral.
 * The april tag position is to find the closest pair of reef scoring positions.
 * Sorry Om I barely know how to code in Kotlin :(
 */
typealias CoralScore = Triple<Translation2d, Pose2d, Pose2d>


/**
 * Object that manages scoring positions but does not store them directly.
 * Instead, it provides methods to retrieve the closest scoring position.
 */
object PathPingu {
    private val scoringPositions = mutableListOf<CoralScore>()

    /**
     * Adds any number of coral scoring positions to the list of potential scoring positions.
     *
     * @param positions One or more lists of triples containing the AprilTag location, left coral position, and right coral position.
     */
    fun addCoralScoringPositions(vararg positions: List<CoralScore>) {
        positions.iterator().forEach {
            scoringPositions.addAll(it)
        }
    }

    /**
     * Finds the closest scoring position to the robot's current position.
     * Starts by finding the closest april tag to the robot's current position.
     * Based on the direction passed in, it returns the corresponding scoring position.
     * Then it path finds to that position (hopefully).
     *
     * @param position The robot's current position.
     * @param direction The direction in which to find the closest scoring position.
     * @return The command to move to the closest scoring position.
     */
    fun findClosestScoringPosition(position: Pose2d, direction: Direction): Command? {
        val closest = scoringPositions.minByOrNull { position.translation.getDistance(it.first) }
        return closest?.let { if (direction == Direction.LEFT) it.second else it.third }
            ?.let { createPathfindingCmd(it) }
    }
}