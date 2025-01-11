package frc.robot.utils

import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.wpilibj2.command.InstantCommand
import frc.robot.commands.sequencing.AutomaticScore
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.SwerveParameters

/**
 * The [Kommand] object provides factory methods to create various commands
 * used in the robot's operation.
 */
object Kommand {
    /**
     * Creates an [InstantCommand] to set the state of the elevator.
     *
     * @param state The desired state of the elevator.
     * @return An [InstantCommand] that sets the elevator state.
     */
    @JvmStatic
    fun setElevatorState(state: ElevatorState) = InstantCommand({ Elevator.getInstance().state = state })

    /**
     * Creates an [InstantCommand] to score in a specified direction.
     *
     * @param dir The direction in which to score.
     * @return An [InstantCommand] that performs the scoring action.
     */
    @JvmStatic
    fun score(dir: Direction) = InstantCommand({ AutomaticScore(dir) })

    /**
     * Creates an [InstantCommand] to reset the Pidgey sensor.
     *
     * @return An [InstantCommand] that resets the Pidgey sensor.
     */
    @JvmStatic
    fun resetPidgey() = InstantCommand({ Swerve.getInstance().resetPidgey() })

    /**
     * Creates an [InstantCommand] to set the teleoperation PID.
     *
     * @return An [InstantCommand] that sets the teleoperation PID.
     */
    @JvmStatic
    fun setTelePid() = InstantCommand({ Swerve.getInstance().setTelePID() })

    /**
     * Creates a [PathPlannerAuto] command for autonomous operation.
     *
     * @return A [PathPlannerAuto] command for autonomous operation.
     */
    @JvmStatic
    fun autonomousCommand() = PathPlannerAuto(SwerveParameters.PATHPLANNER_AUTO_NAME)
}
