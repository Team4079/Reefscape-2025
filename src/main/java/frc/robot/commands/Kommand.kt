package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.WaitCommand
import frc.robot.commands.sequencing.AutomaticScore
import frc.robot.subsystems.CoralManipulator
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Swerve
import frc.robot.utils.Direction
import frc.robot.utils.ElevatorState
import frc.robot.utils.ElevatorState.L4
import frc.robot.utils.RobotParameters.SwerveParameters
import frc.robot.utils.RobotParameters.SwerveParameters.PIDParameters.PATH_CONSTRAINTS

/**
 * The [Kommand] object provides factory methods to create various commands
 * used in the robot's operation.
 *
 * This is called for instant commands instead of functions
 */
object Kommand {
    /**
     * Creates an [InstantCommand] that executes the given function.
     *
     * @param function The function to execute.
     * @return An [InstantCommand] that executes the given function.
     */
    @JvmStatic
    fun cmd(function: () -> Unit) = InstantCommand(function)

    /**
     * Creates an [InstantCommand] to set the state of the elevator.
     *
     * @param state The desired state of the elevator.
     * @return An [InstantCommand] that sets the elevator state.
     */
    @JvmStatic
    fun setElevatorState(state: ElevatorState) = cmd { Elevator.getInstance().state = state }

    /**
     * Creates an [InstantCommand] to move the elevator to a specific level.
     *
     * @return An [InstantCommand] that moves the elevator to a specific level.
     */
    @JvmStatic
    fun moveElevatorToLevel() = cmd { Elevator.getInstance().moveElevatorToLevel() }

    /**
     * Creates an [InstantCommand] to start the coral manipulator motors.
     *
     * @return An [InstantCommand] that starts the coral manipulator motors.
     */
    @JvmStatic
    fun startCoralManipulator() = cmd { CoralManipulator.getInstance().setHasPiece(false) }

    /**
     * Creates an [InstantCommand] to stop the coral manipulator motors.
     *
     * @return An [InstantCommand] that stops the coral manipulator motors.
     */
    @JvmStatic
    fun stopCoralManipulator() = cmd { CoralManipulator.getInstance().stopMotors() }

    /**
     * Creates an [AutomaticScore] command to score in a specified direction.
     *
     * @param dir The direction in which to score.
     * @param state The desired state of the elevator. Defaults to [L4].
     * @return An [AutomaticScore] that performs the scoring action.
     */
    @JvmStatic
    @JvmOverloads
    fun score(
        dir: Direction,
        state: ElevatorState = L4,
    ) = AutomaticScore(dir, state)

    /**
     * Creates an [AlignSwerve] command to align the robot in a specified direction.
     *
     * @param dir The direction in which to align the robot.
     * @return An [AlignSwerve] command that aligns the robot.
     */
    @JvmStatic
    fun align(dir: Direction) = AlignSwerve(dir)

    /**
     * Creates a [PadDrive] command to control the robot's driving mechanism.
     *
     * @param controller The gaming controller used to drive the robot.
     * @return A [PadDrive] command to control the robot's driving mechanism.
     */
    @JvmStatic
    fun drive(controller: XboxController) = PadDrive(controller)

    /**
     * Creates an [InstantCommand] to reset the Pidgey sensor.
     *
     * @return An [InstantCommand] that resets the Pidgey sensor.
     */
    @JvmStatic
    fun resetPidgey() = cmd { Swerve.getInstance().resetPidgey() }

    /**
     * Creates an [InstantCommand] to set the teleoperation PID.
     *
     * @return An [InstantCommand] that sets the teleoperation PID.
     */
    @JvmStatic
    fun setTelePid() = cmd { Swerve.getInstance().setTelePID() }

    /**
     * Creates a [PathPlannerAuto] command for autonomous operation.
     *
     * @return A [PathPlannerAuto] command for autonomous operation.
     */
    @JvmStatic
    fun autonomousCommand() = PathPlannerAuto(SwerveParameters.PATHPLANNER_AUTO_NAME)

    /**
     * Creates a [WaitCommand] to wait for a specified number of seconds.
     *
     * @param seconds The number of seconds to wait.
     * @return A [WaitCommand] that waits for the specified number of seconds.
     */
    @JvmStatic
    fun waitCmd(seconds: Double) = WaitCommand(seconds)

    /**
     * Creates a pathfinding command to move to a specified pose.
     *
     * @param targetPose The target pose to move to.
     * @param endVelocity The end velocity for the pathfinding. Defaults to 0.0.
     * @return A command that performs the pathfinding operation.
     */
    @JvmStatic
    @JvmOverloads
    fun createPathfindingCmd(
        targetPose: Pose2d,
        endVelocity: Double = 0.0,
    ): Command =
        AutoBuilder.pathfindToPose(
            targetPose,
            PATH_CONSTRAINTS,
            endVelocity, // Goal end velocity in meters/sec
        )

    /**
     * Creates a [PadDrive] command to control the elevator.
     *
     * @param controller The gaming controller used to move the elevator.
     * @return A [PadDrive] command to control the robot's elevator.
     */
    @JvmStatic
    fun padElevator(controller: XboxController) = PadElevator(controller)
}
