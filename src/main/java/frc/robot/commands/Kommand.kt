package frc.robot.commands

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.*
import frc.robot.commands.sequencing.AutomaticScore
import frc.robot.subsystems.Coral
import frc.robot.subsystems.Elevator
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.CoralManipulatorParameters.hasPiece
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.PATH_CONSTRAINTS
import frc.robot.utils.emu.CoralState
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.emu.ElevatorState.L4
import frc.robot.utils.pingu.PathPingu.findClosestScoringPosition
import frc.robot.utils.pingu.PathPingu.findClosestScoringPositionNotL4
import kotlin.math.abs

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
    fun cmd(
        vararg reqs: Subsystem,
        function: () -> Unit,
    ) = InstantCommand(function, *reqs)

    /**
     * Creates a [WaitUntilCommand] that waits until the given condition is true.
     *
     * @param function The condition to evaluate.
     * @return A [WaitUntilCommand] that waits until the condition is true.
     */
    @JvmStatic
    fun waitUntil(function: () -> Boolean) = WaitUntilCommand(function)

    /**
     * Creates an [InstantCommand] to set the state of the elevator.
     *
     * @param state The desired state of the elevator.
     * @return An [InstantCommand] that sets the elevator state.
     */
    @JvmStatic
    fun setElevatorState(state: ElevatorState) = cmd { Elevator.getInstance().state = state }

    /**
     * Creates a [SequentialCommandGroup] to move the elevator to a specified state.
     *
     * @param state The desired state of the elevator.
     * @return A [SequentialCommandGroup] that moves the elevator to the specified state.
     */
    @JvmStatic
    fun moveElevatorState(state: ElevatorState) =
        SequentialCommandGroup(
            cmd(Elevator.getInstance()) {
                Elevator.getInstance().state = state
            },
            waitUntil {
                abs(Elevator.getInstance().elevatorPosAvg - state.pos) < 0.3
            },
        )

    /**
     * Creates an [InstantCommand] to set the state of the coral manipulator.
     *
     * @param state The desired state of the coral manipulator.
     * @return An [InstantCommand] that sets the coral manipulator state.
     */
    @JvmStatic
    fun setCoralState(state: CoralState) = cmd { Coral.getInstance().setState(state) }

    /**
     * Creates an [InstantCommand] to start the coral manipulator motors.
     *
     * @return An [InstantCommand] that starts the coral manipulator motors.
     */
    @JvmStatic
    fun startCoralManipulator() = cmd { Coral.getInstance().setHasPiece(false) }

    /**
     * Creates an [InstantCommand] to stop the coral manipulator motors.
     *
     * @return An [InstantCommand] that stops the coral manipulator motors.
     */
    @JvmStatic
    fun stopCoralManipulator() = cmd { Coral.getInstance().stopMotors() }

    /**
     * Creates an [AutomaticScore] command to score in a specified direction.
     *
     * @param dir The direction in which to score.
     * @param state The desired state of the elevator. Defaults to [L4].
     * @return An [AutomaticScore] that performs the scoring action.
     */
    @JvmStatic
    fun score(dir: Direction) = cmd {}
//    fun score(dir: Direction) = AutomaticScore(dir)

    /**
     * Creates an new [ReverseIntake] command to reverse the intake.
     *
     * @return A [ReverseIntake] command that reverses the intake.
     */
    @JvmStatic
    fun reverseIntake() = ReverseIntake()

    /**
     * Creates a new [InstantCommand] to start the coral motors
     */
    @JvmStatic
    fun startCoralMotors() = cmd { Coral.getInstance().startCoralIntake() }

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
     * Creates an [InstantCommand] to set the intake to algae.
     *
     * @return An [InstantCommand] that sets the intake to algae.
     */
    @JvmStatic
    fun setIntakeAlgae() = ToggleIntakeAlgae()

    /**
     * Creates a [PathPlannerAuto] command for autonomous operation.
     *
     * @return A [PathPlannerAuto] command for autonomous operation.
     */
//    @JvmStatic
//    fun autonomousCommand() = PathPlannerAuto(SwerveParameters.PATHPLANNER_AUTO_NAME)

    /**
     * Creates a [WaitCommand] to wait for a specified number of seconds.
     *
     * @param seconds The number of seconds to wait.
     * @return A [WaitCommand] that waits for the specified number of seconds.
     */
    @JvmStatic
    fun waitCmd(seconds: Double) = WaitCommand(seconds)

    @JvmStatic
    fun cancelCmd() = cmd{ CommandScheduler.getInstance().cancelAll() }

    /**
     * Creates a pathfinding command to move to a specified pose.
     *
     * @param targetPose The target pose to move to.
     * @param endVelocity The end velocity for the pathfinding in m/s. Defaults to 0.0.
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
            endVelocity,
        )

    /**
     * Creates a [PadDrive] command to control the elevator.
     *
     * @param controller The gaming controller used to move the elevator.
     * @return A [PadDrive] command to control the robot's elevator.
     */
    @JvmStatic
    fun padElevator(
        controller: XboxController,
        controller2: XboxController,
    ) = PadElevator(controller, controller2)

    /**
     * Creates an [InstantCommand] to set the coral manipulator intaking state to true.
     *
     * @return An [InstantCommand] that sets the coral intaking state to true.
     */
    @JvmStatic
    fun coralIntaking() = cmd { hasPiece = false }

    /**
     * Creates a command to move the robot to the closest coral
     * scoring position in the specified coral direction.
     * @param direction The direction in which to find the closest scoring position.
     *
     * @return A command that performs the pathfinding operation.
     */
    @JvmStatic
    fun moveToClosestCoralScore(
        direction: Direction,
        pose: Pose2d,
    ) = findClosestScoringPosition(pose, direction)

    /**
     * Creates a command to move the robot to the closest coral
     * scoring position in the specified coral direction.
     * @param direction The direction in which to find the closest scoring position.
     *
     * @return A command that performs the pathfinding operation.
     */
    @JvmStatic
    fun moveToClosestCoralScoreNotL4(
        direction: Direction,
        pose: Pose2d,
    ) = findClosestScoringPositionNotL4(pose, direction)
}
