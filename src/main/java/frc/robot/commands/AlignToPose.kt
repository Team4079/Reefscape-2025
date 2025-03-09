package frc.robot.commands

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.trajectory.TrapezoidProfile
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.commands.Kommand.moveToClosestCoralScore
import frc.robot.commands.Kommand.moveToClosestCoralScoreNotL4
import frc.robot.subsystems.Swerve
import frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState
import frc.robot.utils.RobotParameters.FieldParameters.RobotPoses.addCoralPosList
import frc.robot.utils.RobotParameters.LiveRobotValues.visionDead
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.PROFILE_CONSTRAINTS
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.ROTATIONAL_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.X_PINGU
import frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.Y_PINGU
import frc.robot.utils.emu.Direction
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.pingu.LogPingu.log
import frc.robot.utils.pingu.LogPingu.logs
import frc.robot.utils.pingu.PathPingu.clearCoralScoringPositions
import java.sql.Driver

/**
 * Command to align the robot to a specific pose.
 *
 * @property offsetSide The direction to offset the alignment.
 */
open class AlignToPose(
    var offsetSide: Direction,
) : Command() {
    // Controllers for PID control
    lateinit var rotationalController: ProfiledPIDController
    lateinit var yController: ProfiledPIDController
    lateinit var xController: ProfiledPIDController

    // Target and current poses of the robot
    lateinit var targetPose: Pose2d
    lateinit var currentPose: Pose2d

    // Timer to manage alignment duration
    lateinit var timer: Timer

    // Check if all controllers are at their setpoints
    val controllersAtSetpoint: Boolean
        get() = rotationalController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint()

    /**
     * Initializes the command, setting up the controllers and target pose.
     */
    override fun initialize() {
        addRequirements(Swerve.getInstance())

        // Update the list of coral scoring positions to the correct side (hopefully)
        currentPose = Swerve.getInstance().pose2Dfrom3D

        timer = Timer()

        clearCoralScoringPositions()
        addCoralPosList()

        targetPose =
            if (elevatorToBeSetState == ElevatorState.L4) {
                moveToClosestCoralScore(offsetSide, Swerve.getInstance().pose2Dfrom3D)!!
            } else {
                moveToClosestCoralScoreNotL4(offsetSide, Swerve.getInstance().pose2Dfrom3D)!!
            }

        xController =
            X_PINGU.profiledPIDController.apply {
                setTolerance(0.015)
                setConstraints(PROFILE_CONSTRAINTS)
                setGoal(targetPose.x)
                reset(currentPose.x)
            }

        yController =
            Y_PINGU.profiledPIDController.apply {
                setTolerance(0.015)
                setConstraints(PROFILE_CONSTRAINTS)
                setGoal(targetPose.y)
                reset(currentPose.y)
            }

        rotationalController =
            ROTATIONAL_PINGU.profiledPIDController.apply {
                setTolerance(4.0)
                setConstraints(TrapezoidProfile.Constraints(5.0, 5.0))
                setGoal(targetPose.rotation.degrees)
                reset(currentPose.rotation.degrees)
                enableContinuousInput(-180.0, 180.0)
            }
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until [.isFinished]) returns true.
     *
     *
     * Refer to commit 9c00e5 for the old method of aligning the robot.
     */
    override fun execute() {
        currentPose = Swerve.getInstance().pose2Dfrom3D
        if (
            ((DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) && DriverStation.isTeleop())
            ||
            ((DriverStation.getAlliance().get() == DriverStation.Alliance.Red) && DriverStation.isAutonomous())
            )
        {
            Swerve
                .getInstance()
                .setDriveSpeeds(
                    xController.calculate(currentPose.x),
                    yController.calculate(currentPose.y),
                    rotationalController.calculate(currentPose.rotation.degrees),
                    true,
                )
        } else {
            Swerve
                .getInstance()
                .setDriveSpeeds(
                    -xController.calculate(currentPose.x),
                    -yController.calculate(currentPose.y),
                    rotationalController.calculate(currentPose.rotation.degrees),
                    true,
                )
        }
        alignLogs()
    }

    /**
     * Checks if the command is finished.
     *
     * @return True if the command is finished, false otherwise.
     */
    override fun isFinished(): Boolean {
        if ((rotationalController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint()) || visionDead) {
            timer.start()
        } else {
            timer.reset()
        }
        return timer.hasElapsed(0.2)
    }

    /**
     * Ends the command, stopping the swerve drive.
     *
     * @param interrupted Whether the command was interrupted.
     */
    override fun end(interrupted: Boolean) = Swerve.getInstance().stop()

    /**
     * Logs alignment data for debugging purposes.
     */
    fun alignLogs() {
        logs {
            log("AlignToPose/Current Pose", currentPose)
            log("AlignToPose/Target Pose", targetPose)
            log("AlignToPose/Rotational Error", rotationalController.positionError)
            log("AlignToPose/Y Error", yController.positionError)
            log("AlignToPose/X Error ", xController.positionError)
            log("AlignToPose/X Set ", xController.setpoint.position)
            log("AlignToPose/X Goal ", xController.goal.position)
            log("AlignToPose/Rotational Controller Setpoint", rotationalController.atSetpoint())
            log("AligntoPOse/Heading got from odometry", currentPose.rotation.degrees)
            log("AlignToPose/Y Controller Setpoint", yController.atSetpoint())
            log("AlignToPose/X Controller Setpoint ", xController.atSetpoint())
            log(
                "AlignToPose/X Set Speed ",
                xController.calculate(currentPose.x, targetPose.x),
            )
            log("AlignToPose/Y Set Speed ", yController.calculate(currentPose.y))
            log(
                "AlignToPose/Rot Set Speed ",
                rotationalController.calculate(currentPose.rotation.degrees),
            )
            log("AlignToPose/ X Set Pos", currentPose.x)
            log("AlignToPose/ Y Set Pos", currentPose.y)
            log("AlignToPose/ X Target Pos", targetPose.x)
            log("AlignToPose/ Y Target Pos", targetPose.y)
        }
    }
}
