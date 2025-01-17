package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics

/** Class containing global values for the robot.  */
object RobotParameters {
    /** Class containing global values related to motors.  */
    object MotorParameters {
        // Motor CAN ID Values
        const val FRONT_LEFT_STEER_ID: Int = 1
        const val FRONT_LEFT_DRIVE_ID: Int = 2
        const val FRONT_RIGHT_STEER_ID: Int = 3
        const val FRONT_RIGHT_DRIVE_ID: Int = 4
        const val BACK_LEFT_STEER_ID: Int = 5
        const val BACK_LEFT_DRIVE_ID: Int = 6
        const val BACK_RIGHT_STEER_ID: Int = 7
        const val BACK_RIGHT_DRIVE_ID: Int = 8
        const val FRONT_LEFT_CAN_CODER_ID: Int = 9
        const val FRONT_RIGHT_CAN_CODER_ID: Int = 10
        const val BACK_LEFT_CAN_CODER_ID: Int = 11
        const val BACK_RIGHT_CAN_CODER_ID: Int = 12
        const val ELEVATOR_MOTOR_LEFT_ID: Int = 13
        const val ELEVATOR_MOTOR_RIGHT_ID: Int = 14
        const val PIVOT_MOTOR_ID: Int = 15
        const val PIDGEY_ID: Int = 16
        const val END_EFFECTOR_MOTOR_ID: Int = 17
        const val CORAL_MANIPULATOR_MOTOR_UP_ID: Int = 18
        const val CORAL_MANIPULATOR_MOTOR_DOWN_ID: Int = 19

        // Motor Property Values
        const val MAX_SPEED: Double = 5.76
        const val MAX_ANGULAR_SPEED: Double = (14 * Math.PI) / 3
        const val ENCODER_COUNTS_PER_ROTATION: Double = 1.0
        const val STEER_MOTOR_GEAR_RATIO: Double = 150.0 / 7
        const val DRIVE_MOTOR_GEAR_RATIO: Double = 6.750000000000000
        private const val WHEEL_DIAMETER: Double = 0.106
        const val METERS_PER_REV: Double = WHEEL_DIAMETER * Math.PI * 0.975

        // Limit Values
        const val DRIVE_SUPPLY_LIMIT: Double = 45.0
        const val DRIVE_STATOR_LIMIT: Double = 80.0
        const val STEER_SUPPLY_LIMIT: Double = 30.0
    }

    /** Class containing global values related to the swerve drive system.  */
    object SwerveParameters {
        const val PATHPLANNER_AUTO_NAME: String = "4l4auto"

        const val AUTO_ALIGN_SWERVE_LEFT: Double = -0.1
        const val AUTO_ALIGN_SWERVE_RIGHT: Double = 0.1

        /** Class containing PID constants for the swerve drive system.  */
        object PIDParameters {
            @JvmField
            val STEER_PID_TELE: PIDVController = PIDVController(250.0, 0.000, 20.0, 0.0)

            // val STEER_PID_AUTO: PIDVController = PIDVController(200.0, 0.000, 20.0, 0.0)
            @JvmField
            val STEER_PID_AUTO: PIDVController = PIDVController(750.0, 5.000, 15.0, 0.0)
            // val STEER_PID_AUTO: PIDVController = PIDVController(5.0, 0.000, 0.0, 1.0)

            @JvmField
            val DRIVE_PID_AUTO: PIDVController = PIDVController(5.0, 0.0, 0.0, 0.4)

            @JvmField
            val DRIVE_PID_TELE: PIDVController =
                PIDVController(
                    5.0,
                    0.0,
                    0.0,
                    0.4,
                )

            @JvmField
            val ROTATIONAL_PID: PIDController = PIDController(0.2, 0.0, 0.0)

            @JvmField
            val Y_PID: PIDController = PIDController(0.2, 0.0, 0.0)

            @JvmField
            val DIST_PID: PIDController = PIDController(0.2, 0.0, 0.0)
            val PASS_ROTATIONAL_PID: PIDController = PIDController(0.1, 0.0, 0.0)

            var pathFollower: PPHolonomicDriveController =
                PPHolonomicDriveController(
                    PIDConstants(5.0, 0.00, 0.0), // translation
                    PIDConstants(5.0, 0.0, 0.0), // rotation
                )

            @JvmField
            var config: RobotConfig? = null

            init {
                try {
                    config = RobotConfig.fromGUISettings()
                } catch (e: Exception) {
                    throw RuntimeException("Failed to load robot config", e)
                }
            }
        }

        /** Class containing physical dimensions and kinematics for the swerve drive system.  */
        object PhysicalParameters {
            const val ROBOT_SIZE: Double = 0.43105229381
            private val FRONT_LEFT: Translation2d = Translation2d(0.3048, 0.3048)
            private val FRONT_RIGHT: Translation2d = Translation2d(0.3048, -0.3048)
            private val BACK_LEFT: Translation2d = Translation2d(-0.3048, 0.3048)
            private val BACK_RIGHT: Translation2d = Translation2d(-0.3048, -0.3048)

            @JvmField
            val kinematics: SwerveDriveKinematics =
                SwerveDriveKinematics(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT)
        }

        /** Class containing various thresholds and constants for the swerve drive system.  */
        object Thresholds {
            const val STATE_SPEED_THRESHOLD: Double = 0.05
            const val CANCODER_VAL9: Double = -0.419189
            const val CANCODER_VAL10: Double = -0.825928 - 0.5
            const val CANCODER_VAL11: Double = -0.475098
            const val CANCODER_VAL12: Double = -0.032959 + 0.5

            @JvmField
            val DRIVE_MOTOR_INVERTED: InvertedValue = InvertedValue.CounterClockwise_Positive

            @JvmField
            val STEER_MOTOR_INVERTED: InvertedValue = InvertedValue.Clockwise_Positive
            const val JOYSTICK_DEADBAND: Double = 0.05
            const val USING_VISION: Boolean = false
            const val AUTO_ALIGN: Boolean = false
            const val MOTOR_DEADBAND: Double = 0.05
            const val IS_FIELD_ORIENTED: Boolean = true
            const val SHOULD_INVERT: Boolean = false
            const val ENCODER_OFFSET: Double = (0 / 360.0)
            const val X_DEADZONE: Double = 0.15 * 5.76
            const val Y_DEADZONE: Double = 0.15 * 5.76
            const val OFF_BALANCE_ANGLE_THRESHOLD: Double = 10.0
            const val ON_BALANCE_ANGLE_THRESHOLD: Double = 5.0

            // Testing boolean for logging (to not slow down the robot)
            const val TEST_MODE: Boolean = true
        }
    }

    /** Class containing constants for the Photonvision subsystem.  */
    object PhotonVisionConstants {
        const val CAMERA_ONE_HEIGHT_METER: Double = 0.47
        const val CAMERA_ONE_ANGLE_DEG: Double = 33.0
        const val OFFSET_TOWARD_MID_LEFT: Double = -15.00
        const val CAMERA_TWO_HEIGHT_METER: Double = 0.61
        const val CAMERA_TWO_ANGLE_DEG: Double = 37.5
        const val OFFSET_TOWARD_MID_RIGHT: Double = 15.0
    }

    /** Class containing constants for the elevator subsystem.  */
    object ElevatorParameters {
        const val ELEVATOR_PID_LEFT_P: Double = 0.0001
        const val ELEVATOR_PID_LEFT_I: Double = 0.0
        const val ELEVATOR_PID_LEFT_D: Double = 0.0
        const val ELEVATOR_PID_LEFT_V: Double = 0.0

        const val ELEVATOR_PID_RIGHT_P: Double = 0.0001
        const val ELEVATOR_PID_RIGHT_I: Double = 0.0
        const val ELEVATOR_PID_RIGHT_D: Double = 0.0
        const val ELEVATOR_PID_RIGHT_V: Double = 0.0

        // Elevator Positions
        const val L1: Double = 1.0
        const val L2: Double = 2.0
        const val L3: Double = 3.0
        const val L4: Double = 4.0

        @JvmField
        var isSoftLimitEnabled: Boolean = false
    }

    /** Class containing constants for the pivot subsystem.  */
    object PivotParameters {
        const val PIVOT_PID_P: Double = 0.001
        const val PIVOT_PID_I: Double = 0.0
        const val PIVOT_PID_D: Double = 0.0
        const val PIVOT_PID_V: Double = 0.0

        @JvmField
        var isSoftLimitEnabled: Boolean = false
    }

    object EndEffectorParameters {
        const val END_EFFECTOR_PID_P: Double = 0.001
        const val END_EFFECTOR_PID_I: Double = 0.0
        const val END_EFFECTOR_PID_D: Double = 0.0
        const val END_EFFECTOR_PID_V: Double = 0.0

        @JvmField
        var isSoftLimitEnabled: Boolean = false
    }

    object CoralManipulatorParameters {
        const val CORAL_SENSOR_ID: Int = 8

        @JvmField
        val CORAL_MANIPULATOR_UP_PIDV: PIDVController = PIDVController(0.001, 0.0, 0.0, 0.0)

        @JvmField
        val CORAL_MANIPULATOR_DOWN_PIDV: PIDVController = PIDVController(0.001, 0.0, 0.0, 0.0)

        @JvmField
        var hasPiece: Boolean = false
    }
}
