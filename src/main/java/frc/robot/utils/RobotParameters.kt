package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue
import com.pathplanner.lib.config.PIDConstants
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.controllers.PPHolonomicDriveController
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.controller.PIDController
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation2d.fromDegrees
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.units.Units.Feet
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Distance
import frc.robot.utils.Register.Dash.metaLogs

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
        const val CLIMBER_MOTOR_ID: Int = 15
        const val PIDGEY_ID: Int = 16
        const val ALGAE_MANIPULATOR_MOTOR_ID: Int = 17
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

        const val AUTO_ALIGN_SWERVE_LEFT_OFFSET: Double = -0.1
        const val AUTO_ALIGN_SWERVE_RIGHT_OFFSET: Double = 0.1

        @JvmField
        var robotPos: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0, 0.0))

        /** Class containing PID constants for the swerve drive system.  */
        object PIDParameters {
            @JvmField
            val STEER_PID_TELE: PIDVController = PIDVController(750.0, 5.000, 15.0, 0.0)

            @JvmField
            val STEER_PID_AUTO: PIDVController = PIDVController(750.0, 5.000, 15.0, 0.0)

            @JvmField
            val DRIVE_PID_AUTO: PIDVController = PIDVController(5.0, 0.0, 0.0, 0.4)

            @JvmField
            val DRIVE_PID_TELE: PIDVController = PIDVController(5.0, 0.0, 0.0, 0.4)

            @JvmField
            val ROTATIONAL_PID: PIDController = PIDController(0.2, 0.0, 0.0)

            @JvmField
            val Y_PID: PIDController = PIDController(0.2, 0.0, 0.0)

            @JvmField
            val DIST_PID: PIDController = PIDController(0.2, 0.0, 0.0)

            var pathFollower: PPHolonomicDriveController =
                PPHolonomicDriveController(
                    PIDConstants(5.0, 0.00, 0.0), // translation
                    PIDConstants(5.0, 0.0, 0.0), // rotation
                )

            @JvmField
            val PATH_CONSTRAINTS: PathConstraints =
                PathConstraints(
                    3.0,
                    4.0,
                    degreesToRadians(540.0),
                    degreesToRadians(720.0),
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
            private val ROBOT_WIDTH: Double = 0.7112
            private val SWERVE_MODULE_OFFSET: Double = ROBOT_WIDTH / 2
            private val FRONT_LEFT: Translation2d = Translation2d(SWERVE_MODULE_OFFSET, SWERVE_MODULE_OFFSET)
            private val FRONT_RIGHT: Translation2d = Translation2d(SWERVE_MODULE_OFFSET, -SWERVE_MODULE_OFFSET)
            private val BACK_LEFT: Translation2d = Translation2d(-SWERVE_MODULE_OFFSET, SWERVE_MODULE_OFFSET)
            private val BACK_RIGHT: Translation2d = Translation2d(-SWERVE_MODULE_OFFSET, -SWERVE_MODULE_OFFSET)

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

        // THESE NEED TO BE REPLACED WITH TESTED VALUES PLS (BUT I KNOW WE WONT HAVE TIME FOR THIS)
        @JvmField
        val SINGLE_TARGET_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(1.0, 1.0, 10.0)

        @JvmField
        val MULTI_TARGET_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.3, 0.3, 3.0)
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

    /** Class containing constants for the CLIMBER subsystem.  */
    object ClimberParameters {
        const val CLIMBER_PID_P: Double = 0.001
        const val CLIMBER_PID_I: Double = 0.0
        const val CLIMBER_PID_D: Double = 0.0
        const val CLIMBER_PID_V: Double = 0.0

        @JvmField
        var isSoftLimitEnabled: Boolean = false
    }

    object AlgaeManipulatorParameters {
        const val ALGAE_MANIPULATOR_PID_P: Double = 0.001
        const val ALGAE_MANIPULATOR_PID_I: Double = 0.0
        const val ALGAE_MANIPULATOR_PID_D: Double = 0.0
        const val ALGAE_MANIPULATOR_PID_V: Double = 0.0

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

    object ConstField {
        val FIELD_LENGTH_METERS = 17.3744 // 57 feet + 6 7/8 inches
        val FIELD_WIDTH_METERS = 8.2296 // 26 feet + 5 inches

        val FIELD_LENGTH: Distance = Feet.of(57.0).plus(Inches.of(6.0 + 7.0 / 8.0))
        val FIELD_WIDTH: Distance = Feet.of(26.0).plus(Inches.of(5.0))

        /**
         * All poses on the field, defined by their location on the BLUE Alliance.
         */
        object Poses {
            // Blue Alliance Reef Poses (THESE VALUES ARE PROBABLY WRONG)
            private val REEF_A_BLUE = Pose2d(2.860, 4.187, fromDegrees(0.0))
            private val REEF_B_BLUE = Pose2d(2.860, 3.857, fromDegrees(0.0))
            private val REEF_C_BLUE = Pose2d(3.527, 2.694, fromDegrees(60.0))
            private val REEF_D_BLUE = Pose2d(3.813, 2.535, fromDegrees(60.0))
            private val REEF_E_BLUE = Pose2d(5.160, 2.529, fromDegrees(120.0))
            private val REEF_F_BLUE = Pose2d(5.445, 2.694, fromDegrees(120.0))
            private val REEF_G_BLUE = Pose2d(6.119, 3.857, fromDegrees(180.0))
            private val REEF_H_BLUE = Pose2d(6.119, 4.187, fromDegrees(180.0))
            private val REEF_I_BLUE = Pose2d(5.452, 5.343, fromDegrees(-120.0))
            private val REEF_J_BLUE = Pose2d(5.166, 5.527, fromDegrees(-120.0))
            private val REEF_K_BLUE = Pose2d(3.826, 5.508, fromDegrees(-60.0))
            private val REEF_L_BLUE = Pose2d(3.534, 5.368, fromDegrees(-60.0))

            // Red Alliance Reef Poses
            private val REEF_A_RED = Pose2d(14.5144, 4.0426, fromDegrees(180.0))
            private val REEF_B_RED = Pose2d(14.5144, 4.3726, fromDegrees(180.0))
            private val REEF_C_RED = Pose2d(13.8474, 5.5356, fromDegrees(240.0))
            private val REEF_D_RED = Pose2d(13.5614, 5.6946, fromDegrees(240.0))
            private val REEF_E_RED = Pose2d(12.2144, 5.7006, fromDegrees(300.0))
            private val REEF_F_RED = Pose2d(11.9294, 5.5356, fromDegrees(300.0))
            private val REEF_G_RED = Pose2d(11.2554, 4.3726, fromDegrees(0.0))
            private val REEF_H_RED = Pose2d(11.2554, 4.0426, fromDegrees(0.0))
            private val REEF_I_RED = Pose2d(11.9224, 2.8866, fromDegrees(60.0))
            private val REEF_J_RED = Pose2d(12.2084, 2.7026, fromDegrees(60.0))
            private val REEF_K_RED = Pose2d(13.5484, 2.7206, fromDegrees(120.0))
            private val REEF_L_RED = Pose2d(13.8404, 2.8606, fromDegrees(120.0))

            val BLUE_REEF_POSES =
                listOf(
                    REEF_A_BLUE,
                    REEF_B_BLUE,
                    REEF_C_BLUE,
                    REEF_D_BLUE,
                    REEF_E_BLUE,
                    REEF_F_BLUE,
                    REEF_G_BLUE,
                    REEF_H_BLUE,
                    REEF_I_BLUE,
                    REEF_J_BLUE,
                    REEF_K_BLUE,
                    REEF_L_BLUE,
                )

            val RED_REEF_POSES =
                listOf(
                    REEF_A_RED,
                    REEF_B_RED,
                    REEF_C_RED,
                    REEF_D_RED,
                    REEF_E_RED,
                    REEF_F_RED,
                    REEF_G_RED,
                    REEF_H_RED,
                    REEF_I_RED,
                    REEF_J_RED,
                    REEF_K_RED,
                    REEF_L_RED,
                )
        }
    }

    /** Yes I know Om you are gonna rename it */
    object Info {
        const val ROBOT_NAME: String = "Fridgebot Pro Max"
        const val TEAM_NUMBER: String = "4079"
        const val TEAM_NAME: String = "Quantum Leap"
        const val COMPETITION: String = "Build Season"

        @JvmStatic
        fun logInfo() {
            metaLogs("Robot Name", ROBOT_NAME)
            metaLogs("Team Number", TEAM_NUMBER)
            metaLogs("Team Name", TEAM_NAME)
            metaLogs("Competition", COMPETITION)
        }
    }

    object LED_Values {
        const val LED_COUNT: Int = 120
    }
}
