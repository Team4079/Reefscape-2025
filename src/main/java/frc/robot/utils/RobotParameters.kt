package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N4
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.units.Units.Feet
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
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
        const val CORAL_SCORE_ID: Int = 15
        const val PIDGEY_ID: Int = 16
        const val ALGAE_PIVOT_MOTOR_ID: Int = 17
        const val ALGAE_INTAKE_MOTOR_ID: Int = 18
        const val CORAL_FEEDER_ID: Int = 19

        // Motor Property Values
        const val MAX_SPEED: Double = 5.76
        const val MAX_ANGULAR_SPEED: Double = (14 * Math.PI) / 3
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

        // TODO: Placeholder for the offset amount, figure out the correct value
        const val AUTO_ALIGN_SWERVE_LEFT_OFFSET: Double = -0.1
        const val AUTO_ALIGN_SWERVE_RIGHT_OFFSET: Double = 0.1

        @JvmField
        var robotPos: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0, 0.0))

        /** Class containing PID constants for the swerve drive system.  */
        object PinguParameters {
            @JvmField
            val STEER_PINGU_TELE = Pingu(750.0, 5.000, 15.0, 0.0)

            @JvmField
            val STEER_PINGU_AUTO = Pingu(750.0, 5.000, 15.0, 0.0)

            @JvmField
            val DRIVE_PINGU_AUTO = Pingu(5.0, 0.0, 0.0, 0.4)

            @JvmField
            val DRIVE_PINGU_TELE = Pingu(5.0, 0.0, 0.0, 0.4)

            @JvmField
            val ROTATIONAL_PINGU = Pingu(0.2, 0.0, 0.0)

            @JvmField
            val Y_PINGU = Pingu(0.2, 0.0, 0.0)

            @JvmField
            val DIST_PINGU = Pingu(0.2, 0.0, 0.0)

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
            private const val ROBOT_WIDTH: Double = 0.7112
            private const val SWERVE_MODULE_OFFSET: Double = ROBOT_WIDTH / 2
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
            const val CANCODER_VAL9: Double = -0.419189
            const val CANCODER_VAL10: Double = -0.825928 - 0.5
            const val CANCODER_VAL11: Double = -0.475098
            const val CANCODER_VAL12: Double = -0.032959 + 0.5

            @JvmField
            val DRIVE_MOTOR_INVERTED: InvertedValue = InvertedValue.CounterClockwise_Positive

            @JvmField
            val STEER_MOTOR_INVERTED: InvertedValue = InvertedValue.Clockwise_Positive
            const val JOYSTICK_DEADBAND: Double = 0.05
            const val IS_FIELD_ORIENTED: Boolean = true
            const val SHOULD_INVERT: Boolean = false
            const val ENCODER_OFFSET: Double = (0 / 360.0)
            const val X_DEADZONE: Double = 0.1
            const val Y_DEADZONE: Double = 0.1

            // Testing boolean for logging (to not slow down the robot)
            val TEST_MODE: Boolean = !DriverStation.isFMSAttached()
        }
    }

    /** CLass for robot values that change and affect the robot. */
    object LiveRobotValues {
        const val LOW_BATTERY_VOLTAGE: Double = 11.8

        @JvmField
        var lowBattery: Boolean = false
    }

    /** Class containing constants for the Photonvision subsystem.  */
    object PhotonVisionConstants {
        const val CAMERA_ONE_HEIGHT_METER: Double = 0.13
        const val CAMERA_ONE_ANGLE_DEG: Double = 33.0
        const val OFFSET_TOWARD_MID_LEFT: Double = -15.00
        const val CAMERA_TWO_HEIGHT_METER: Double = 0.61
        const val CAMERA_TWO_ANGLE_DEG: Double = 37.5
        const val OFFSET_TOWARD_MID_RIGHT: Double = 15.0

        // THESE NEED TO BE REPLACED WITH TESTED VALUES PLS (BUT I KNOW WE WON'T HAVE TIME FOR THIS)
        @JvmField
        val SINGLE_TARGET_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(1.0, 1.0, 10.0)

        @JvmField
        val MULTI_TARGET_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.3, 0.3, 3.0)

        @JvmField
        val SINGLE_TARGET_STD_DEV_3D: Matrix<N4, N1> = VecBuilder.fill(1.0, 1.0, 10.0, 10.0)

        @JvmField
        val MULTI_TARGET_STD_DEV_3D: Matrix<N4, N1> = VecBuilder.fill(0.3, 0.3, 3.0, 3.0)
    }

    /** Class containing constants for the elevator subsystem.  */
    object ElevatorParameters {
        @JvmField
        val ELEVATOR_PINGU: Pingu = Pingu(5.0, 0.0, 0.0, 0.35, 0.5199, 0.0) // g could be 0.42

        // MM ↓

        @JvmField
        val ELEVATOR_MAGIC_PINGU = MagicPingu(90.0, 180.0, 0.0)

        const val ELEVATOR_SOFT_LIMIT_DOWN: Double = 0.0

        const val ELEVATOR_SOFT_LIMIT_UP: Double = 61.5

        @JvmField
        var elevatorSetState: ElevatorState = ElevatorState.DEFAULT

        @JvmField
        var elevatorToBeSetState: ElevatorState = ElevatorState.DEFAULT

        @JvmField
        var isSoftLimitEnabled: Boolean = false
    }

    /** Class containing constants for the CLIMBER subsystem.  */
    object ClimberParameters {
        @JvmField val CLIMBER_PINGU = Pingu(0.001, 0.0, 0.0, 0.0)

        @JvmField
        var isSoftLimitEnabled: Boolean = false
    }

    object AlgaeManipulatorParameters {
        @JvmField val ALGAE_PINGU = Pingu(2.0, 0.0, 0.0, 0.0)

        @JvmField
        var isSoftLimitEnabled: Boolean = false

        @JvmField
        var algaeState: AlgaeState = AlgaeState.UP

        @JvmField
        var algaeIntaking: Boolean = false

        @JvmField
        var algaeCounter: Int = 0
    }

    object CoralManipulatorParameters {
        const val CORAL_SENSOR_ID: Int = 0

        @JvmField
        val CORAL_FEEDER_PINGU = Pingu(0.001, 0.0, 0.0, 0.0)

        @JvmField
        var coralState: CoralState = CoralState.CORAL_INTAKE

        @JvmField
        var hasPiece: Boolean = false

        @JvmField
        var isCoralIntaking: Boolean = true
    }

    object FieldParameters {
        const val FIELD_LENGTH_METERS = 17.3744 // 57 feet + 6 7/8 inches
        const val FIELD_WIDTH_METERS = 8.2296 // 26 feet + 5 inches

        val FIELD_LENGTH: Distance = Feet.of(57.0).plus(Inches.of(6.0 + 7.0 / 8.0))
        val FIELD_WIDTH: Distance = Feet.of(26.0).plus(Inches.of(5.0))

        object RobotPoses {
            // Red first then blue poses
            @JvmField
            val reefs =
                listOf(
                    Pose2d(Translation2d(5.008, 5.279), Rotation2d.fromDegrees(-120.0)),
                    Pose2d(Translation2d(5.345, 5.12), Rotation2d.fromDegrees(-120.0)),
                    Pose2d(Translation2d(5.84, 4.084), Rotation2d.fromDegrees(180.0)),
                    Pose2d(Translation2d(5.84, 3.916), Rotation2d.fromDegrees(180.0)),
                    Pose2d(Translation2d(5.345, 2.88), Rotation2d.fromDegrees(120.0)),
                    Pose2d(Translation2d(5.008, 2.721), Rotation2d.fromDegrees(120.0)),
                    Pose2d(Translation2d(3.972, 2.721), Rotation2d.fromDegrees(60.0)),
                    Pose2d(Translation2d(3.635, 2.88), Rotation2d.fromDegrees(60.0)),
                    Pose2d(Translation2d(3.14, 3.916), Rotation2d.fromDegrees(0.0)),
                    Pose2d(Translation2d(3.14, 4.084), Rotation2d.fromDegrees(0.0)),
                    Pose2d(Translation2d(3.635, 5.12), Rotation2d.fromDegrees(-60.0)),
                    Pose2d(Translation2d(3.972, 5.279), Rotation2d.fromDegrees(-60.0)),
                )

            // List of Source positions
            @JvmField
            val sources =
                listOf(
                    Pose2d(Translation2d(1.582, 7.275), Rotation2d.fromDegrees(126.0)),
                    Pose2d(Translation2d(0.767, 6.692), Rotation2d.fromDegrees(126.0)),
                    Pose2d(Translation2d(0.767, 1.35), Rotation2d.fromDegrees(-126.0)),
                    Pose2d(Translation2d(1.582, 0.78), Rotation2d.fromDegrees(-126.0)),
                )
        }
    }

    /** Important external information */
    object Info {
        private const val ROBOT_NAME: String = "Nautilus"
        private const val TEAM_NUMBER: String = "4079"
        private const val TEAM_NAME: String = "Quantum Leap"
        private val COMPETITION: String = DriverStation.getEventName()
        private val MATCH_NUMBER: String = DriverStation.getMatchNumber().toString()
        private val ALLIANCE: String = DriverStation.getAlliance().toString()

        @JvmStatic
        fun logInfo() {
            metaLogs("Robot Name", ROBOT_NAME)
            metaLogs("Team Number", TEAM_NUMBER)
            metaLogs("Team Name", TEAM_NAME)
            metaLogs("Competition", COMPETITION)
            metaLogs("Match Number", MATCH_NUMBER)
            metaLogs("Alliance", ALLIANCE)
        }
    }

    object LEDValues {
        const val LED_COUNT: Int = 120
    }
}
