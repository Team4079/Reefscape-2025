package frc.robot.utils

import com.ctre.phoenix6.signals.InvertedValue
import com.pathplanner.lib.config.RobotConfig
import com.pathplanner.lib.path.PathConstraints
import edu.wpi.first.apriltag.AprilTagFields
import edu.wpi.first.math.Matrix
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation2d.fromDegrees
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.SwerveDriveKinematics
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3
import edu.wpi.first.math.numbers.N4
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints
import edu.wpi.first.math.util.Units.degreesToRadians
import edu.wpi.first.units.Units.Feet
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.utils.emu.AlgaeCounter
import frc.robot.utils.emu.AlgaePivotState
import frc.robot.utils.emu.CoralState
import frc.robot.utils.emu.ElevatorState
import frc.robot.utils.pingu.CoralScore
import frc.robot.utils.pingu.LogPingu.metaLogs
import frc.robot.utils.pingu.MagicPingu
import frc.robot.utils.pingu.PathPingu
import frc.robot.utils.pingu.Pingu
import kotlin.math.PI
import kotlin.math.cos
import kotlin.math.sin

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
        const val STAR_FEEDER_ID: Int = 18
        const val CORAL_FEEDER_ID: Int = 19

        // Motor Property Values
        const val MAX_SPEED: Double = 5.76
        const val MAX_ANGULAR_SPEED: Double = (14 * PI) / 3
        const val STEER_MOTOR_GEAR_RATIO: Double = 150.0 / 7
        const val DRIVE_MOTOR_GEAR_RATIO: Double = 6.750000000000000
        private const val WHEEL_DIAMETER: Double = 0.106
        // new treads so i increased


        // TODO CALIBRATE WHEELS
        const val METERS_PER_REV: Double = WHEEL_DIAMETER * PI * 0.99

        // Limit Values
        const val DRIVE_SUPPLY_LIMIT: Double = 45.0
        const val DRIVE_STATOR_LIMIT: Double = 80.0
        const val STEER_SUPPLY_LIMIT: Double = 30.0
    }

    /** Class containing global values related to the swerve drive system.  */
    object SwerveParameters {
        /** Class containing PID constants for the swerve drive system.  */
        object PinguParameters {
            @JvmField
            val STEER_PINGU_TELE = Pingu(750.0, 5.000, 15.0, 0.0)

            @JvmField
            val STEER_PINGU_AUTO = Pingu(750.0, 5.000, 15.0, 0.0)

            @JvmField
            val DRIVE_PINGU_AUTO = Pingu(5.0, 0.0, 0.0, 0.7)

            @JvmField
            val DRIVE_PINGU_TELE = Pingu(5.0, 0.0, 0.0, 0.7)

            @JvmField
            val ROTATIONAL_PINGU = Pingu(0.15, 0.0, 0.01)

            @JvmField
            val Y_PINGU = Pingu(4.079, 0.0, 0.0)

            @JvmField
            val X_PINGU = Pingu(4.079, 0.0, 0.0)

            @JvmField
            val DIST_PINGU = Pingu(0.2910, 0.0, 0.0)

            @JvmField
            val PROFILE_CONSTRAINTS = Constraints(0.4, 0.4)

            // TODO remember to update path planner config values and measure everything (cameras etc)
            @JvmField
            val PATH_CONSTRAINTS: PathConstraints =
                PathConstraints(
                    5.0,
                    5.0,
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
//            val TEST_MODE: Boolean = !DriverStation.isFMSAttached()
            val TEST_MODE: Boolean = true
        }
    }

    /** CLass for robot values that change and affect the robot. */
    object LiveRobotValues {
        const val LOW_BATTERY_VOLTAGE: Double = 11.8

        // make this a supplier
        @JvmField
        var robotPos: Pose2d = Pose2d(0.0, 0.0, Rotation2d(0.0, 0.0))

        @JvmField
        var lowBattery: Boolean = false

        @JvmField
        var visionDead: Boolean = false
    }

    /** Class containing constants for the Photonvision subsystem.  */
    object PhotonVisionConstants {
        const val CAMERA_ONE_HEIGHT_METER: Double = 0.195
        const val CAMERA_ONE_ANGLE_DEG: Double = 33.0
        const val OFFSET_TOWARD_MID_LEFT: Double = -15.00
        const val CAMERA_TWO_HEIGHT_METER: Double = 0.61
        const val CAMERA_TWO_ANGLE_DEG: Double = 37.5
        const val OFFSET_TOWARD_MID_RIGHT: Double = 15.0

        const val LEFT_OFFSET: Double = 0.163
        const val RIGHT_OFFSET: Double = -0.163

        // THESE NEED TO BE REPLACED WITH TESTED VALUES PLS (BUT I KNOW WE WON'T HAVE TIME FOR THIS)
        @JvmField
        val SINGLE_TARGET_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.08, 0.08, 0.05)

        @JvmField
        val MULTI_TARGET_STD_DEV: Matrix<N3, N1> = VecBuilder.fill(0.05, 0.05, 0.03)

        @JvmField
        val SINGLE_TARGET_STD_DEV_3D: Matrix<N4, N1> = VecBuilder.fill(0.08, 0.08, 0.08, 0.05)

        @JvmField
        val MULTI_TARGET_STD_DEV_3D: Matrix<N4, N1> = VecBuilder.fill(0.05, 0.05, 0.05, 0.03)
    }

    /** Class containing constants for the elevator subsystem.  */
    object ElevatorParameters {
        @JvmField
        val ELEVATOR_PINGU: Pingu = Pingu(5.0, 0.0, 0.0, 0.35, 0.5199, 0.42) // g could be 0.42

        // MM ↓

        @JvmField
        val ELEVATOR_MAGIC_PINGU = MagicPingu(90.0, 180.0, 0.0)

        const val ELEVATOR_SOFT_LIMIT_DOWN: Double = 0.0

        const val ELEVATOR_SOFT_LIMIT_UP: Double = 65.0

        @JvmField
        var elevatorToBeSetState: ElevatorState = ElevatorState.L4

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
        @JvmField val ALGAE_PINGU = Pingu(8.033, 0.0, 0.0, 0.0)

        @JvmField
        var isSoftLimitEnabled: Boolean = false

        @JvmField
        var algaePivotState: AlgaePivotState = AlgaePivotState.UP

        @JvmField
        var algaeIntaking: Boolean = false

        @JvmField
        var algaeCounter: AlgaeCounter = AlgaeCounter.INTAKE
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
        var coralScoring: Boolean = false
        // coral should be intaking and coral state should be intaking

//        @JvmField
//        var isCoralIntaking: Boolean = false
    }

    object FieldParameters {
        const val FIELD_LENGTH_METERS = 17.5483 // 57 feet + 6 7/8 inches
        const val FIELD_WIDTH_METERS = 8.0518 // 26 feet + 5 inches

        val AprilTagFieldLayout = AprilTagFields.k2025ReefscapeWelded

        val FIELD_LENGTH: Distance = Feet.of(57.0).plus(Inches.of(6.0 + 7.0 / 8.0))
        val FIELD_WIDTH: Distance = Feet.of(26.0).plus(Inches.of(5.0))

        @Suppress("MemberVisibilityCanBePrivate")
        object RobotPoses {
            // Red first then blue poses

            // Represents how far we want to go from the pole
            // TODO make for blue and red
            private const val DIS = -0.02

            val REEF_A = Pose2d(3.171, 4.189, fromDegrees(0.0))
            val REEF_B = Pose2d(3.171, 3.863, fromDegrees(0.0))
            val REEF_C = Pose2d(3.688, 2.968, fromDegrees(60.0))
            val REEF_D = Pose2d(3.975, 2.803, fromDegrees(60.0))
            val REEF_E = Pose2d(5.001, 2.804, fromDegrees(120.0))
            val REEF_F = Pose2d(5.285, 2.964, fromDegrees(120.0))
            val REEF_G = Pose2d(5.805, 3.863, fromDegrees(180.0))
            val REEF_H = Pose2d(5.805, 4.189, fromDegrees(180.0))
            val REEF_I = Pose2d(5.288, 5.083, fromDegrees(-120.0))
            val REEF_J = Pose2d(5.002, 5.248, fromDegrees(-120.0))
            val REEF_K = Pose2d(3.972, 5.247, fromDegrees(-60.0))
            val REEF_L = Pose2d(3.693, 5.079, fromDegrees(-60.0))

            val SCORING_A_BLUE =
                Pose2d(REEF_A.x - (DIS * cos(REEF_A.rotation.radians)), REEF_A.y - (DIS * sin(REEF_A.rotation.radians)), REEF_A.rotation)
            val SCORING_B_BLUE =
                Pose2d(REEF_B.x - (DIS * cos(REEF_B.rotation.radians)), REEF_B.y - (DIS * sin(REEF_B.rotation.radians)), REEF_B.rotation)
            val SCORING_C_BLUE =
                Pose2d(REEF_C.x - (DIS * cos(REEF_C.rotation.radians)), REEF_C.y - (DIS * sin(REEF_C.rotation.radians)), REEF_C.rotation)
            val SCORING_D_BLUE =
                Pose2d(REEF_D.x - (DIS * cos(REEF_D.rotation.radians)), REEF_D.y - (DIS * sin(REEF_D.rotation.radians)), REEF_D.rotation)
            val SCORING_E_BLUE =
                Pose2d(REEF_E.x - (DIS * cos(REEF_E.rotation.radians)), REEF_E.y - (DIS * sin(REEF_E.rotation.radians)), REEF_E.rotation)
            val SCORING_F_BLUE =
                Pose2d(REEF_F.x - (DIS * cos(REEF_F.rotation.radians)), REEF_F.y - (DIS * sin(REEF_F.rotation.radians)), REEF_F.rotation)
            val SCORING_G_BLUE =
                Pose2d(REEF_G.x - (DIS * cos(REEF_G.rotation.radians)), REEF_G.y - (DIS * sin(REEF_G.rotation.radians)), REEF_G.rotation)
            val SCORING_H_BLUE =
                Pose2d(REEF_H.x - (DIS * cos(REEF_H.rotation.radians)), REEF_H.y - (DIS * sin(REEF_H.rotation.radians)), REEF_H.rotation)
            val SCORING_I_BLUE =
                Pose2d(REEF_I.x - (DIS * cos(REEF_I.rotation.radians)), REEF_I.y - (DIS * sin(REEF_I.rotation.radians)), REEF_I.rotation)
            val SCORING_J_BLUE =
                Pose2d(REEF_J.x - (DIS * cos(REEF_J.rotation.radians)), REEF_J.y - (DIS * sin(REEF_J.rotation.radians)), REEF_J.rotation)
            val SCORING_K_BLUE =
                Pose2d(REEF_K.x - (DIS * cos(REEF_K.rotation.radians)), REEF_K.y - (DIS * sin(REEF_K.rotation.radians)), REEF_K.rotation)
            val SCORING_L_BLUE =
                Pose2d(REEF_L.x - (DIS * cos(REEF_L.rotation.radians)), REEF_L.y - (DIS * sin(REEF_L.rotation.radians)), REEF_L.rotation)

            val SCORING_A_BLUE_NOT_L4 =
                Pose2d(REEF_A.x, REEF_A.y, REEF_A.rotation)
            val SCORING_B_BLUE_NOT_L4 =
                Pose2d(REEF_B.x, REEF_B.y, REEF_B.rotation)
            val SCORING_C_BLUE_NOT_L4 =
                Pose2d(REEF_C.x, REEF_C.y, REEF_C.rotation)
            val SCORING_D_BLUE_NOT_L4 =
                Pose2d(REEF_D.x, REEF_D.y, REEF_D.rotation)
            val SCORING_E_BLUE_NOT_L4 =
                Pose2d(REEF_E.x, REEF_E.y, REEF_E.rotation)
            val SCORING_F_BLUE_NOT_L4 =
                Pose2d(REEF_F.x, REEF_F.y, REEF_F.rotation)
            val SCORING_G_BLUE_NOT_L4 =
                Pose2d(REEF_G.x, REEF_G.y, REEF_G.rotation)
            val SCORING_H_BLUE_NOT_L4 =
                Pose2d(REEF_H.x, REEF_H.y, REEF_H.rotation)
            val SCORING_I_BLUE_NOT_L4 =
                Pose2d(REEF_I.x, REEF_I.y, REEF_I.rotation)
            val SCORING_J_BLUE_NOT_L4 =
                Pose2d(REEF_J.x, REEF_J.y, REEF_J.rotation)
            val SCORING_K_BLUE_NOT_L4 =
                Pose2d(REEF_K.x, REEF_K.y, REEF_K.rotation)
            val SCORING_L_BLUE_NOT_L4 =
                Pose2d(REEF_L.x, REEF_L.y, REEF_L.rotation)

            // Tag order is 18, 19, 20, 21, 22, 17
            @JvmField
            val coralScoreBlueList: List<CoralScore> =
                listOf(
                    Triple(Translation2d(3.6576, 4.0259), SCORING_A_BLUE, SCORING_B_BLUE),
                    Triple(Translation2d(4.0739, 3.3063), SCORING_C_BLUE, SCORING_D_BLUE),
                    Triple(Translation2d(4.9047, 3.3063), SCORING_E_BLUE, SCORING_F_BLUE),
                    Triple(Translation2d(5.321046, 4.0259), SCORING_G_BLUE, SCORING_H_BLUE),
                    Triple(Translation2d(4.90474, 4.7455), SCORING_I_BLUE, SCORING_J_BLUE),
                    Triple(Translation2d(4.0739, 4.7455), SCORING_K_BLUE, SCORING_L_BLUE),
                )

            @JvmField
            val coralScoreBlueListNotL4: List<CoralScore> =
                listOf(
                    Triple(Translation2d(3.6576, 4.0259), SCORING_A_BLUE_NOT_L4, SCORING_B_BLUE_NOT_L4),
                    Triple(Translation2d(4.0739, 3.3063), SCORING_C_BLUE_NOT_L4, SCORING_D_BLUE_NOT_L4),
                    Triple(Translation2d(4.9047, 3.3063), SCORING_E_BLUE_NOT_L4, SCORING_F_BLUE_NOT_L4),
                    Triple(Translation2d(5.321046, 4.0259), SCORING_G_BLUE_NOT_L4, SCORING_H_BLUE_NOT_L4),
                    Triple(Translation2d(4.90474, 4.7455), SCORING_I_BLUE_NOT_L4, SCORING_J_BLUE_NOT_L4),
                    Triple(Translation2d(4.0739, 4.7455), SCORING_K_BLUE_NOT_L4, SCORING_L_BLUE_NOT_L4),
                )

            @JvmStatic
            fun getRedAlliancePose(bluePose: Pose2d): Pose2d =
                Pose2d(
                    FIELD_LENGTH_METERS - bluePose.x,
                    FIELD_WIDTH_METERS - bluePose.y,
                    bluePose.rotation.plus(Rotation2d.fromDegrees(180.0)),
                )

            @JvmStatic
            fun getRedAllianceTranslation(blueTranslation: Translation2d): Translation2d =
                Translation2d(
                    FIELD_LENGTH_METERS - blueTranslation.x,
                    FIELD_WIDTH_METERS - blueTranslation.y,
                )

            @JvmStatic
            fun addCoralPosList() {
                if (DriverStation.getAlliance().isPresent) {
                    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                        PathPingu.addCoralScoringPositions(coralScoreBlueList)
                        PathPingu.addCoralScoringPositionsNotL4(coralScoreBlueListNotL4)
                    } else {
                        val coralScoreRedList =
                            coralScoreBlueList.map {
                                Triple(getRedAllianceTranslation(it.first), getRedAlliancePose(it.second), getRedAlliancePose(it.third))
                            }
                        val coralScoreRedListNotL4 =
                            coralScoreBlueListNotL4.map {
                                Triple(getRedAllianceTranslation(it.first), getRedAlliancePose(it.second), getRedAlliancePose(it.third))
                            }
                        PathPingu.addCoralScoringPositions(coralScoreRedList)
                        PathPingu.addCoralScoringPositionsNotL4(coralScoreRedListNotL4)
                    }
                }
//                PathPingu.addCoralScoringPositions(coralScoreBlueList)
//                PathPingu.addCoralScoringPositionsNotL4(coralScoreBlueListNotL4)
            }

            // TODO convert to red, now we need to implement it so that we do not have to swap sides every deploy
            // REMEMBER EVERY DEPLOY CHANGE THE DRIVERSATION TO THE CORRECT COLOR

            // List of Source positions
            @JvmField
            val LEFT_CORAL_STATION_FAR: Pose2d = Pose2d(1.64, 7.33, fromDegrees(-54.5))

            @JvmField
            val LEFT_CORAL_STATION_NEAR: Pose2d = Pose2d(0.71, 6.68, fromDegrees(-54.5))

            @JvmField
            val RIGHT_CORAL_STATION_FAR: Pose2d = Pose2d(1.61, 0.70, fromDegrees(55.0))

            @JvmField
            val RIGHT_CORAL_STATION_NEAR: Pose2d = Pose2d(0.64, 1.37, fromDegrees(55.0))
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
