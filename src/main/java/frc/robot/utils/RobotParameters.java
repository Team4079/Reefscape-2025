package frc.robot.utils;

import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.*; //TODO: Add the controller function for everything
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Class containing global values for the robot. */
public class RobotParameters {
  /** Class containing global values related to motors. */
  public static class MotorParameters {
    // Motor CAN ID Values
    public static final int FRONT_LEFT_STEER_ID = 1;
    public static final int FRONT_LEFT_DRIVE_ID = 2;
    public static final int FRONT_RIGHT_STEER_ID = 3;
    public static final int FRONT_RIGHT_DRIVE_ID = 4;
    public static final int BACK_LEFT_STEER_ID = 5;
    public static final int BACK_LEFT_DRIVE_ID = 6;
    public static final int BACK_RIGHT_STEER_ID = 7;
    public static final int BACK_RIGHT_DRIVE_ID = 8;
    public static final int FRONT_LEFT_CAN_CODER_ID = 9;
    public static final int FRONT_RIGHT_CAN_CODER_ID = 10;
    public static final int BACK_LEFT_CAN_CODER_ID = 11;
    public static final int BACK_RIGHT_CAN_CODER_ID = 12;
    public static final int ELEVATOR_MOTOR_LEFT_ID = 13;
    public static final int ELEVATOR_MOTOR_RIGHT_ID = 14;
    public static final int PIVOT_MOTOR_ID = 15;
    public static final int PIDGEY_ID = 16;
    public static final int END_EFFECTOR_MOTOR_ID = 17;

    // Motor Property Values
    public static final double MAX_SPEED = 5.76;
    public static final double MAX_ANGULAR_SPEED = (14 * Math.PI) / 3;
    public static final double ENCODER_COUNTS_PER_ROTATION = 1.0;
    public static final double STEER_MOTOR_GEAR_RATIO = 150.0 / 7;
    public static final double DRIVE_MOTOR_GEAR_RATIO = 5.9;
    public static final double WHEEL_DIAMETER = 0.106;
    // public static final double SPEED_CONSTANT = 0.6;
    // public static final double TURN_CONSTANT = 0.3;
    public static final double METERS_PER_REV = WHEEL_DIAMETER * Math.PI * 0.99;
    public static double HEADING = 0.0;

    // Limit Values
    public static final double DRIVE_SUPPLY_LIMIT = 45.0;
    public static final double DRIVE_STATOR_LIMIT = 80.0;
    public static final double STEER_SUPPLY_LIMIT = 30.0;
  }

  /** Class containing global values related to the swerve drive system. */
  public static class SwerveParameters {
    public static final String pathPlannerAutoName = "Straight Auto";

    /** Class containing PID constants for the swerve drive system. */
    public static class PIDParameters {
      public static final PID STEER_PID_TELE = new PID(15.0, 0.000, 0.1, 0.0);
      public static final PID STEER_PID_AUTO = new PID(15.0, 0.000, 0.1, 0.0);
      public static final PID DRIVE_PID_AUTO = new PID(7.0, 0.0, 0.00);
      public static final double DRIVE_PID_V_AUTO = 0.5;
      public static final PID DRIVE_PID_TELE = new PID(1.5, 0.0, 0.0);
      public static final double DRIVE_PID_V_TELE = 0.0;
      public static final PID ROTATIONAL_PID = new PID(0.2, 0.0, 0.0, 0.0);
      public static final PID Y_PID = new PID(0.2, 0.0, 0.0, 0.0);
      public static final PID DIST_PID = new PID(0.2, 0.0, 0.0, 0.0);
      public static final PID PASS_ROTATIONAL_PID = new PID(0.1, 0.0, 0.0, 0.0);

      public static PPHolonomicDriveController pathFollower =
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.00, 0.0), // translation
              new PIDConstants(5.0, 0.0, 0.0) // rotation
              );
      public static RobotConfig config = null;

      static {
        try {
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          throw new RuntimeException("Failed to load robot config", e);
        }
      }
    }

    /** Class containing physical dimensions and kinematics for the swerve drive system. */
    public static class PhysicalParameters {
      public static final double ROBOT_SIZE = 0.43105229381;
      public static final Translation2d FRONT_LEFT = new Translation2d(0.3048, 0.3048);
      public static final Translation2d FRONT_RIGHT = new Translation2d(0.3048, -0.3048);
      public static final Translation2d BACK_LEFT = new Translation2d(-0.3048, 0.3048);
      public static final Translation2d BACK_RIGHT = new Translation2d(-0.3048, -0.3048);
      public static final SwerveDriveKinematics kinematics =
          new SwerveDriveKinematics(FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT);
    }

    /** Class containing various thresholds and constants for the swerve drive system. */
    public static class Thresholds {
      public static final double STATE_SPEED_THRESHOLD = 0.05;
      public static final double CANCODER_VAL9 = -0.419189;
      public static final double CANCODER_VAL10 = -0.825928 - 0.5;
      public static final double CANCODER_VAL11 = -0.475098;
      public static final double CANCODER_VAL12 = -0.032959 + 0.5;
      public static final InvertedValue DRIVE_MOTOR_INVERTED =
          InvertedValue.CounterClockwise_Positive;
      public static final InvertedValue STEER_MOTOR_INVERTED = InvertedValue.Clockwise_Positive;
      public static final double JOYSTICK_DEADBAND = 0.05;
      public static final boolean USING_VISION = false;
      public static final boolean AUTO_ALIGN = false;
      public static final double MOTOR_DEADBAND = 0.05;
      public static final boolean IS_FIELD_ORIENTED = true;
      public static final boolean SHOULD_INVERT = false;
      public static final double ENCODER_OFFSET = (0 / 360.0);
      public static Pose2d currentPose = new Pose2d(0.0, 0.0, new Rotation2d(0.0));
      public static final double X_DEADZONE = 0.15 * 5.76;
      public static final double Y_DEADZONE = 0.15 * 5.76;
      public static final double OFF_BALANCE_ANGLE_THRESHOLD = 10.0;
      public static final double ON_BALANCE_ANGLE_THRESHOLD = 5.0;

      // Robot LED Constants
      public static final int[] TAN_LED = {255, 120, 20};
      public static final int[] RED_LED = {255, 0, 0};
      public static final int[] GREEN_LED = {0, 255, 0};
      public static final int[] ORANGE_LED = {255, 165, 0};
      public static final int[] PURPLE_LED = {160, 32, 240};
      public static final int[] HIGHTIDE_LED = {0, 182, 174};

      // Testing boolean for SmartDashboard (to not slow down the robot)
      public static boolean TEST_MODE = true;
    }
  }

  /** Class containing constants for the Photonvision subsystem. */
  public static class PhotonvisionConstants {
    public static final double CAMERA_ONE_HEIGHT_METER = 0.47;
    public static final double CAMERA_ONE_ANGLE_DEG = 33.0;
    public static final double OFFSET_TOWARD_MID_LEFT = -15.00;
    public static final double CAMERA_TWO_HEIGHT_METER = 0.61;
    public static final double CAMERA_TWO_ANGLE_DEG = 37.5;
    public static final double OFFSET_TOWARD_MID_RIGHT = 15.0;
  }

  /** Class containing constants for the elevator subsystem. */
  public static class ElevatorParameters {
    public static final double ELEVATOR_PID_LEFT_P = 0.0001;
    public static final double ELEVATOR_PID_LEFT_I = 0.0;
    public static final double ELEVATOR_PID_LEFT_D = 0.0;
    public static final double ELEVATOR_PID_LEFT_V = 0.0;

    public static final double ELEVATOR_PID_RIGHT_P = 0.0001;
    public static final double ELEVATOR_PID_RIGHT_I = 0.0;
    public static final double ELEVATOR_PID_RIGHT_D = 0.0;
    public static final double ELEVATOR_PID_RIGHT_V = 0.0;

    // TODO: Get elevator values for these once the mechanism is built and functioning
    // Elevator Positions
    public static final double L1 = 1.0;
    public static final double L2 = 2.0;
    public static final double L3 = 3.0;
    public static final double L4 = 4.0;
    //Brandon: Why are these static?

    public static boolean SOFT_LIMIT_ENABLED = false;
    public static boolean IS_SOFTLIMIT = false;
  }

  /** Class containing constants for the pivot subsystem. */
  public static class PivotParameters {
    public static final double PIVOT_PID_P = 0.001;
    public static final double PIVOT_PID_I = 0.0;
    public static final double PIVOT_PID_D = 0.0;
    public static final double PIVOT_PID_V = 0.0;

    public static boolean SOFT_LIMIT_ENABLED = false;
    public static boolean IS_SOFTLIMIT = false;
  }

  public static class EndEffectorParameters {
    public static final double END_EFFECTOR_PID_P = 0.001;
    public static final double END_EFFECTOR_PID_I = 0.0;
    public static final double END_EFFECTOR_PID_D = 0.0;
    public static final double END_EFFECTOR_PID_V = 0.0;

    public static boolean SOFT_LIMIT_ENABLED = false;
    public static boolean IS_SOFTLIMIT = false;
    //Brandon: Tf why are there two bruh
  }
}
