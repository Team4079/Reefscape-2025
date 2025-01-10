package frc.robot.utils;

import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.TEST_MODE;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.*;

/**
 * Utility class for interacting with the SmartDashboard and logging. Provides methods to update PID
 * values, retrieve double values, create pairs, and perform test logging.
 */
public class Dash {
  private Dash() {}

  /**
   * Method to update PIDV values from the SmartDashboard.
   *
   * @param pid The PID object to update.
   * @param velocity The velocity to update.
   * @param prefix The prefix for the SmartDashboard keys.
   * @param changeV The function to change the velocity.
   */
  public static void dashPID(
      String prefix, PID pid, double velocity, java.util.function.DoubleConsumer changeV) {
    pid.setP(SmartDashboard.getNumber(prefix + " Auto P", pid.getP()));
    pid.setI(SmartDashboard.getNumber(prefix + " Auto I", pid.getI()));
    pid.setD(SmartDashboard.getNumber(prefix + " Auto D", pid.getD()));
    changeV.accept(SmartDashboard.getNumber(prefix + " Auto V", velocity));
  }

  /**
   * Logs a double value with a specified key if the system is in test mode.
   *
   * @param key The key associated with the value to log.
   * @param value The double value to log.
   */
  public static void log(String key, double value) {
    if (TEST_MODE) {
      Logger.recordOutput(key, value);
    }
  }

  /**
   * Logs an integer value with a specified key if the system is in test mode.
   *
   * @param key The key associated with the value to log.
   * @param value The integer value to log.
   */
  public static void log(String key, int value) {
    if (TEST_MODE) {
      Logger.recordOutput(key, value);
    }
  }

  /**
   * Logs a boolean value with a specified key if the system is in test mode.
   *
   * @param key The key associated with the value to log.
   * @param value The boolean value to log.
   */
  public static void log(String key, boolean value) {
    if (TEST_MODE) {
      Logger.recordOutput(key, value);
    }
  }

  /**
   * Logs a String value with a specified key if the system is in test mode.
   *
   * @param key The key associated with the value to log.
   * @param value The String value to log.
   */
  public static void log(String key, String value) {
    if (TEST_MODE) {
      Logger.recordOutput(key, value);
    }
  }

    /**
   * Logs a SwerveModuleState[] value with a specified key if the system is in test mode.
   *
   * @param key The key associated with the value to log.
   * @param value The SwerveModuleState[] value to log.
   */
  public static void log(String key, SwerveModuleState[] value) {
    if (TEST_MODE) {
      Logger.recordOutput(key, value);
    }
  }

  public static <T extends WPISerializable> void log(String key, T value) {
    if (TEST_MODE) {
      Logger.recordOutput(key, value);
    }
  }
}
