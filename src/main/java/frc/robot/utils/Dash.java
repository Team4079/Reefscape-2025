package frc.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

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
   * Logs a test output with a fixed value to the Logger. This method is used for testing purposes
   * to ensure logging functionality.
   */
  public static void test() {
    Logger.recordOutput("TEST", 1);
  }
}
