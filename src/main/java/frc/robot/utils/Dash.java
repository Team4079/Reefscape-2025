package frc.robot.utils;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds;
import kotlin.Pair;

public class Dash {
  /**
   * Function to update values from the SmartDashboard.
   *
   * @param pairs The pairs of keys and values to update.
   */
  @SafeVarargs
  public static void dash(Pair<String, Object>... pairs) {
    if (Thresholds.TEST_MODE) {
      for (Pair<String, Object> pair : pairs) {
        String key = pair.getFirst();
        Object value = pair.getSecond();
        if (value instanceof Number numberValue) {
          SmartDashboard.putNumber(key, numberValue.doubleValue());
        } else if (value instanceof Boolean booleanValue) {
          SmartDashboard.putBoolean(key, booleanValue);
        } else if (value instanceof String stringValue) {
          SmartDashboard.putString(key, stringValue);
        } else if (value instanceof Sendable sendableValue) {
          SmartDashboard.putData(key, sendableValue);
        } else {
          System.out.println("Oh great the dash function isn't working");
          throw new IllegalArgumentException("Unsupported type: " + value.getClass());
        }
      }
    }
  }

  /**
   * Function to update PIDV values from the SmartDashboard.
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

  public static Pair<String, Object> pairOf(String s, Object o) {
    return new Pair<>(s, o);
  }
}
