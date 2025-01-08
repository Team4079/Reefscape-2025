package frc.robot.utils;

import edu.wpi.first.math.Pair;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

public class Dash {

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

  public static double getDoubleValue(String key) {
    return SmartDashboard.getNumber(key, 0);
  }

  public static Pair<String, Object> pairOf(String s, Object o) {
    return new Pair<>(s, o);
  }

  public static void test() {
    Logger.recordOutput("TEST", 1);
  }
}