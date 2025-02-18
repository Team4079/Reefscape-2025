package frc.robot.utils.pingu;

import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.*;

import static edu.wpi.first.wpilibj.Alert.AlertType.kError;

/**
 * A subsystem for aggregating Alerts
 */
public class AlertPingu extends SubsystemBase {
  private final ArrayList<TalonFX> motors = new ArrayList<>();
  private final ArrayList<CANcoder> canCoders = new ArrayList<>();
  private final ArrayList<Alert> motorAlerts = new ArrayList<>();
  private final ArrayList<Alert> canCoderAlerts = new ArrayList<>();

  /**
   * The Singleton instance of this AlertPingu. Code should use the {@link #getInstance()} method to
   * get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final AlertPingu INSTANCE = new AlertPingu();

  /**
   * Creates a new instance of this AlertPingu. This constructor is private since this class is a
   * Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private AlertPingu() {}

  /**
   * Returns the Singleton instance of this AlertPingu. This static method should be used, rather
   * than the constructor, to get the single instance of this class. For example: {@code
   * AlertPingu.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static AlertPingu getInstance() {
    return INSTANCE;
  }

  /**
   * Automatically periodically set the alerts to each of the motors
   */
  @Override
  public void periodic() {
    for (int i = 0; i < motors.size(); i++) {
      motorAlerts.get(i).set(!motors.get(i).isConnected());
    }
    for (int i = 0; i < canCoders.size(); i++) {
      canCoderAlerts.get(i).set(!canCoders.get(i).isConnected());
    }
  }

  /** Add a motor to the {@link AlertPingu} subsystem
   *
   * @param motor The motor to add
   * @param motorName The name of the motor being added
   */
  public void add(TalonFX motor, String motorName) {
    motors.add(motor);
    motorAlerts.add(new Alert("Disconnected " + motorName + " motor " + motor.getDeviceID(), kError));
  }

  /** Add a canCoder to the {@link AlertPingu} subsystem
   *
   * @param canCoder The canCoder to add
   * @param canCoderName The name of the canCoder being added
   */
  public void add(CANcoder canCoder) {
    canCoders.add(canCoder);
    canCoderAlerts.add(new Alert("Disconnected canCoder " + canCoder.getDeviceID(), kError));
  }
}
