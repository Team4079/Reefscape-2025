package frc.robot.utils;

import edu.wpi.first.math.controller.*;

/**
 * The PIDVController class extends the PIDController class to include an additional velocity term
 * (kV) for more precise control.
 */
public class PIDVController extends PIDController {
  private double v;

  /**
   * Constructs a PIDVController with the specified PID constants and velocity term.
   *
   * @param kp The proportional gain.
   * @param ki The integral gain.
   * @param kd The derivative gain.
   * @param kV The velocity term.
   */
  public PIDVController(double kp, double ki, double kd, double kV) {
    super(kp, ki, kd);
    v = kV;
  }

  /**
   * Constructs a PIDVController by copying the PID constants from an existing PIDController and
   * adding a velocity term.
   *
   * @param controller The existing PIDController to copy.
   * @param kV The velocity term.
   */
  public PIDVController(PIDController controller, double kV) {
    super(controller.getP(), controller.getI(), controller.getD());
    v = kV;
  }

  /**
   * Gets the velocity term.
   *
   * @return The velocity term.
   */
  public double getV() {
    return v;
  }

  /**
   * Sets the velocity term.
   *
   * @param v The new velocity term.
   */
  public void setV(double v) {
    this.v = v;
  }
}
