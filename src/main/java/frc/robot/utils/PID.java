package frc.robot.utils;

/** A class representing a PID controller. */
@SuppressWarnings("unused")
public class PID {
  private double p;
  private double i;
  private double d;
  private double f;
  private int s;

  private double previousError = 0;
  private double integral = 0;
  private double setpoint = 0;
  private double error = 0;
  private double output = 0;

  /**
   * Constructor for PID with P, I, D, F values and S value.
   *
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative coefficient.
   * @param f The feedforward coefficient.
   * @param s The S value.
   */
  public PID(double p, double i, double d, double f, int s) {
    updatePID(p, i, d, f, s);
  }

  /**
   * Constructor for PID with P, I, and D values.
   *
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative coefficient.
   */
  public PID(double p, double i, double d) {
    updatePID(p, i, d);
  }

  /**
   * Constructor for PID with P, I, D, and F values.
   *
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative coefficient.
   * @param f The feedforward coefficient.
   */
  public PID(double p, double i, double d, double f) {
    updatePID(p, i, d, f);
  }

  /**
   * Updates the PID coefficients and S value.
   *
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative coefficient.
   * @param f The feedforward coefficient.
   * @param s The S value.
   */
  public void updatePID(double p, double i, double d, double f, int s) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.f = f;
    this.s = s;
  }

  /**
   * Updates the PID coefficients.
   *
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative coefficient.
   * @param f The feedforward coefficient.
   */
  public void updatePID(double p, double i, double d, double f) {
    updatePID(p, i, d, f, -1);
  }

  /**
   * Updates the PID coefficients.
   *
   * @param p The proportional coefficient.
   * @param i The integral coefficient.
   * @param d The derivative coefficient.
   */
  public void updatePID(double p, double i, double d) {
    updatePID(p, i, d, 0);
  }

  /**
   * Calculates the PID output based on the actual value and setpoint.
   *
   * @param actual The actual value.
   * @param setpoint The target setpoint.
   * @return The PID output.
   */
  public double calculate(double actual, double setpoint) {
    this.error = setpoint - actual; // Error = Target - Actual
    this.integral +=
        (error
            * 0.02); // Integral is increased by the error*time (which is 0.02 seconds using normal
    double derivative = (error - this.previousError) / 0.02;
    this.output = p * error + i * this.integral + d * derivative;
    this.previousError = error;
    return output;
  }

  /** Resets the integral term. */
  public void resetI() {
    setI(0);
  }

  /**
   * Gets the proportional coefficient.
   *
   * @return The proportional coefficient.
   */
  public double getP() {
    return p;
  }

  /**
   * Sets the proportional coefficient.
   *
   * @param p The proportional coefficient.
   */
  public void setP(double p) {
    this.p = p;
  }

  /**
   * Gets the integral coefficient.
   *
   * @return The integral coefficient.
   */
  public double getI() {
    return i;
  }

  /**
   * Sets the integral coefficient.
   *
   * @param i The integral coefficient.
   */
  public void setI(double i) {
    this.i = i;
  }

  /**
   * Gets the derivative coefficient.
   *
   * @return The derivative coefficient.
   */
  public double getD() {
    return d;
  }

  /**
   * Sets the derivative coefficient.
   *
   * @param d The derivative coefficient.
   */
  public void setD(double d) {
    this.d = d;
  }

  /**
   * Gets the feedforward coefficient.
   *
   * @return The feedforward coefficient.
   */
  public double getF() {
    return f;
  }

  /**
   * Sets the feedforward coefficient.
   *
   * @param f The feedforward coefficient.
   */
  public void setF(double f) {
    this.f = f;
  }

  /**
   * Gets the S value.
   *
   * @return The S value.
   */
  public int getS() {
    return s;
  }

  /**
   * Sets the S value.
   *
   * @param s The S value.
   */
  public void setS(int s) {
    this.s = s;
  }

  /**
   * Gets the previous error.
   *
   * @return The previous error.
   */
  public double getPreviousError() {
    return previousError;
  }

  /**
   * Sets the previous error.
   *
   * @param previousError The previous error.
   */
  public void setPreviousError(double previousError) {
    this.previousError = previousError;
  }

  /**
   * Gets the integral term.
   *
   * @return The integral term.
   */
  public double getIntegral() {
    return integral;
  }

  /**
   * Sets the integral term.
   *
   * @param integral The integral term.
   */
  public void setIntegral(double integral) {
    this.integral = integral;
  }

  /**
   * Gets the setpoint.
   *
   * @return The setpoint.
   */
  public double getSetpoint() {
    return setpoint;
  }

  /**
   * Sets the setpoint.
   *
   * @param setpoint The setpoint.
   */
  public void setSetpoint(double setpoint) {
    this.setpoint = setpoint;
  }

  /**
   * Gets the current error.
   *
   * @return The current error.
   */
  public double getError() {
    return error;
  }

  /**
   * Sets the current error.
   *
   * @param error The current error.
   */
  public void setError(double error) {
    this.error = error;
  }

  /**
   * Gets the PID output.
   *
   * @return The PID output.
   */
  public double getOutput() {
    return output;
  }

  /**
   * Sets the PID output.
   *
   * @param output The PID output.
   */
  public void setOutput(double output) {
    this.output = output;
  }
}
