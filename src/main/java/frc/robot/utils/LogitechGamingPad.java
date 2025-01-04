package frc.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/** A class representing a Logitech Gaming Pad. */
public class LogitechGamingPad extends XboxController {
  private final Joystick gamepad;

  /** Enum representing the DPad directions. */
  public enum DPad {
    UP(0),
    UP_RIGHT(45),
    RIGHT(90),
    DOWN_RIGHT(135),
    DOWN(180),
    DOWN_LEFT(225),
    LEFT(270),
    UP_LEFT(315);

    public final int angle;

    DPad(int angle) {
      this.angle = angle;
    }
  }

  /** Enum representing the buttons on the gamepad. */
  public enum Button {
    A(1),
    B(2),
    X(3),
    Y(4),
    LEFT_BUMPER(5),
    RIGHT_BUMPER(6),
    BACK(7),
    START(8);

    public final int button;

    Button(int button) {
      this.button = button;
    }
  }

  /** Enum representing the axes on the gamepad. */
  public enum Axis {
    LEFT_ANALOG_X(0),
    LEFT_ANALOG_Y(1),
    RIGHT_ANALOG_X(4),
    RIGHT_ANALOG_Y(5);

    public final int axis;

    Axis(int axis) {
      this.axis = axis;
    }
  }

  /** Enum representing the triggers on the gamepad. */
  public enum Trigger {
    LEFT(2),
    RIGHT(3);

    public final int trigger;

    Trigger(int trigger) {
      this.trigger = trigger;
    }
  }

  /**
   * Constructor for the LogitechGamingPad.
   *
   * @param usbPort The USB port the gamepad is connected to.
   */
  public LogitechGamingPad(int usbPort) {
    super(usbPort);
    this.gamepad = new Joystick(usbPort);
  }

  /**
   * Gets the value of the left trigger.
   *
   * @return The value of the left trigger.
   */
  public double getLeftTriggerValue() {
    return gamepad.getRawAxis(Trigger.LEFT.trigger);
  }

  /**
   * Gets the value of the left analog X axis.
   *
   * @return The value of the left analog X axis.
   */
  public double getLeftAnalogXAxis() {
    return gamepad.getRawAxis(Axis.LEFT_ANALOG_X.axis);
  }

  /**
   * Gets the value of the left analog Y axis.
   *
   * @return The value of the left analog Y axis.
   */
  public double getLeftAnalogYAxis() {
    return gamepad.getRawAxis(Axis.LEFT_ANALOG_Y.axis);
  }

  /**
   * Gets the value of the right trigger.
   *
   * @return The value of the right trigger.
   */
  public double getRightTriggerValue() {
    return gamepad.getRawAxis(Trigger.RIGHT.trigger);
  }

  /**
   * Gets the value of the right analog X axis.
   *
   * @return The value of the right analog X axis.
   */
  public double getRightAnalogXAxis() {
    return gamepad.getRawAxis(Axis.RIGHT_ANALOG_X.axis);
  }

  /**
   * Gets the value of the right analog Y axis.
   *
   * @return The value of the right analog Y axis.
   */
  public double getRightAnalogYAxis() {
    return gamepad.getRawAxis(Axis.RIGHT_ANALOG_Y.axis);
  }

  @Override
  public boolean getAButton() {
    return gamepad.getRawButton(Button.A.button);
  }

  @Override
  public boolean getBButton() {
    return gamepad.getRawButton(Button.B.button);
  }

  @Override
  public boolean getXButton() {
    return gamepad.getRawButton(Button.X.button);
  }

  @Override
  public boolean getYButton() {
    return gamepad.getRawButton(Button.Y.button);
  }

  @Override
  public boolean getBackButton() {
    return gamepad.getRawButton(Button.BACK.button);
  }

  @Override
  public boolean getStartButton() {
    return gamepad.getRawButton(Button.START.button);
  }

  /**
   * Checks if the DPad is pressed in a specific direction.
   *
   * @param index The index of the DPad direction.
   * @return True if the DPad is pressed in the specified direction, false otherwise.
   */
  public boolean checkDPad(int index) {
    return 0 <= index && index <= 7 && (index * 45) == gamepad.getPOV();
  }

  /**
   * Checks if the DPad is pressed at a specific angle.
   *
   * @param angle The angle to check.
   * @param inDegrees Whether the angle is in degrees.
   * @return True if the DPad is pressed at the specified angle, false otherwise.
   */
  public boolean checkDPad(double angle, boolean inDegrees) {
    double angdeg = inDegrees ? angle : Math.toDegrees(angle);
    return (int) angdeg == gamepad.getPOV();
  }

  /**
   * Gets the DPad value.
   *
   * @return The DPad value.
   */
  public int getDPad() {
    int pov = gamepad.getPOV();
    return pov == -1 ? pov : pov / 45;
  }

  /**
   * Gets the DPad value in degrees or radians.
   *
   * @param inDegrees Whether to return the value in degrees.
   * @return The DPad value in degrees or radians.
   */
  public double getDPad(boolean inDegrees) {
    return inDegrees ? gamepad.getPOV() : Math.toRadians(gamepad.getPOV());
  }

  /**
   * Checks if the DPad is pressed.
   *
   * @return True if the DPad is pressed, false otherwise.
   */
  public boolean dPadIsPressed() {
    return gamepad.getPOV() != -1;
  }

  /**
   * Sets the rumble amount for the gamepad.
   *
   * @param amount The rumble amount.
   */
  public void setRumble(float amount) {
    gamepad.setRumble(RumbleType.kLeftRumble, amount);
    gamepad.setRumble(RumbleType.kRightRumble, amount);
  }

  @Override
  public double getRawAxis(int which) {
    return gamepad.getRawAxis(which);
  }

  @Override
  public boolean getRawButton(int button) {
    return gamepad.getRawButton(button);
  }

  @Override
  public int getPOV(int pov) {
    return gamepad.getPOV(pov);
  }

  /**
   * Gets the twist value.
   *
   * @return The twist value.
   */
  public double getTwist() {
    return 0.0;
  }

  /**
   * Gets the throttle value.
   *
   * @return The throttle value.
   */
  public double getThrottle() {
    return 0.0;
  }
}
