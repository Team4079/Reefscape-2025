package frc.robot.utils.controller;

import static frc.robot.utils.controller.Axis.*;
import static frc.robot.utils.controller.Button.*;
import static frc.robot.utils.controller.Trigger.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/** A class representing a Logitech Gaming Pad. */
public class GamingController extends XboxController {
  private final Joystick gamepad;

  /**
   * Constructor for the LogitechGamingPad.
   *
   * @param usbPort The USB port the gamepad is connected to.
   */
  public GamingController(int usbPort) {
    super(usbPort);
    this.gamepad = new Joystick(usbPort);
  }

  /**
   * Gets the value of the left trigger.
   *
   * @return The value of the left trigger.
   */
  public double getLeftTriggerValue() {
    return gamepad.getRawAxis(LEFT);
  }

  /**
   * Gets the value of the right trigger.
   *
   * @return The value of the right trigger.
   */
  public double getRightTriggerValue() {
    return gamepad.getRawAxis(RIGHT);
  }

  /**
   * Gets the value of the left analog X axis.
   *
   * @return The value of the left analog X axis.
   */
  public double getLeftAnalogXAxis() {
    return gamepad.getRawAxis(LEFT_ANALOG_X);
  }

  /**
   * Gets the value of the left analog Y axis.
   *
   * @return The value of the left analog Y axis.
   */
  public double getLeftAnalogYAxis() {
    return gamepad.getRawAxis(LEFT_ANALOG_Y);
  }

  /**
   * Gets the value of the right analog X axis.
   *
   * @return The value of the right analog X axis.
   */
  public double getRightAnalogXAxis() {
    return gamepad.getRawAxis(RIGHT_ANALOG_X);
  }

  /**
   * Gets the value of the right analog Y axis.
   *
   * @return The value of the right analog Y axis.
   */
  public double getRightAnalogYAxis() {
    return gamepad.getRawAxis(RIGHT_ANALOG_Y);
  }

  @Override
  public boolean getAButton() {
    return gamepad.getRawButton(A);
  }

  @Override
  public boolean getBButton() {
    return gamepad.getRawButton(B);
  }

  @Override
  public boolean getXButton() {
    return gamepad.getRawButton(X);
  }

  @Override
  public boolean getYButton() {
    return gamepad.getRawButton(Y);
  }

  @Override
  public boolean getBackButton() {
    return gamepad.getRawButton(BACK);
  }

  @Override
  public boolean getStartButton() {
    return gamepad.getRawButton(START);
  }

  public boolean getDPadUp() {
    return checkDPad(0);
  }

  public boolean getDPadRight() {
    return checkDPad(2);
  }

  public boolean getDPadDown() {
    return checkDPad(4);
  }

  public boolean getDPadLeft() {
    return checkDPad(6);
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
}
