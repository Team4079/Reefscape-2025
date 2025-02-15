package frc.robot.subsystems;

import static frc.robot.utils.Register.Dash.log;
import static frc.robot.utils.Register.Dash.logs;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LEDState;
import frc.robot.utils.RobotParameters.*;
import java.util.ArrayList;
import java.util.Random;

public class LED extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer ledBuffer;
  private int rainbowFirstHue = 0;
  public LEDState ledState = LEDState.RAINBOW_FLOW;
  private final Random rand = new Random();
  private final int[] brightnessLevels;
  private int position = 0;
  private boolean goingForward = true;
  private final Timer robonautLEDTimer = new Timer();

  private boolean laser = false;
  private ArrayList<Integer> laserPositions = new ArrayList<>();
  private final int laser_count = 10;
  private final int spacing = 5;

  /**
   * The Singleton instance of this LEDSubsystem. Code should use the {@link #getInstance()} method
   * to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final LED INSTANCE = new LED();

  /**
   * Returns the Singleton instance of this LEDSubsystem. This static method should be used, rather
   * than the constructor, to get the single instance of this class. For example: {@code
   * LEDSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static LED getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this LEDSubsystem. This constructor is private since this class is a
   * Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private LED() {
    leds = new AddressableLED(9);
    ledBuffer = new AddressableLEDBuffer(LEDValues.LED_COUNT);
    brightnessLevels = new int[LEDValues.LED_COUNT];
    leds.setLength(ledBuffer.getLength());
    leds.setData(ledBuffer);
    leds.start();

    robonautLEDTimer.start();

    for (int i = 0; i < laser_count; i++) {
      laserPositions.add(-i * spacing);
    }
  }

  /**
   * This method will be called once per scheduler run. Updates the LED pattern based on the robot
   * state.
   */
  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      ledState = LEDState.LASER;
    }

    if (DriverStation.isDisabled()) {
      ledState = LEDState.RAINBOW_FLOW;
    }

    if (DriverStation.isDisabled() && LiveRobotValues.lowBattery) {
      ledState = LEDState.TWINKLE;
    }

    switch (this.ledState) {
      case RAINBOW_FLOW:
        flowingRainbow();
        break;
      case HIGHTIDE_FLOW:
        highTideFlow();
        break;
      case TWINKLE:
        twinkle();
        break;
      case ROBONAUT:
        robonautsLED();
        break;
      case FUNNY_ROBONAUT:
        funnyRobonaunts();
        break;
      case LASER:
        laserbeam();
        break;
      default:
        setRed();
    }
  }

  /**
   * Sets the color for each of the LEDs based on RGB values.
   *
   * @param r (Red) Integer values between 0 - 255
   * @param g (Green) Integer values between 0 - 255
   * @param b (Blue) Integer values between 0 - 255
   */
  public void setRGB(int r, int g, int b) {

    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, r, g, b);
    }
    logs(
        () -> {
          log("LED Length", ledBuffer.getLength());
          log("LED Color Blue", ledBuffer.getLED(0).blue);
          log("LED Color Red", ledBuffer.getLED(0).red);
          log("LED Color Green", ledBuffer.getLED(0).green);
        });
    leds.setData(ledBuffer);
  }

  /**
   * Sets the color for each of the LEDs based on HSV values
   *
   * @param h (Hue) Integer values between 0 - 180
   * @param s (Saturation) Integer values between 0 - 255
   * @param v (Value) Integer values between 0 - 255
   */
  public void rainbowHSV(int h, int s, int v) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, h, s, v);
    }
    leds.setData(ledBuffer);
  }

  /** Sets the LED color to tan. */
  public void setTan() {
    setRGB(255, 122, 20);
  }

  /** Sets the LED color to red. */
  public void setRed() {
    setRGB(255, 0, 0);
  }

  /** Sets the LED color to green. */
  public void setGreen() {
    setRGB(0, 255, 0);
  }

  /**
   * Sets the LED color to orange. This is a specific shade of orange that is used for the LED
   * strip.
   */
  public void setOrange() {
    setRGB(255, 165, 0);
  }

  /** Sets the LED color to purple. */
  public void setPurpleColor() {
    setRGB(160, 32, 240);
  }

  /** Sets the LED color to high tide (a specific shade of blue-green). */
  public void setHighTide() {
    setRGB(0, 182, 174);
  }

  /**
   * Creates a flowing high tide effect on the LED strip. The effect is based on a sine wave pattern
   * that changes over time.
   */
  public void highTideFlow() {
    long currentTime = System.currentTimeMillis();
    int length = ledBuffer.getLength();

    final int waveSpeed = 30;
    final int waveWidth = 55;

    for (int i = 0; i < length; i++) {
      double wave =
          Math.sin((i + ((double) currentTime / waveSpeed)) % length * (2 * Math.PI / waveWidth));

      wave = (wave + 1) / 2;

      int r = (int) (wave * 0);
      int g = (int) (wave * 200);
      int b = (int) (wave * 50);

      ledBuffer.setRGB(i, r, g, b);
    }
    leds.setData(ledBuffer);
  }

  /** Creates a flowing rainbow effect. */
  public void flowingRainbow() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      int hue = (rainbowFirstHue + (i * 180 / ledBuffer.getLength())) % 180;
      ledBuffer.setHSV(i, hue, 255, 255);
    }

    rainbowFirstHue = rainbowFirstHue + 4 % 180;

    leds.setData(ledBuffer);
  }

  /** Creates a twinkle where lights flicker at random. */
  public void twinkle() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (rand.nextDouble() < 0.1) {
        brightnessLevels[i] = rand.nextInt(255);
      } else {
        brightnessLevels[i] = Math.max(0, brightnessLevels[i] - 10);
      }

      int hue = rand.nextInt(180);
      ledBuffer.setHSV(i, hue, 255, brightnessLevels[i]);
    }
    leds.setData(ledBuffer);
  }

  /** Creates a robonauts themed LED pattern courtesy of CalamityCow */
  public void robonautsLED() {
    if (robonautLEDTimer.advanceIfElapsed(0.1)) {
      for (int i = 0; i < ledBuffer.getLength(); i++) {
        int color = rand.nextInt(0, 100);

        if (color < 70) {
          ledBuffer.setRGB(i, 0, 0, 0);
        } else if (color < 85) {
          ledBuffer.setRGB(i, 244, 177, 25);
        } else if (color < 100) {
          ledBuffer.setRGB(i, 255, 255, 255);
        }

        leds.setData(ledBuffer);
      }
    }
  }

  /** Funny robonaunts lights */
  public void funnyRobonaunts() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i == position) {
        ledBuffer.setHSV(i, 45, 255, 255);
      } else {
        ledBuffer.setHSV(i, 45, 255, 50);
      }
    }

    if (goingForward) {
      position++;
      if (position >= ledBuffer.getLength() - 1) goingForward = false;
    } else {
      position--;
      if (position < 0) goingForward = true;
    }
    leds.setData(ledBuffer);
  }

  /** Laser effect for LED lights */
  public void laserbeam() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setHSV(i, 0, 255, 20);
    }

    for (int i = 0; i < laserPositions.size(); i++) {
      int pos = laserPositions.get(i);
      if (pos >= 0 && pos < ledBuffer.getLength()) {
        ledBuffer.setHSV(pos, 0, 255, 0);
      }
      laserPositions.set(i, pos + 2);

      if (laserPositions.get(i) >= ledBuffer.getLength()) {
        laserPositions.set(i, -rand.nextInt(spacing));
      }
    }
    leds.setData(ledBuffer);
  }
}
