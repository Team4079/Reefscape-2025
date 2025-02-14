package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static frc.robot.utils.Register.Dash.log;
import static frc.robot.utils.Register.Dash.logs;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LEDState;
import frc.robot.utils.RobotParameters.*;

import java.util.Random;

public class LED extends SubsystemBase {
  private final AddressableLED leds;
  private final AddressableLEDBuffer addressableLEDBuffer;
  private int rainbowFirstHue = 0;
  public LEDState ledState = LEDState.RAINBOW_FLOW;
  private Random rand = new Random();
  private final int[] brightnessLevels;
  private final Timer robonautLEDTimer = new Timer();

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
    addressableLEDBuffer = new AddressableLEDBuffer(LEDValues.LED_COUNT);
    brightnessLevels = new int[LEDValues.LED_COUNT];
    leds.setLength(addressableLEDBuffer.getLength());
    leds.setData(addressableLEDBuffer);
    leds.start();

    robonautLEDTimer.start();
  }

  /**
   * This method will be called once per scheduler run. Updates the LED pattern based on the robot
   * state.
   */
  @Override
  public void periodic() {
    if (DriverStation.isAutonomousEnabled()) {
      ledState = LEDState.HIGHTIDE_FLOW;
    }

    if (DriverStation.isDisabled()) {
      ledState = LEDState.RAINBOW_FLOW;
    }

    switch(this.ledState) {
      case RAINBOW_FLOW:
        flowingRainbow();
        break;
      case HIGHTIDE_FLOW:
        highTideFlow();
        break;
      case ROBONAUT:
        robonautsLED();
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

    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      addressableLEDBuffer.setRGB(i, r, g, b);
    }
    logs(
        () -> {
          log("LED Length", addressableLEDBuffer.getLength());
          log("LED Color Blue", addressableLEDBuffer.getLED(0).blue);
          log("LED Color Red", addressableLEDBuffer.getLED(0).red);
          log("LED Color Green", addressableLEDBuffer.getLED(0).green);
        });
    leds.setData(addressableLEDBuffer);
  }

  /**
   * Sets the color for each of the LEDs based on HSV values
   *
   * @param h (Hue) Integer values between 0 - 180
   * @param s (Saturation) Integer values between 0 - 255
   * @param v (Value) Integer values between 0 - 255
   */
  public void rainbowHSV(int h, int s, int v) {
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      addressableLEDBuffer.setHSV(i, h, s, v);
    }
    leds.setData(addressableLEDBuffer);
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
    int length = addressableLEDBuffer.getLength();

    final int waveSpeed = 30;
    final int waveWidth = 55;

    for (int i = 0; i < length; i++) {
      double wave =
          Math.sin((i + ((double) currentTime / waveSpeed)) % length * (2 * Math.PI / waveWidth));

      wave = (wave + 1) / 2;

      int r = (int) (wave * 0);
      int g = (int) (wave * 200);
      int b = (int) (wave * 50);

      addressableLEDBuffer.setRGB(i, r, g, b);
    }
    leds.setData(addressableLEDBuffer);
  }

  /**
   * Creates a flowing rainbow effect.
   */
  public void flowingRainbow() {
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      int hue = (rainbowFirstHue + (i * 180 / addressableLEDBuffer.getLength())) % 180;
      addressableLEDBuffer.setHSV(i, hue, 255, 255);
    }

    rainbowFirstHue = rainbowFirstHue + 4 % 180;

    leds.setData(addressableLEDBuffer);
  }

  /**
   * Creates a twinkle where lights flicker at random.
   */
  public void twinkle() {
    for (int i = 0; i < addressableLEDBuffer.getLength(); i++) {
      if (rand.nextDouble() < 0.1) {
        brightnessLevels[i] = rand.nextInt(255);
      } else {
        brightnessLevels[i] = Math.max(0, brightnessLevels[i] - 10);
      }

      int hue = rand.nextInt(180);
      addressableLEDBuffer.setHSV(i, hue, 255, brightnessLevels[i]);
    }
    leds.setData(addressableLEDBuffer);
  }

  public void robonautsLED()
  {
    if (robonautLEDTimer.advanceIfElapsed(0.1))
    {
      for (int i = 0; i < addressableLEDBuffer.getLength(); i++)
      {
        int color = rand.nextInt(0, 100);
        if (color < 70)
        {
          //set leds to black
          addressableLEDBuffer.setRGB(i, 0, 0, 0);
        }

        else if (color >= 70 && color < 85)
        {
          addressableLEDBuffer.setRGB(i, 244, 177, 25);
        }

        else if (color >= 85 && color < 100)
        {
          addressableLEDBuffer.setRGB(i, 255, 255, 255);
        }

//      else if (color >= 45 && color < 50)
//      {
////        addressableLEDBuffer.setRGB(i, 207, 34, 43);
//        addressableLEDBuffer.setRGB(i, 0, 0, 0);
//      }

        leds.setData(addressableLEDBuffer);
      }
    }
  }
}
