package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Algae extends SubsystemBase {
  /**
   * The Singleton instance of this AlgaeSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final Algae INSTANCE = new Algae();

  /**
   * Returns the Singleton instance of this AlgaeSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * AlgaeSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Algae getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this AlgaeSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private Algae() {
    // TODO: Set the default command, if any, for this subsystem by calling
    // setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the
    // subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }
}
