package frc.robot;

import static frc.robot.utils.Direction.*;
import static frc.robot.utils.ElevatorState.*;
import static frc.robot.commands.Kommand.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.utils.RobotParameters.SwerveParameters.*;
import frc.robot.utils.controller.*;
import frc.robot.utils.controller.Trigger;
import org.littletonrobotics.junction.networktables.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final JoystickButton padA;
  private final JoystickButton padB;
  private final JoystickButton padX;
  private final JoystickButton padY;
  private final JoystickButton padStart;
  private final JoystickButton padLeftBumper;
  private final JoystickButton padRightBumper;

  public LoggedDashboardChooser<Command> networkChooser =
      new LoggedDashboardChooser<>("AutoChooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    GamingController pad = new GamingController(0);
    padStart = new JoystickButton(pad, 8);
    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    padLeftBumper = new JoystickButton(pad, 5);
    padRightBumper = new JoystickButton(pad, 6);

    Swerve.getInstance().setDefaultCommand(drive(pad, Thresholds.IS_FIELD_ORIENTED));

    configureBindings();

    NamedCommands.registerCommand("scoreLeft", score(LEFT));
    NamedCommands.registerCommand("scoreRight", score(RIGHT));
    NamedCommands.registerCommand("SetL1", setElevatorState(L1));
    NamedCommands.registerCommand("SetL2", setElevatorState(L2));
    NamedCommands.registerCommand("SetL3", setElevatorState(L3));
    NamedCommands.registerCommand("SetL4", setElevatorState(L4));

    networkChooser.addDefaultOption("Do Nothing", new PathPlannerAuto("Straight Auto"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger} or our {@link JoystickButton} constructor with an arbitrary predicate, or via
   * the named factories in {@link CommandGenericHID}'s subclasses for {@link
   * CommandXboxController}/{@link CommandPS4Controller} controllers or {@link CommandJoystick}.
   */

  // TODO: Remap bindings
  private void configureBindings() {
    padStart.onTrue(resetPidgey()); // Prev Button: padB
    padY.onTrue(setTelePid());
    // padA.onTrue(setElevatorState(L1));
    // padB.onTrue(setElevatorState(L2));
    // padX.onTrue(setElevatorState(L3));
    // padY.onTrue(setElevatorState(L4));
    padLeftBumper.onTrue(score(LEFT));
    padRightBumper.onTrue(score(RIGHT));
  }

  public Command getAutonomousCommand() {
    return networkChooser.get();
  }
}
