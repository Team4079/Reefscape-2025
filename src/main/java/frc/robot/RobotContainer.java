package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.Button.*;
import static frc.robot.utils.Direction.*;
import static frc.robot.utils.ElevatorState.*;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import java.util.EnumMap;
import java.util.Map;
import org.littletonrobotics.junction.networktables.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Map<Button, JoystickButton> buttons = new EnumMap<>(Button.class);

  public static final LoggedDashboardChooser<Command> networkChooser =
      new LoggedDashboardChooser<>("AutoChooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    XboxController pad = new XboxController(0);

    Button.getEntries()
        .forEach(button -> buttons.put(button, new JoystickButton(pad, button.getButtonNumber())));

    Swerve.getInstance().setDefaultCommand(drive(pad));

    configureBindings();

    NamedCommands.registerCommand("scoreLeft", score(LEFT, L4));
    NamedCommands.registerCommand("scoreRight", score(RIGHT, L4));
    NamedCommands.registerCommand("SetL1", setElevatorState(L1));
    NamedCommands.registerCommand("SetL2", setElevatorState(L2));
    NamedCommands.registerCommand("SetL3", setElevatorState(L3));
    NamedCommands.registerCommand("SetL4", setElevatorState(L4));

    networkChooser.addDefaultOption("Do Nothing", new PathPlannerAuto("Straight Auto"));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link JoystickButton} constructor with an arbitrary predicate, or via the named factories in
   * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController}/{@link
   * CommandPS4Controller} controllers or {@link CommandJoystick}.
   */
  private void configureBindings() { // TODO: Remap bindings
    buttons.get(START).onTrue(resetPidgey());
    buttons.get(Y).onTrue(setTelePid());
    // buttons.get(A).onTrue(setElevatorState(L1));
    // buttons.get(B).onTrue(setElevatorState(L2));
    // buttons.get(X).onTrue(setElevatorState(L3));
    // buttons.get(Y).onTrue(setElevatorState(L4));
    buttons.get(LEFT_BUMPER).onTrue(score(LEFT, L4));
    buttons.get(RIGHT_BUMPER).onTrue(score(RIGHT, L4));
  }

  public Command getAutonomousCommand() {
    return networkChooser.get();
  }
}
