package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.Button.*;
import static frc.robot.utils.Direction.*;
import static frc.robot.utils.ElevatorState.*;
import static frc.robot.utils.Register.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.PadElevator;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
import java.util.EnumMap;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Map<Button, JoystickButton> buttons = new EnumMap<>(Button.class);

  public final SendableChooser<Command> networkChooser;
  public final XboxController pad;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    pad = new XboxController(0);

    Button.getEntries()
        .forEach(button -> buttons.put(button, new JoystickButton(pad, button.getButtonNumber())));

    Swerve.getInstance().setDefaultCommand(drive(pad));
//    Elevator.getInstance().setDefaultCommand(padElevator(pad));
//    PhotonVision.getInstance();

    networkChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

    Register.commands(
        cmd("scoreLeft", score(LEFT)),
        cmd("scoreRight", score(RIGHT)),
        cmd("SetL1", setElevatorState(L1)),
        cmd("SetL2", setElevatorState(L2)),
        cmd("SetL3", setElevatorState(L3)),
        cmd("SetL4", setElevatorState(L4)));

    //    networkChooser.addDefaultOption("Straight Auto", new PathPlannerAuto("Straight Auto"));
//    networkChooser.addOption("Straight Auto", new InstantCommand());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link JoystickButton} constructor with an arbitrary predicate, or via the named factories in
   * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController}/{@link
   * CommandPS4Controller} controllers or {@link CommandJoystick}.
   */
  private void configureBindings() {
    Register.bindings(
        buttons,
        bind(START, resetPidgey()),
        bind(X, new InstantCommand(() -> Elevator.getInstance().applyElevatorPIDValues())),
        bind(B, align(CENTER).onlyWhile(pad::getAButton)),
//        bind(B, align(LEFT)),
//        bind(A, align(RIGHT)),
        // TODO PLEASE TEST
        //        bind(B, createPathfindingCmd(reefs.get(0))),
         bind(A, setElevatorState(DEFAULT)),
        // bind(B, setElevatorState(L2)),
        // bind(X, setElevatorState(L3)),
         bind(Y, setElevatorState(L4)),
        bind(LEFT_BUMPER, score(LEFT)),
        bind(RIGHT_BUMPER, score(RIGHT)));
  }
}
