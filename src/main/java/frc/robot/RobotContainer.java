package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.*;
import static frc.robot.utils.emu.Button.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.utils.emu.*;
import frc.robot.utils.pingu.*;
import java.util.EnumMap;
import java.util.Map;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Map<Button, JoystickButton> aacrnButtons = new EnumMap<>(Button.class);
  private final Map<Button, JoystickButton> calamityCowButtons = new EnumMap<>(Button.class);

  public final SendableChooser<Command> networkChooser;
  public final XboxController aacrn;
  public final XboxController calamityCow;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    aacrn = new XboxController(0);
    calamityCow = new XboxController(1);

    Elevator.getInstance().setDefaultCommand(padElevator(calamityCow));
    Coral.getInstance();
    Swerve.getInstance().setDefaultCommand(drive(aacrn));

    Button.getEntries()
        .forEach(
            button ->
                aacrnButtons.put(button, new JoystickButton(aacrn, button.getButtonNumber())));

    Button.getEntries()
        .forEach(
            button ->
                calamityCowButtons.put(
                    button, new JoystickButton(calamityCow, button.getButtonNumber())));

    networkChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

    new CommandPingu()
        .bind("scoreLeft", score(LEFT))
        .bind("scoreRight", score(RIGHT))
        .bind("SetL1", setElevatorState(L1))
        .bind("SetL2", setElevatorState(L2))
        .bind("SetL3", setElevatorState(L3))
        .bind("SetL4", setElevatorState(L4));

    //    networkChooser.addDefaultOption("Straight Auto", new PathPlannerAuto("Straight Auto"));
    //    networkChooser.addOption("Straight Auto", new InstantCommand());
  }

  /**
   * Use this method to define your trigger -> command mappings. Triggers can be created via the
   * {@link JoystickButton} constructor with an arbitrary predicate, or via the named factories in
   * {@link CommandGenericHID}'s subclasses for {@link CommandXboxController}/{@link
   * CommandPS4Controller} controllers or {@link CommandJoystick}.
   */
  private void configureBindings() {
    new Bingu(aacrnButtons)
        .bind(START, resetPidgey())
        .bind(B, setElevatorState(DEFAULT))
        //      .bind(B, align(CENTER).onlyWhile(pad::getAButton))
        //      .bind(B, align(LEFT))
        //      .bind(B, createPathfindingCmd(reefs.get(0)))
        .bind(A, setIntakeAlgae())
        //      .bind(A, align(RIGHT))
        .bind(Y, startCoralMotors())
        .bind(LEFT_BUMPER, score(LEFT, elevatorToBeSetState))
        .bind(RIGHT_BUMPER, score(RIGHT, elevatorToBeSetState))
        .bind(X, reverseIntake().onlyWhile(aacrn::getXButton));

    new Bingu(calamityCowButtons).bind(A, waitCmd(1));
  }
}
