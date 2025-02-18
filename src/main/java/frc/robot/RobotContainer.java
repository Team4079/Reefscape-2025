package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;
import static frc.robot.utils.emu.Button.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;
import static frc.robot.utils.Register.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.subsystems.*;
import frc.robot.utils.*;
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

    Elevator.getInstance().setDefaultCommand(padElevator(aacrn));
    Coral.getInstance();
    Swerve.getInstance().setDefaultCommand(drive(aacrn));
    AlertPingu.INSTANCE.getSubsystem();

    Button.getEntries()
        .forEach(button -> aacrnButtons.put(button, new JoystickButton(aacrn, button.getButtonNumber())));

    Button.getEntries()
        .forEach(button -> calamityCowButtons.put(button, new JoystickButton(calamityCow, button.getButtonNumber())));

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
    // For aacrn
    Register.bindings(
            aacrnButtons,
        bind(START, resetPidgey()),
        bind(B, setElevatorState(DEFAULT)),
        //        bind(B, align(CENTER).onlyWhile(pad::getAButton)),
        //        bind(B, align(LEFT)),
        bind(A, setIntakeAlgae()),
        bind(Y, startCoralMotors()),
        //        bind(A, align(RIGHT)),
        // TODO: PLEASE TEST
        //                bind(B, createPathfindingCmd(reefs.get(0))),
        bind(LEFT_BUMPER, score(LEFT, elevatorToBeSetState)),
        bind(RIGHT_BUMPER, score(RIGHT, elevatorToBeSetState)),
        bind(X, reverseIntake().onlyWhile(aacrn::getXButton)));

    // For calamityCow
    Register.bindings(
            calamityCowButtons,
        // THESE ARE PLACEHOLDERS!!! pls om do not delete for real
        // i dont wanna rewrite this when jayden needs an actual button
        bind(A, waitCmd(1)));
  }
}
