package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.commands.sequencing.Sequences.*;
import static frc.robot.utils.emu.Button.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;
import static frc.robot.utils.pingu.Bingu.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.*;
import frc.robot.commands.sequencing.*;
import frc.robot.subsystems.*;
import frc.robot.utils.pingu.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  public final SendableChooser<Command> networkChooser;
  public final XboxController aacrn;
  public final XboxController calamityCow;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    aacrn = new XboxController(0);
    calamityCow = new XboxController(1);

    Elevator.getInstance().setDefaultCommand(padElevator(aacrn, calamityCow));
    Coral.getInstance();
    Swerve.getInstance().setDefaultCommand(drive(aacrn));
    Algae.getInstance();

    NamedCommands.registerCommand("ScoreL1", variableScore(L1));
    NamedCommands.registerCommand("ScoreL2", variableScore(L2));
    NamedCommands.registerCommand("ScoreL3", variableScore(L3));
    NamedCommands.registerCommand("ScoreL4", variableScore(L4));
    NamedCommands.registerCommand("Score4Left", fullScoreAuto(LEFT, L4));
    NamedCommands.registerCommand("Score4Right", fullScoreAuto(RIGHT, L4));

    // TODO add autoalign for auto

    NamedCommands.registerCommand("AlignLeft", new AlignToPose(LEFT).withTimeout(1.3));
    NamedCommands.registerCommand("AlignRight", new AlignToPose(RIGHT).withTimeout(1.3));

    networkChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

    new CommandPingu()
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
    bindings(
        aacrn,
        bind(START, Kommand::resetPidgey),
        // bind(B, () -> setElevatorState(DEFAULT)),
        // bind(B, () -> align(CENTER).onlyWhile(pad::getAButton)),
        bind(B, Sequences::resetScore),
        // bind(B, () -> createPathfindingCmd(reefs.get(0))),
        // bind(A, Kommand::setIntakeAlgae),
        // bind(A, () -> align(RIGHT)),
        bind(Y, Kommand::startCoralMotors),
        bind(X, () -> reverseIntake().onlyWhile(aacrn::getXButton)),
        bind(RIGHT_BUMPER, () -> fullScore(RIGHT)),
        bind(LEFT_BUMPER, () -> fullScore(LEFT)));

    bindings(calamityCow, bind(A, () -> waitFor(1)));
  }
}
