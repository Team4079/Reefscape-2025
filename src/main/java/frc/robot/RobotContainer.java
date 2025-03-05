package frc.robot;

import static frc.robot.commands.Kommand.*;
import static frc.robot.commands.sequencing.Sequences.*;
import static frc.robot.utils.emu.Button.*;
import static frc.robot.utils.emu.Direction.*;
import static frc.robot.utils.emu.ElevatorState.*;
import static frc.robot.utils.pingu.Bingu.*;

import com.pathplanner.lib.auto.AutoBuilder;
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

    new CommandPingu()
        .bind("ScoreL1", variableScore(L1))
        .bind("ScoreL2", variableScore(L2))
        .bind("ScoreL3", variableScore(L3))
        .bind("ScoreL4", variableScore(L4))
        .bind("ScoreL4Left", fullScoreAuto(LEFT, L4))
        .bind("ScoreL4Right", fullScoreAuto(RIGHT, L4))
        .bind("HasPieceFalse", hasPieceFalse())
        .bind("AlignLeft", new AlignToPoseAuto(LEFT).withTimeout(1.3))
        .bind("AlignRight", new AlignToPoseAuto(RIGHT).withTimeout(1.3))
        .bind("ScoreCoralLeft", scoreCoralAuto(LEFT).withTimeout(1.3))
        .bind("ScoreCoralRight", scoreCoralAuto(RIGHT).withTimeout(1.3))
        .bind("MoveElevatorL4Auto", moveElevatorState(L4))
        .bind("ScoreL4Auto", variableScore(L4))
        .bind("scoreLeft", fullScore(LEFT))
        .bind("scoreRight", fullScore(RIGHT))
        .bind("SetL1", setElevatorState(L1))
        .bind("SetL2", setElevatorState(L2))
        .bind("SetL3", setElevatorState(L3))
        .bind("SetL4", setElevatorState(L4));

    networkChooser = AutoBuilder.buildAutoChooser();

    configureBindings();

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

    bindings(calamityCow, bind(A, Kommand::toggleVisionKillSwitch));
  }
}
