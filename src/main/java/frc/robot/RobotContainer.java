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
    Algae.getInstance();
    Swerve.getInstance().setDefaultCommand(drive(aacrn));

    new CommandPingu()
        .bind("ScoreL4Left", fullScoreAuto(LEFT))
        .bind("ScoreL4Right", fullScoreAuto(RIGHT))
        .bind("HasPieceFalse", hasPieceFalse())
        .bind("MoveElevatorL4Auto", moveElevatorState(L4))
        .bind("MoveElevatorDefaultAuto", moveElevatorState(DEFAULT))
        .bind("SetL1", setElevatorState(L1))
        .bind("SetL2", setElevatorState(L2))
        .bind("SetL3", setElevatorState(L3))
        .bind("SetL4", setElevatorState(L4))
        .bind("MoveElevatorDown", setElevatorState(DEFAULT));

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
         bind(A, Kommand::setIntakeAlgae),
        // bind(A, () -> align(RIGHT)),
        bind(Y, Kommand::startCoralMotors),
        bind(X, () -> reverseIntake().onlyWhile(aacrn::getXButton)),
        bind(RIGHT_BUMPER, () -> fullScore(RIGHT)),
        bind(LEFT_BUMPER, () -> fullScore(LEFT)));

    bindings(calamityCow, bind(A, Kommand::offVision));
    bindings(calamityCow, bind(B, Kommand::onVision));
  }
}
