// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.PadDrive;
import frc.robot.commands.AlignSwerve.Direction;
import frc.robot.commands.sequencing.AutomaticScore;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.SwerveParameters;
import frc.robot.utils.RobotParameters.SwerveParameters.*;

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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LogitechGamingPad pad = new LogitechGamingPad(0);
    padStart = new JoystickButton(pad, 8);
    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);
    padLeftBumper = new JoystickButton(pad, 5);
    padRightBumper = new JoystickButton(pad, 6);

    Swerve.getInstance()
        .setDefaultCommand(new PadDrive(pad, Thresholds.IS_FIELD_ORIENTED));

    configureBindings();

    NamedCommands.registerCommand("scoreLeft", new AutomaticScore(Direction.LEFT));
    NamedCommands.registerCommand("scoreRight", new AutomaticScore(Direction.RIGHT));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger} or our {@link JoystickButton} constructor with an arbitrary predicate, or via
   * the named factories in {@link CommandGenericHID}'s subclasses for {@link
   * CommandXboxController}/{@link CommandPS4Controller} controllers or {@link CommandJoystick}.
   */
  private void configureBindings() {
    // padA.onTrue(new InstantCommand(SwerveSubsystem.getInstance()::addRotorPositionsforModules));
    padStart.onTrue(
        new InstantCommand(Swerve.getInstance()::resetPidgey)); // Prev Button: padB
    padY.onTrue(new InstantCommand(Swerve.getInstance()::setTelePID));
    // padX.onTrue(new InstantCommand(SwerveSubsystem.getInstance()::configSlowMode));

    padA.onTrue(new InstantCommand(() -> Elevator.getInstance().setState(ElevatorState.L1)));
    padB.onTrue(new InstantCommand(() -> Elevator.getInstance().setState(ElevatorState.L2)));
    padX.onTrue(new InstantCommand(() -> Elevator.getInstance().setState(ElevatorState.L3)));
    padY.onTrue(new InstantCommand(() -> Elevator.getInstance().setState(ElevatorState.L4)));
    // TODO: These are placeholders, please change them to the correct values when Aaron finalizes them
      padLeftBumper.onTrue(
        new InstantCommand(() -> new AutomaticScore(Direction.LEFT)));
    padRightBumper.onTrue(
        new InstantCommand(() -> new AutomaticScore(Direction.RIGHT)));
    }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(SwerveParameters.PATHPLANNER_AUTO_NAME);
  }
}
