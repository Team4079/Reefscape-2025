// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.*;
import frc.robot.commands.PadDrive;
import frc.robot.subsystems.*;
import frc.robot.utils.LogitechGamingPad;
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

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    LogitechGamingPad pad = new LogitechGamingPad(0);
    padA = new JoystickButton(pad, 1);
    padB = new JoystickButton(pad, 2);
    padX = new JoystickButton(pad, 3);
    padY = new JoystickButton(pad, 4);

    SwerveSubsystem.getInstance()
        .setDefaultCommand(new PadDrive(pad, Thresholds.IS_FIELD_ORIENTED));

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger} or our {@link JoystickButton} constructor with an arbitrary predicate, or via
   * the named factories in {@link CommandGenericHID}'s subclasses for {@link
   * CommandXboxController}/{@link CommandPS4Controller} controllers or {@link CommandJoystick}.
   */
  private void configureBindings() {
    // padA.onTrue(new InstantCommand(SwerveSubsystem.getInstance()::addRotorPositionsforModules));
    padB.onTrue(new InstantCommand(SwerveSubsystem.getInstance()::resetPidgey));
    // padY.onTrue(new InstantCommand(SwerveSubsystem.getInstance()::configAAcornMode));
    // padX.onTrue(new InstantCommand(SwerveSubsystem.getInstance()::configSlowMode));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto(SwerveParameters.pathPlannerAutoName);
  }
}
