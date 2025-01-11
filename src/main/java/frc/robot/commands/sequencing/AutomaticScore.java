// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignSwerve;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.*;
import frc.robot.utils.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutomaticScore extends SequentialCommandGroup {
  public AutomaticScore(Direction offsetSide) {
    addCommands(
        new AlignSwerve(offsetSide), // Align the robot to the april tag (and add an offset)
        new InstantCommand(() -> Elevator.getInstance().moveElevatorToLevel()),
        // Reverse rollers
        // Stop rollers
        new InstantCommand(() -> Elevator.getInstance().setState(ElevatorState.L1)),
        new InstantCommand(
            () -> Elevator.getInstance().moveElevatorToLevel()) // Move the elevator back to L1
        );
  }
}
