// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequencing;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AlignLeft;
import frc.robot.commands.elevator.MoveToLevel;
import frc.robot.commands.elevator.SetL1;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreRight extends SequentialCommandGroup {
  public ScoreRight() {
    addCommands(
      new ParallelCommandGroup(
        new AlignLeft() // Align the robot to the april tag (and add an offset)
      ),
      new MoveToLevel(), // Move the elevator to the desired level
      // Pivot to branch
      // Reverse rollers
      // Stop rollers
      new SetL1(),
      new MoveToLevel() // Move the elevator back to L1

    );
  }
}
