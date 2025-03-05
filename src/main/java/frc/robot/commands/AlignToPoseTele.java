package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import frc.robot.utils.emu.Direction;

public class AlignToPoseTele extends AlignToPose {
  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignToPoseTele(Direction offsetSide) {
    super(offsetSide);
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {
    currentPose = Swerve.getInstance().getPose2Dfrom3D();
    Swerve.getInstance()
        .setDriveSpeeds(
            xController.calculate(currentPose.getX()),
            yController.calculate(currentPose.getY()),
            rotationalController.calculate(currentPose.getRotation().getDegrees()),
            true);
    alignLogs();
  }
}
