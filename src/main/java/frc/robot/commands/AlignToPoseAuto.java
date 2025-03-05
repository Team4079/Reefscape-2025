package frc.robot.commands;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.*;

import frc.robot.subsystems.Swerve;
import frc.robot.utils.emu.Direction;

public class AlignToPoseAuto extends AlignToPose {
  /**
   * Creates a new AlignSwerve using the Direction Enum.
   *
   * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
   *     "center".
   */
  public AlignToPoseAuto(Direction offsetSide) {
    super(offsetSide);
  }

  /**
   * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
   * called repeatedly until {@link #isFinished()}) returns true.
   */
  @Override
  public void execute() {
    Swerve swerve = Swerve.getInstance();
    currentPose = swerve.getPose2Dfrom3D();
    Swerve.getInstance()
        .setDriveSpeeds(
            xController.calculate(currentPose.getX()),
            yController.calculate(currentPose.getY()),
            rotationalController.calculate(currentPose.getRotation().getDegrees()),
            true);

    //    if (DriverStation.getAlliance().isPresent() &&
    // DriverStation.getAlliance().get().equals(Blue)) {
    //      if (targetPose.getX() < 4.5) {
    //        swerve.setDriveSpeeds(
    //            xController.calculate(currentPose.getX(), targetPose.getX()),
    //            yController.calculate(currentPose.getY(), targetPose.getY()),
    //            rotationalController.calculate(currentPose.getRotation().getDegrees()),
    //            false);
    //      } else {
    //        swerve.setDriveSpeeds(
    //            -xController.calculate(currentPose.getX(), targetPose.getX()),
    //            -yController.calculate(currentPose.getY(), targetPose.getY()),
    //            rotationalController.calculate(currentPose.getRotation().getDegrees()),
    //            false);
    //      }
    //    } else {
    //      // TODO: MAKE A COPY OF ALIGNTOPOSE FOR AUTO WITH FIELD CENTRIC TURE!@!!!!@
    //      if (targetPose.getX() < 13) {
    //        swerve.setDriveSpeeds(
    //            xController.calculate(currentPose.getX()),
    //            yController.calculate(currentPose.getY()),
    //            rotationalController.calculate(currentPose.getRotation().getDegrees()),
    //            false);
    //      } else {
    //        swerve.setDriveSpeeds(
    //            -xController.calculate(currentPose.getX()),
    //            -yController.calculate(currentPose.getY()),
    //            rotationalController.calculate(currentPose.getRotation().getDegrees()),
    //            false);
    //      }
    //    }
    alignLogs();
  }
}
