package frc.robot.commands;

import static frc.robot.commands.Kommand.moveToClosestCoralScore;
import static frc.robot.commands.Kommand.moveToClosestCoralScoreNotL4;
import static frc.robot.utils.RobotParameters.ElevatorParameters.elevatorToBeSetState;
import static frc.robot.utils.RobotParameters.FieldParameters.RobotPoses.addCoralPosList;
import static frc.robot.utils.RobotParameters.LiveRobotValues.visionDead;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;
import static frc.robot.utils.pingu.LogPingu.log;
import static frc.robot.utils.pingu.LogPingu.logs;
import static frc.robot.utils.pingu.PathPingu.clearCoralScoringPositions;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.emu.Direction;
import frc.robot.utils.emu.ElevatorState;
import frc.robot.utils.pingu.NetworkPingu;
import org.photonvision.PhotonCamera;

public class AlignToPoseAuto extends Command {
    private double yaw;
    private double y;
    private double dist;
    private PhotonVision photonVision;
    private ProfiledPIDController rotationalController;
    private ProfiledPIDController yController;
    private ProfiledPIDController xController;
    private Pose2d targetPose;
    private Pose2d currentPose;
    private Timer timer;
    private double
            offset; // double offset is the left/right offset from the april tag to make it properly align
    PhotonCamera camera;
    private Swerve swerve;
    private Direction offsetSide;

    private NetworkPingu networkPinguRotation;
    private NetworkPingu networkPinguY;
    private NetworkPingu networkPinguX;



    /**
     * Creates a new AlignSwerve using the Direction Enum.
     *
     * @param offsetSide The side of the robot to offset the alignment to. Can be "left", "right", or
     *     "center".
     */
    public AlignToPoseAuto(Direction offsetSide) {

        photonVision = PhotonVision.getInstance();
        swerve = Swerve.getInstance();
        this.offsetSide = offsetSide;
    }

    /** The initial subroutine of a command. Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        // Update the list of coral scoring positions to the correct side (hopefully)
        currentPose = swerve.getPose2Dfrom3D();

        timer = new Timer();
        addRequirements(swerve);

        clearCoralScoringPositions();
        addCoralPosList();
        currentPose = swerve.getPose2Dfrom3D();

        if (elevatorToBeSetState == ElevatorState.L4) {
            targetPose = moveToClosestCoralScore(offsetSide, Swerve.getInstance().getPose2Dfrom3D());
        } else {
            targetPose = moveToClosestCoralScoreNotL4(offsetSide, Swerve.getInstance().getPose2Dfrom3D());
        }

        xController = X_PINGU.getProfiledPIDController();
        xController.setTolerance(0.015);
        xController.setConstraints(PROFILE_CONSTRAINTS);
        xController.setGoal(targetPose.getX());
        xController.reset(currentPose.getX());

        yController = Y_PINGU.getProfiledPIDController();
        yController.setTolerance(0.015);
        yController.setConstraints(PROFILE_CONSTRAINTS);
        yController.setGoal(targetPose.getY());
        yController.reset(currentPose.getY());

        rotationalController = ROTATIONAL_PINGU.getProfiledPIDController();
        rotationalController.setTolerance(2.0);
        rotationalController.setConstraints(new TrapezoidProfile.Constraints(5, 5));
        rotationalController.setGoal(targetPose.getRotation().getDegrees());
        rotationalController.reset(currentPose.getRotation().getDegrees());
        rotationalController.enableContinuousInput(-180, 180);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled. (That is, it is
     * called repeatedly until {@link #isFinished()}) returns true.
     */
    @Override
    public void execute() {
        currentPose = swerve.getPose2Dfrom3D();

        //TODO PLS CHECK BOTH SIDES AND BOTH APRIL TAG SIDES AND MAKE SURE IT ACTUALLY ALIGNS -SHAWN
        //TODO when aligning the x and y axes, you need to add sin/cos of the other axes angles and feed it into the PID (probably need to test)
        //TODO the swerve should not need a negative anymore and we should not even have to check poses theoretically??????? TEST THIS IN THE PIT ASAP

        if (DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue)) {
            if (targetPose.getX() < 4.5) {
                swerve.setDriveSpeeds(
                        xController.calculate(currentPose.getX(), targetPose.getX()),
                        yController.calculate(currentPose.getY(), targetPose.getY()),
                        rotationalController.calculate(currentPose.getRotation().getDegrees()),
                        false);
            } else {
                swerve.setDriveSpeeds(
                        -xController.calculate(currentPose.getX(), targetPose.getX()),
                        -yController.calculate(currentPose.getY(), targetPose.getY()),
                        rotationalController.calculate(currentPose.getRotation().getDegrees()),
                        false);
            }
        } else {
            // TODO MAKE A COPY OF ALIGNTOPOSE FOR AUTO WITH FIELD CENTRIC TURE!@!!!!@
            if (targetPose.getX() < 13) {
                swerve.setDriveSpeeds(xController.calculate(currentPose.getX()), yController.calculate(currentPose.getY()), rotationalController.calculate(currentPose.getRotation().getDegrees()), false);
            } else {
                swerve.setDriveSpeeds(-xController.calculate(currentPose.getX()), -yController.calculate(currentPose.getY()), rotationalController.calculate(currentPose.getRotation().getDegrees()), false);
            }
        }
        logs(
                () -> {
                    log("AlignToPose/Current Pose", currentPose);
                    log("AlignToPose/Target Pose", targetPose);
                    log("AlignToPose/Rotational Error", rotationalController.getPositionError());
                    log("AlignToPose/Y Error", yController.getPositionError());
                    log("AlignToPose/X Error ", xController.getPositionError());
                    log("AlignToPose/X Set ", xController.getSetpoint().position);
                    log("AlignToPose/X Goal ", xController.getGoal().position);
                    log("AlignToPose/Rotational Controller Setpoint", rotationalController.atSetpoint());
                    log("AlignToPose/Y Controller Setpoint", yController.atSetpoint());
                    log("AlignToPose/X Controller Setpoint ", xController.atSetpoint());
                    log("AlignToPose/X Set Speed ", xController.calculate(currentPose.getX(), targetPose.getX()));
                    log("AlignToPose/Y Set Speed ", yController.calculate(currentPose.getY()));
                    log(
                            "AlignToPose/Rot Set Speed ",
                            rotationalController.calculate(currentPose.getRotation().getDegrees()));
                    log("AlignToPose/ X Set Pos", currentPose.getX());
                    log("AlignToPose/ Y Set Pos", currentPose.getY());
                    log("AlignToPose/ X Target Pos", targetPose.getX());
                    log("AlignToPose/ Y Target Pos", targetPose.getY());
                });
    }

    /**
     * Returns whether this command has finished. Once a command finishes -- indicated by this method
     * returning true -- the scheduler will call its {@link #end(boolean)} method.
     *
     * <p>Returning false will result in the command never ending automatically. It may still be
     * cancelled manually or interrupted by another command. Hard coding this command to always return
     * true will result in the command executing once and finishing immediately. It is recommended to
     * use * {@link InstantCommand InstantCommand} for such an operation.
     *
     * @return whether this command has finished.
     */
    @Override
    public boolean isFinished() {
        if ((rotationalController.atSetpoint() && yController.atSetpoint() &&
                xController.atSetpoint()) || visionDead) {
            timer.start();
        } else {
            timer.reset();
        }
        return timer.hasElapsed(0.15);
//    return false;
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally --
     * that is called when {@link #isFinished()} returns true -- or when it is interrupted/canceled.
     * This is where you may want to wrap up loose ends, like shutting off a motor that was being used
     * in the command.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        Swerve.getInstance().stop();
    }

    //  public void initializeLoggedNetworkPingu() {
    //    networkPinguRotation = new NetworkPingu(new
    // LoggedNetworkNumber("Tuning/AlignToPose/Rotational P", rotationalController.getP()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/Rotational I", rotationalController.getI()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/Rotational D", rotationalController.getD()));
    //    networkPinguY = new NetworkPingu(new LoggedNetworkNumber("Tuning/AlignToPose/Y P",
    // yController.getP()), new LoggedNetworkNumber("Tuning/AlignToPose/Y I", yController.getI()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/Y D", yController.getD()));
    //    networkPinguX = new NetworkPingu(new LoggedNetworkNumber("Tuning/AlignToPose/X P",
    // xController.getP()), new LoggedNetworkNumber("Tuning/AlignToPose/X I", xController.getI()), new
    // LoggedNetworkNumber("Tuning/AlignToPose/X D", xController.getD()));
    //  }
}
