package frc.robot.subsystems;

import static com.pathplanner.lib.path.PathPlannerPath.fromPathFile;
import static com.pathplanner.lib.util.PathPlannerLogging.*;
import static edu.wpi.first.math.VecBuilder.*;
import static edu.wpi.first.math.geometry.Rotation2d.*;
import static edu.wpi.first.math.kinematics.ChassisSpeeds.*;
import static edu.wpi.first.math.kinematics.SwerveDriveKinematics.*;
import static edu.wpi.first.math.util.Units.*;
import static frc.robot.utils.ExtensionsKt.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.PhysicalParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.*;

import java.util.Optional;

public class Swerve extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();
  private final Pigeon2 pidgey = new Pigeon2(PIDGEY_ID);
  private final SwerveModuleState[] states = new SwerveModuleState[4];
  private SwerveModuleState[] setStates = new SwerveModuleState[4];
  private final SwerveModule[] modules;
  private final PathPlannerPath pathToScore;

  Thread swerveLoggingThread =
      new Thread(
          () -> {
            while (true) {
              logs("Swerve Module States", getModuleStates());
              try {
                Thread.sleep(100);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
              }
            }
          });

  Thread swerveLoggingThreadBeforeSet =
      new Thread(
          () -> {
            while (true) {
              logs("Set Swerve Module States", setStates);
              try {
                Thread.sleep(100);
              } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
              }
            }
          });

  // from feeder to the goal and align itself
  // The plan is for it to path towards it then we use a set path to align itself
  // with the goal and
  // be more accurate
  // Use this
  // https://pathplanner.dev/pplib-pathfinding.html#pathfind-then-follow-path
  PathConstraints constraints =
      new PathConstraints(2.0, 3.0, degreesToRadians(540), degreesToRadians(720));

  /**
   * The Singleton instance of this SwerveSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final Swerve INSTANCE = new Swerve();

  /**
   * Returns the Singleton instance of this SwerveSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * SwerveSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Swerve getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this SwerveSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private Swerve() {
    this.modules = initializeModules();
    this.pidgey.reset();
    this.poseEstimator = initializePoseEstimator();
    configureAutoBuilder();
    initializePathPlannerLogging();

    swerveLoggingThread.start();
    swerveLoggingThreadBeforeSet.start();

    try {
      pathToScore = fromPathFile("Straight Path");
    } catch (Exception e) {
      throw new RobotConfigException("Failed to load robot config", e);
    }
  }

  /**
   * Initializes the swerve modules. Ensure the swerve modules are initialized in the same order as
   * in kinematics.
   *
   * @return SwerveModule[], An array of initialized SwerveModule objects.
   */
  private SwerveModule[] initializeModules() {
    return new SwerveModule[] {
      new SwerveModule(
          FRONT_LEFT_DRIVE_ID, FRONT_LEFT_STEER_ID, FRONT_LEFT_CAN_CODER_ID, CANCODER_VAL9),
      new SwerveModule(
          FRONT_RIGHT_DRIVE_ID, FRONT_RIGHT_STEER_ID, FRONT_RIGHT_CAN_CODER_ID, CANCODER_VAL10),
      new SwerveModule(
          BACK_LEFT_DRIVE_ID, BACK_LEFT_STEER_ID, BACK_LEFT_CAN_CODER_ID, CANCODER_VAL11),
      new SwerveModule(
          BACK_RIGHT_DRIVE_ID, BACK_RIGHT_STEER_ID, BACK_RIGHT_CAN_CODER_ID, CANCODER_VAL12)
    };
  }

  /**
   * Initializes the SwerveDrivePoseEstimator. The SwerveDrivePoseEsimator estimates the robot's
   * position. This is based on a combination of the robot's movement and vision.
   *
   * @return SwerveDrivePoseEstimator, A new SwerveDrivePoseEstimator object.
   */
  private SwerveDrivePoseEstimator initializePoseEstimator() {
    return new SwerveDrivePoseEstimator(
        kinematics,
        fromDegrees(getHeading()),
        getModulePositions(),
        new Pose2d(0.0, 0.0, fromDegrees(0.0)),
        fill(0.05, 0.05, degreesToRadians(5)),
        fill(0.5, 0.5, degreesToRadians(30)));
  }

  /**
   * Initializes the PathPlannerLogging. This allows the PathPlanner to log the robot's current
   * pose.
   */
  private void initializePathPlannerLogging() {
    setLogCurrentPoseCallback(field::setRobotPose);
    setLogTargetPoseCallback(pose -> field.getObject("target pose").setPose(pose));
    setLogActivePathCallback(poses -> field.getObject("path").setPoses(poses));
  }

  /**
   * Configures the AutoBuilder for autonomous driving. READ DOCUMENTATION TO PUT IN CORRECT VALUES
   * Allows PathPlanner to get pose and output robot-relative chassis speeds Needs tuning
   */
  private void configureAutoBuilder() {
    assert PIDParameters.config != null;
    AutoBuilder.configure(
        this::getPose,
        this::newPose,
        this::getAutoSpeeds,
        this::chassisSpeedsDrive,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PIDParameters.config,
        () -> DriverStation.getAlliance().filter(value -> value == Alliance.Red).isPresent(),
        this);
  }

  /**
   * This method is called periodically by the scheduler. It updates the pose estimator and
   * dashboard values.
   */
  @Override
  public void periodic() {
    /*
     * This method checks whether the bot is in Teleop, and adds it to poseEstimator
     * based on VISION
     */
    if (DriverStation.isTeleop()) {
      PhotonVision.getInstance()
          .resultPairs
          .get()
          .forEach(
              pair -> {
                EstimatedRobotPose pose =
                    getEstimatedPose(pair, poseEstimator.getEstimatedPosition());
                updateStdDev(pair, Optional.ofNullable(pose));
                if (pose != null) {
                  double timestamp = pose.timestampSeconds;
                  Pose2d visionMeasurement2d = pose.estimatedPose.toPose2d();
                  poseEstimator.addVisionMeasurement(
                      visionMeasurement2d, timestamp, pair.getFirst().getCurrentStdDevs());
                  robotPos = poseEstimator.getEstimatedPosition();
                }
              });
    }

    /*
     * Updates the robot position based on movement and rotation from the pidgey and
     * encoders.
     */
    poseEstimator.update(getPidgeyRotation(), getModulePositions());

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    logs(
        () -> {
          log("Pidgey Yaw", getPidgeyYaw());
          log("Pidgey Heading", getHeading());
          log("Pidgey Rotation2D", pidgey.getRotation2d().getDegrees());
          log("Robot Pose", field.getRobotPose());
        });
  }

  /**
   * Sets the drive speeds for the swerve modules.
   *
   * @param forwardSpeed The forward speed.
   * @param leftSpeed The left speed.
   * @param turnSpeed The turn speed.
   */
  public void setDriveSpeeds(double forwardSpeed, double leftSpeed, double turnSpeed) {
    setDriveSpeeds(forwardSpeed, leftSpeed, turnSpeed, IS_FIELD_ORIENTED);
  }

  /**
   * Sets the drive speeds for the swerve modules.
   *
   * @param forwardSpeed The forward speed.
   * @param leftSpeed The left speed.
   * @param turnSpeed The turn speed.
   * @param isFieldOriented Whether the robot is field oriented.
   */
  public void setDriveSpeeds(
      double forwardSpeed, double leftSpeed, double turnSpeed, boolean isFieldOriented) {
    logs(
        () -> {
          log("Forward speed", forwardSpeed);
          log("Left speed", leftSpeed);
          log("Turn speed", turnSpeed);
        });

    // Converts to a measure that the robot aktualy understands
    ChassisSpeeds speeds =
        isFieldOriented
            ? fromFieldRelativeSpeeds(forwardSpeed, leftSpeed, turnSpeed, getPidgeyRotation())
            : new ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed);

    speeds = discretize(speeds, 0.02);

    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(speeds);
    desaturateWheelSpeeds(newStates, MAX_SPEED);

    setModuleStates(newStates);
  }

  /**
   * Gets the rotation of the Pigeon2 IMU.
   *
   * @return Rotation2d, The rotation of the Pigeon2 IMU.
   */
  public Rotation2d getPidgeyRotation() {
    return pidgey.getRotation2d();
  }

  /**
   * Gets the heading of the robot.
   *
   * @return double, The heading of the robot.
   */
  public double getHeading() {
    return -pidgey.getYaw().getValueAsDouble();
  }

  /**
   * Gets the yaw of the Pigeon2 IMU.
   *
   * @return double, The yaw of the Pigeon2 IMU.
   */
  public double getPidgeyYaw() {
    return pidgey.getYaw().getValueAsDouble();
  }

  /** Resets the Pigeon2 IMU. */
  public void resetPidgey() {
    pidgey.reset();
  }

  /**
   * Gets the current pose of the robot from the pose estimator.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the pose of the robot to zero. */
  public void zeroPose() {
    poseEstimator.resetPosition(
        fromDegrees(getHeading()), getModulePositions(), new Pose2d(0.0, 0.0, fromDegrees(0.0)));
  }

  /**
   * Sets a new pose for the robot.
   *
   * @param pose The new pose.
   */
  public void newPose(Pose2d pose) {
    poseEstimator.resetPosition(getPidgeyRotation(), getModulePositions(), pose);
  }

  /**
   * Gets the chassis speeds for autonomous driving.
   *
   * @return ChassisSpeeds, The chassis speeds for autonomous driving.
   */
  public ChassisSpeeds getAutoSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the rotation of the Pigeon2 IMU for PID control.
   *
   * @return Rotation2d, The rotation of the Pigeon2 IMU for PID control.
   */
  public Rotation2d getRotationPidggy() {
    return fromDegrees(-pidgey.getRotation2d().getDegrees());
  }

  /**
   * Drives the robot using chassis speeds.
   *
   * @param chassisSpeeds The chassis speeds.
   */
  public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(newStates);
  }

  /**
   * Gets the states of the swerve modules.
   *
   * @return SwerveModuleState[], The states of the swerve modules.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i < modules.length; i++) {
      moduleStates[i] = modules[i].getState();
    }
    return moduleStates;
  }

  /**
   * Sets the states of the swerve modules.
   *
   * @param states The states of the swerve modules.
   */
  public void setModuleStates(SwerveModuleState[] states) {
    setStates = states;
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /**
   * Gets the positions of the swerve modules.
   *
   * @return SwerveModulePosition[], The positions of the swerve modules.
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[states.length];
    for (int i = 0; i < positions.length; i++) {
      positions[i] = modules[i].getPosition();
    }
    return positions;
  }

  /** Stops all swerve modules. */
  public void stop() {
    for (SwerveModule module : modules) {
      module.stop();
    }
  }

  /** Sets the PID constants for autonomous driving. */
  public void setAutoPID() {
    for (SwerveModule module : modules) {
      module.setAutoPID();
    }
  }

  /** Sets the PID constants for teleoperated driving. */
  public void setTelePID() {
    for (SwerveModule module : modules) {
      module.setTelePID();
      module.applyTelePIDValues();
    }
  }

  /** Resets the drive positions of all swerve modules. */
  public void resetDrive() {
    for (SwerveModule module : modules) {
      module.resetDrivePosition();
    }
  }

  public void updateModuleTelePIDValues() {
    for (SwerveModule module : modules) {
      module.updateTelePID();
    }
  }

  public Command pathFindToGoal() {
    return AutoBuilder.pathfindThenFollowPath(pathToScore, constraints);
  }

  public Command pathFindTest() {
    return AutoBuilder.pathfindThenFollowPath(pathToScore, constraints);
  }

  private static class RobotConfigException extends RuntimeException {
    public RobotConfigException(String message, Throwable cause) {
      super(message, cause);
    }
  }
}
