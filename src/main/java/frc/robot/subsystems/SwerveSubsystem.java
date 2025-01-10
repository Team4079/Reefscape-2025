package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.smartdashboard.SmartDashboard.*;
import static frc.robot.utils.Dash.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.SHOULD_INVERT;

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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.RobotParameters.SwerveParameters.*;
import frc.robot.subsystems.PhotonVision;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.photonvision.EstimatedRobotPose;

public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();
  private final Pigeon2 pidgey = new Pigeon2(RobotParameters.MotorParameters.PIDGEY_ID);
  private final SwerveModuleState[] states = new SwerveModuleState[4];
  private final SwerveModule[] modules;
  private final PID pid;
  private PathPlannerPath pathToScore = null;
  // from feeder to the goal and align itself
  // The plan is for it to path towards it then we use a set path to align itself with the goal and
  // be more accurate
  // Use this https://pathplanner.dev/pplib-pathfinding.html#pathfind-then-follow-path
  PathConstraints constraints =
      new PathConstraints(2.0, 3.0, Units.degreesToRadians(540), Units.degreesToRadians(720));
  private double velocity = 0.0;

  /**
   * The Singleton instance of this SwerveSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final SwerveSubsystem INSTANCE = new SwerveSubsystem();

  /**
   * Returns the Singleton instance of this SwerveSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * SwerveSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static SwerveSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this SwerveSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private SwerveSubsystem() {
    this.modules = initializeModules();
    this.pid = initializePID();
    this.pidgey.reset();
    this.poseEstimator = initializePoseEstimator();
    configureAutoBuilder();
  }

  /**
   * Initializes the swerve modules. Ensure the swerve modules are intialized in the same order as
   * in kinematics.
   *
   * @return An array of initialized SwerveModule objects.
   */
  private SwerveModule[] initializeModules() {
    return new SwerveModule[] {
      new SwerveModule(
          MotorParameters.FRONT_LEFT_DRIVE_ID,
          MotorParameters.FRONT_LEFT_STEER_ID,
          MotorParameters.FRONT_LEFT_CAN_CODER_ID,
          SwerveParameters.Thresholds.CANCODER_VAL9),
      new SwerveModule(
          MotorParameters.FRONT_RIGHT_DRIVE_ID,
          MotorParameters.FRONT_RIGHT_STEER_ID,
          MotorParameters.FRONT_RIGHT_CAN_CODER_ID,
          SwerveParameters.Thresholds.CANCODER_VAL10),
      new SwerveModule(
          MotorParameters.BACK_LEFT_DRIVE_ID,
          MotorParameters.BACK_LEFT_STEER_ID,
          MotorParameters.BACK_LEFT_CAN_CODER_ID,
          SwerveParameters.Thresholds.CANCODER_VAL11),
      new SwerveModule(
          MotorParameters.BACK_RIGHT_DRIVE_ID,
          MotorParameters.BACK_RIGHT_STEER_ID,
          MotorParameters.BACK_RIGHT_CAN_CODER_ID,
          SwerveParameters.Thresholds.CANCODER_VAL12)
    };
  }

  /**
   * Initializes the PID controller.
   *
   * @return A new PID object with values from the SmartDashboard.
   */
  private PID initializePID() {
    return new PID(
        getNumber("AUTO: P", PIDParameters.DRIVE_PID_AUTO.getP()),
        getNumber("AUTO: I", PIDParameters.DRIVE_PID_AUTO.getI()),
        getNumber("AUTO: D", PIDParameters.DRIVE_PID_AUTO.getD()));
  }

  /**
   * Initializes the SwerveDrivePoseEstimator. The SwerveDrivePoseEsimator estimates the robot's
   * position. This is based on a combination of the robot's movement and vision.
   *
   * @return A new SwerveDrivePoseEstimator object.
   */
  private SwerveDrivePoseEstimator initializePoseEstimator() {
    return new SwerveDrivePoseEstimator(
        SwerveParameters.PhysicalParameters.kinematics,
        Rotation2d.fromDegrees(getHeading()),
        getModulePositions(),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
  }

  /**
   * Configures the AutoBuilder for autonomous driving. READ DOCUMENTATION TO PUT IN CORRECT VALUES
   * Allows PathPlanner to get pose and output robot-relative chassis speeds Needs tuning
   */
  private void configureAutoBuilder() {
    AutoBuilder.configure(
        this::getPose,
        this::newPose,
        this::getAutoSpeeds,
        this::chassisSpeedsDrive,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PIDParameters.config,
        () -> {
          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isEmpty()) {
            return false;
          }
          if (SHOULD_INVERT) {
            return alliance.get() == Alliance.Red;
          } else {
            return alliance.get() != Alliance.Blue;
          }
        },
        this);
  }

  /**
   * This method is called periodically by the scheduler. It updates the pose estimator and
   * dashboard values.
   */
  @Override
  public void periodic() {
    /*
     This method checks whether the bot is in Teleop, and adds it to poseEstimator based on VISION
    */
    if (DriverStation.isTeleop()) {
      EstimatedRobotPose estimatedPose =
          PhotonVision.getInstance()
              .getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
      if (estimatedPose != null) {
        double timestamp = estimatedPose.timestampSeconds;
        Pose2d visionMeasurement2d = estimatedPose.estimatedPose.toPose2d();
        poseEstimator.addVisionMeasurement(visionMeasurement2d, timestamp);
        SwerveParameters.Thresholds.currentPose = poseEstimator.getEstimatedPosition();
      }
    }

    /*
     Updates the robot position based on movement and rotation from the pidgey and encoders.
    */
    poseEstimator.update(getPidgeyRotation(), getModulePositions());

    field.setRobotPose(poseEstimator.getEstimatedPosition());

    log("Pidgey Heading", getHeading());
    log("Pidgey Rotation2D", getPidgeyRotation().getDegrees());
    log("Robot Pose", field.getRobotPose());

    // Test mode toggle, replace later with Dash instance preferably instead of SmartDashboard
    putBoolean("Test Mode Enabled", Thresholds.TEST_MODE);
  }

  /**
   * Sets the drive speeds for the swerve modules.
   *
   * @param forwardSpeed The forward speed.
   * @param leftSpeed The left speed.
   * @param turnSpeed The turn speed.
   * @param isFieldOriented Whether the drive is field-oriented.
   */
  public void setDriveSpeeds(
      double forwardSpeed, double leftSpeed, double turnSpeed, boolean isFieldOriented) {

    log("Forward speed", forwardSpeed);
    log("Left speed", leftSpeed);

    ChassisSpeeds speeds =
        !isFieldOriented
            ? new ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed)
            : ChassisSpeeds.fromFieldRelativeSpeeds(
                forwardSpeed, leftSpeed, turnSpeed, getPidgeyRotation());
    SwerveModuleState[] states2 =
        SwerveParameters.PhysicalParameters.kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states2, MotorParameters.MAX_SPEED);
    setModuleStates(states2);
  }

  /**
   * Gets the rotation of the Pigeon2 IMU.
   *
   * @return The rotation of the Pigeon2 IMU.
   */
  public Rotation2d getPidgeyRotation() {
    return pidgey.getRotation2d();
  }

  /**
   * Gets the heading of the robot.
   *
   * @return The heading of the robot.
   */
  public double getHeading() {
    return -pidgey.getYaw().getValueAsDouble();
  }

  /**
   * Gets the yaw of the Pigeon2 IMU.
   *
   * @return The yaw of the Pigeon2 IMU.
   */
  public double getPidgeyYaw() {
    return pidgey.getYaw().getValueAsDouble();
  }

  /** Resets the Pigeon2 IMU. */
  public void resetPidgey() {
    pidgey.reset();
  }

  /**
   * Gets the current pose of the robot from he pose estimator.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Resets the pose of the robot to zero. */
  public void zeroPose() {
    poseEstimator.resetPosition(
        Rotation2d.fromDegrees(getHeading()),
        getModulePositions(),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)));
  }

  /**
   * Sets a new pose for the robot.
   *
   * @param pose The new pose.
   */
  public void newPose(Pose2d pose) {
    poseEstimator.resetPosition(Rotation2d.fromDegrees(getHeading()), getModulePositions(), pose);
  }

  /**
   * Gets the chassis speeds for autonomous driving.
   *
   * @return The chassis speeds for autonomous driving.
   */
  public ChassisSpeeds getAutoSpeeds() {
    SwerveDriveKinematics k = SwerveParameters.PhysicalParameters.kinematics;
    return k.toChassisSpeeds(getModuleStates());
  }

  /**
   * Gets the rotation of the Pigeon2 IMU for PID control.
   *
   * @return The rotation of the Pigeon2 IMU for PID control.
   */
  public Rotation2d getRotationPidggy() {
    return Rotation2d.fromDegrees(-pidgey.getRotation2d().getDegrees());
  }

  /**
   * Drives the robot using chassis speeds.
   *
   * @param chassisSpeeds The chassis speeds.
   */
  public void chassisSpeedsDrive(ChassisSpeeds chassisSpeeds) {
    SwerveModuleState[] newStates =
        SwerveParameters.PhysicalParameters.kinematics.toSwerveModuleStates(chassisSpeeds);
    setModuleStates(newStates);
  }

  /**
   * Gets the states of the swerve modules.
   *
   * @return The states of the swerve modules.
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
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
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }

  /**
   * Gets the positions of the swerve modules.
   *
   * @return The positions of the swerve modules.
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
    }
  }

  /** Resets the drive positions of all swerve modules. */
  public void resetDrive() {
    for (SwerveModule module : modules) {
      module.resetDrivePosition();
    }
  }

  /** Sets custom PID constants for the drive. */
  public void setCustomDrivePID() {
    dashPID("Drive", pid, PIDParameters.DRIVE_PID_V_AUTO, v -> velocity = v);
    for (SwerveModule module : modules) {
      module.setDrivePID(pid, velocity);
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
}
