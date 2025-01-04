package frc.robot.subsystems;

import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.SHOULD_INVERT;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Dash;
import frc.robot.utils.PID;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.RobotParameters.SwerveParameters.*;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

/**
 * The SwerveSubsystem class represents the swerve drive subsystem of the robot. It handles the
 * control and state estimation of the swerve drive modules.
 */
public class SwerveSubsystem extends SubsystemBase {
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Field2d field = new Field2d();
  private final Pigeon2 pidgey = new Pigeon2(MotorParameters.PIDGEY_ID);
  private final SwerveModuleState[] states = new SwerveModuleState[4];
  private final SwerveModule[] modules;
  private final Photonvision photonvision;
  private final PID pid;
  private double velocity = 0.0;

  /**
   * Constructs a new SwerveSubsystem.
   *
   * @param photonvision The Photonvision instance used for vision processing.
   */
  public SwerveSubsystem(Photonvision photonvision) {
    this.photonvision = photonvision;
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
        SmartDashboard.getNumber("AUTO: P", PIDParameters.DRIVE_PID_AUTO.getP()),
        SmartDashboard.getNumber("AUTO: I", PIDParameters.DRIVE_PID_AUTO.getI()),
        SmartDashboard.getNumber("AUTO: D", PIDParameters.DRIVE_PID_AUTO.getD()));
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
          photonvision.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
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

    Dash.dash(
        Dash.pairOf("Pitch", pidgey.getPitch().getValueAsDouble()),
        Dash.pairOf("Heading", -pidgey.getYaw().getValueAsDouble()),
        Dash.pairOf("Yaw", pidgey.getYaw().getValueAsDouble()),
        Dash.pairOf("Roll", pidgey.getRoll().getValueAsDouble()),
        Dash.pairOf("Robot Pose", field));
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
    Dash.dash(
        Dash.pairOf("Forward speed", forwardSpeed),
        Dash.pairOf("Left speed", leftSpeed),
        Dash.pairOf("Pidgey Heading", getHeading()));

    ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, leftSpeed, turnSpeed);
    if (!isFieldOriented) {
      speeds.toRobotRelativeSpeeds(getPidgeyRotation());
    }

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

  /**
   * Sets the initial heading of the robot based on the alliance color. Our attempt to set the
   * intial heading based on our alliance color. Likely should be able to use vision to set this
   * automaically.
   */
  public void setInitialHeading() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty() || alliance.get() == Alliance.Red) {
      pidgey.setYaw(27.4);
    } else {
      pidgey.setYaw(-27.4);
    }
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
      module.setAUTOPID();
    }
  }

  /** Sets the PID constants for teleoperated driving. */
  public void setTelePID() {
    for (SwerveModule module : modules) {
      module.setTELEPID();
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
    Dash.dashPID("Drive", pid, PIDParameters.DRIVE_PID_V_AUTO, v -> velocity = v);
    for (SwerveModule module : modules) {
      module.setDrivePID(pid, velocity);
    }
  }
}
