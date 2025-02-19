package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.SensorDirectionValue.*;
import static edu.wpi.first.math.geometry.Rotation2d.*;
import static frc.robot.utils.ExtensionsKt.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.Thresholds.*;
import static frc.robot.utils.pingu.LogPingu.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.pingu.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Represents a swerve module used in a swerve drive system. */
public class SwerveModule {
  private final TalonFX driveMotor;
  private final CANcoder canCoder;
  private final TalonFX steerMotor;
  private final PositionTorqueCurrentFOC positionSetter;
  private final VelocityTorqueCurrentFOC velocitySetter;
  private final SwerveModulePosition swerveModulePosition;
  private SwerveModuleState state;
  private double driveVelocity;
  private double drivePosition;
  private double steerPosition;
  private double steerVelocity;
  private final TalonFXConfiguration driveConfigs;
  private final TalonFXConfiguration steerConfigs;
  private final TorqueCurrentConfigs driveTorqueConfigs;

  private NetworkPingu networkPinguDrive;
  private NetworkPingu networkPinguSteer;

  /**
   * Constructs a new SwerveModule.
   *
   * @param driveId The ID of the drive motor.
   * @param steerId The ID of the steer motor.
   * @param canCoderID The ID of the CANcoder.
   * @param canCoderDriveStraightSteerSetPoint The set point for the CANcoder drive straight steer.
   */
  public SwerveModule(
      int driveId, int steerId, int canCoderID, double canCoderDriveStraightSteerSetPoint) {
    driveMotor = new TalonFX(driveId);
    canCoder = new CANcoder(canCoderID);
    steerMotor = new TalonFX(steerId);
    positionSetter = new PositionTorqueCurrentFOC(0.0);
    velocitySetter = new VelocityTorqueCurrentFOC(0.0);
    swerveModulePosition = new SwerveModulePosition();
    state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    driveConfigs = new TalonFXConfiguration();

    // Set the Pingu values for the drive motor
    setPingu(driveConfigs, DRIVE_PINGU_AUTO);

    // Sets the brake mode, invered, and current limits for the drive motor
    driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.DRIVE_MOTOR_INVERTED;
    driveConfigs.CurrentLimits.SupplyCurrentLimit = DRIVE_SUPPLY_LIMIT;
    driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfigs.CurrentLimits.StatorCurrentLimit = DRIVE_STATOR_LIMIT;
    driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfigs.Feedback.RotorToSensorRatio = DRIVE_MOTOR_GEAR_RATIO;

    steerConfigs = new TalonFXConfiguration();

    // Set the PID values for the steer motor
    setPingu(steerConfigs, STEER_PINGU_AUTO);
    steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // Sets the brake mode, inverted, and current limits for the steer motor
    steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.STEER_MOTOR_INVERTED;
    steerConfigs.Feedback.FeedbackRemoteSensorID = canCoderID;
    steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfigs.Feedback.RotorToSensorRatio = STEER_MOTOR_GEAR_RATIO;
    steerConfigs.CurrentLimits.SupplyCurrentLimit = STEER_SUPPLY_LIMIT;
    steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveTorqueConfigs = new TorqueCurrentConfigs();

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

    /*
     * Sets the CANCoder direction, absolute sensor range, and magnet offset for the
     * CANCoder Make sure the magnet offset is ACCURATE and based on when the wheel
     * is straight!
     */
    canCoderConfiguration.MagnetSensor.SensorDirection = CounterClockwise_Positive;
    canCoderConfiguration.MagnetSensor.MagnetOffset =
        ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint;
    canCoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

    driveMotor.getConfigurator().apply(driveConfigs);
    steerMotor.getConfigurator().apply(steerConfigs);
    canCoder.getConfigurator().apply(canCoderConfiguration);

    driveVelocity = driveMotor.getVelocity().getValueAsDouble();
    drivePosition = driveMotor.getPosition().getValueAsDouble();
    steerVelocity = steerMotor.getVelocity().getValueAsDouble();
    steerPosition = steerMotor.getPosition().getValueAsDouble();

    initializeLoggedNetworkPID();
    initializeAlarms(driveId, steerId, canCoderID);
  }

  /**
   * Gets the current position of the swerve module.
   *
   * @return SwerveModulePosition, The current position of the swerve module.
   */
  public SwerveModulePosition getPosition() {
    driveVelocity = driveMotor.getVelocity().getValueAsDouble();
    drivePosition = driveMotor.getPosition().getValueAsDouble();
    steerVelocity = steerMotor.getVelocity().getValueAsDouble();
    steerPosition = steerMotor.getPosition().getValueAsDouble();

    swerveModulePosition.angle = fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    swerveModulePosition.distanceMeters = drivePosition / DRIVE_MOTOR_GEAR_RATIO * METERS_PER_REV;

    return swerveModulePosition;
  }

  /**
   * Gets the current state of the swerve module.
   *
   * @return SwerveModuleState, The current state of the swerve module, including the angle and
   *     speed.
   */
  public SwerveModuleState getState() {
    state.angle = fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    state.speedMetersPerSecond =
        driveMotor.getRotorVelocity().getValueAsDouble() / DRIVE_MOTOR_GEAR_RATIO * METERS_PER_REV;
    return state;
  }

  /**
   * Sets the state of the swerve module.
   *
   * @param desiredState The desired state of the swerve module.
   */
  public void setState(SwerveModuleState desiredState) {
    // Get the current angle
    Rotation2d currentAngle = fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());

    // Optimize the desired state based on current angle
    desiredState.optimize(currentAngle);

    // Set the angle for the steer motor
    double angleToSet = desiredState.angle.getRotations();
    steerMotor.setControl(positionSetter.withPosition(angleToSet));

    // Set the velocity for the drive motor
    double velocityToSet =
        (desiredState.speedMetersPerSecond * (DRIVE_MOTOR_GEAR_RATIO / METERS_PER_REV));
    driveMotor.setControl(velocitySetter.withVelocity(velocityToSet));

    // Log the actual and set values for debugging
    logs(
        () -> {
          log("Swerve/Drive actual sped", driveMotor.getVelocity().getValueAsDouble());
          log("Swerve/Drive set sped", velocityToSet);
          log("Swerve/Steer actual angle", canCoder.getAbsolutePosition().getValueAsDouble());
          log("Swerve/Steer set angle", angleToSet);
          log("Swerve/Desired state after optimize", desiredState.angle.getRotations());
        });

    // Update the state with the optimized values
    state = desiredState;
  }

  /** Stops the swerve module motors. */
  public void stop() {
    steerMotor.stopMotor();
    driveMotor.stopMotor();
  }

  /**
   * Sets the PID configuration values for the drive motor using a Pingu configuration object.
   *
   * @param pingu A Pingu object containing PID and feedforward values (P, I, D, V)
   */
  public void setDrivePingu(Pingu pingu) {
    setPingu(driveConfigs, pingu);
    driveMotor.getConfigurator().apply(driveConfigs);
  }

  /**
   * Sets the PID configuration values for the steer motor using a Pingu configuration object.
   *
   * @param pingu A Pingu object containing PID and feedforward values (P, I, D, V)
   */
  public void setSteerPingu(Pingu pingu) {
    setPingu(steerConfigs, pingu);
    driveMotor.getConfigurator().apply(steerConfigs);
  }

  /** Sets the PID values for teleoperation mode. */
  public void setTelePID() {
    setDrivePingu(DRIVE_PINGU_TELE);
    setSteerPingu(STEER_PINGU_TELE);
  }

  /** Sets the PID values for autonomous mode. */
  public void setAutoPID() {
    setDrivePingu(DRIVE_PINGU_AUTO);
  }

  /** Resets the drive motor position to zero. */
  public void resetDrivePosition() {
    driveMotor.setPosition(0.0);
  }

  /**
   * Initializes the logged network PID values for the drive and steer motors. This method sets the
   * PID values for the drive and steer motors using the network Pingu configuration objects and
   * applies these configurations to the respective motors.
   */
  public void initializeLoggedNetworkPID() {
    networkPinguDrive =
        new NetworkPingu(
            new LoggedNetworkNumber("Tuning/Swerve/Drive P", driveConfigs.Slot0.kP),
            new LoggedNetworkNumber("Tuning/Swerve/Drive I", driveConfigs.Slot0.kI),
            new LoggedNetworkNumber("Tuning/Swerve/Drive D", driveConfigs.Slot0.kD),
            new LoggedNetworkNumber("Tuning/Swerve/Drive V", driveConfigs.Slot0.kV));
    networkPinguSteer =
        new NetworkPingu(
            new LoggedNetworkNumber("Tuning/Swerve/Steer P", steerConfigs.Slot0.kP),
            new LoggedNetworkNumber("Tuning/Swerve/Steer I", steerConfigs.Slot0.kI),
            new LoggedNetworkNumber("Tuning/Swerve/Steer D", steerConfigs.Slot0.kD),
            new LoggedNetworkNumber("Tuning/Swerve/Steer V", steerConfigs.Slot0.kV));
  }

  /**
   * Updates the PID values for teleoperation mode. This method retrieves the PID values from the
   * network Pingu configuration objects and sets these values for the drive and steer motors.
   */
  public void updateTelePID() {
    DRIVE_PINGU_TELE.setPID(networkPinguDrive);
    STEER_PINGU_TELE.setPID(networkPinguSteer);

    applyTelePIDValues();
  }

  /**
   * Applies the PID configuration values for teleoperation mode. This method sets the PID values
   * for both the drive and steer motors using the network Pingu configuration objects and applies
   * these configurations to the respective motors.
   */
  public void applyTelePIDValues() {
    setPingu(driveConfigs, networkPinguDrive);
    setPingu(steerConfigs, networkPinguSteer);

    driveMotor.getConfigurator().apply(driveConfigs);
    steerMotor.getConfigurator().apply(steerConfigs);
  }

  /**
   * Initializes alerts for disconnected components.
   *
   * @param driveId The ID of the drive motor.
   * @param steerId The ID of the steer motor.
   * @param canCoderId The ID of the CANcoder.
   */
  public void initializeAlarms(int driveId, int steerId, int canCoderId) {
    AlertPingu.add(driveMotor, "drive motor");
    AlertPingu.add(steerMotor, "steer motor");
    AlertPingu.add(canCoder);
  }
}
