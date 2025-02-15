package frc.robot.subsystems;

import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.SwerveParameters.PinguParameters.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.RobotParameters.SwerveParameters.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Represents a swerve module used in a swerve drive system. */
public class SwerveModule {
  private final TalonFX driveMotor;
  private final CANcoder canCoder;
  private final TalonFX steerMotor;
  private final PositionTorqueCurrentFOC positionSetter;
  private final VelocityTorqueCurrentFOC velocitySetter;
  //  private final VelocityDutyCycle velocitySetter;
  private final SwerveModulePosition swerveModulePosition;
  private SwerveModuleState state;
  private double driveVelocity;
  private double drivePosition;
  private double steerPosition;
  private double steerVelocity;
  private final TalonFXConfiguration driveConfigs;
  private final TalonFXConfiguration steerConfigs;
  private final TorqueCurrentConfigs driveTorqueConfigs;

  private LoggedNetworkNumber driveP;
  private LoggedNetworkNumber driveI;
  private LoggedNetworkNumber driveD;
  private LoggedNetworkNumber driveV;

  private LoggedNetworkNumber steerP;
  private LoggedNetworkNumber steerI;
  private LoggedNetworkNumber steerD;
  private LoggedNetworkNumber steerV;

  private Alert driveDisconnectedAlert;
  private Alert turnDisconnectedAlert;
  private Alert canCoderDisconnectedAlert;

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
    //    velocitySetter = new VelocityDutyCycle(0.0);
    swerveModulePosition = new SwerveModulePosition();
    state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    driveConfigs = new TalonFXConfiguration();

    // Set the PID values for the drive motor
    driveConfigs.Slot0.kP = PinguParameters.DRIVE_PINGU_AUTO.getP();
    driveConfigs.Slot0.kI = PinguParameters.DRIVE_PINGU_AUTO.getI();
    driveConfigs.Slot0.kD = PinguParameters.DRIVE_PINGU_AUTO.getD();
    driveConfigs.Slot0.kV = PinguParameters.DRIVE_PINGU_AUTO.getV();

    // Sets the brake mode, invered, and current limits for the drive motor
    driveConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.DRIVE_MOTOR_INVERTED;
    driveConfigs.CurrentLimits.SupplyCurrentLimit = MotorParameters.DRIVE_SUPPLY_LIMIT;
    driveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfigs.CurrentLimits.StatorCurrentLimit = MotorParameters.DRIVE_STATOR_LIMIT;
    driveConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfigs.Feedback.RotorToSensorRatio = MotorParameters.DRIVE_MOTOR_GEAR_RATIO;

    steerConfigs = new TalonFXConfiguration();

    // Set the PID values for the steer motor
    steerConfigs.Slot0.kP = PinguParameters.STEER_PINGU_AUTO.getP();
    steerConfigs.Slot0.kI = PinguParameters.STEER_PINGU_AUTO.getI();
    steerConfigs.Slot0.kD = PinguParameters.STEER_PINGU_AUTO.getD();
    steerConfigs.Slot0.kV = 0.0;
    steerConfigs.ClosedLoopGeneral.ContinuousWrap = true;

    // Sets the brake mode, inverted, and current limits for the steer motor
    steerConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfigs.MotorOutput.Inverted = SwerveParameters.Thresholds.STEER_MOTOR_INVERTED;
    steerConfigs.Feedback.FeedbackRemoteSensorID = canCoderID;
    steerConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    steerConfigs.Feedback.RotorToSensorRatio = MotorParameters.STEER_MOTOR_GEAR_RATIO;
    steerConfigs.CurrentLimits.SupplyCurrentLimit = MotorParameters.STEER_SUPPLY_LIMIT;
    steerConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

    driveTorqueConfigs = new TorqueCurrentConfigs();

    CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();

    /*
     * Sets the CANCoder direction, absolute sensor range, and magnet offset for the
     * CANCoder Make sure the magnet offset is ACCURATE and based on when the wheel
     * is straight!
     */
    canCoderConfiguration.MagnetSensor.SensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfiguration.MagnetSensor.MagnetOffset =
        SwerveParameters.Thresholds.ENCODER_OFFSET + canCoderDriveStraightSteerSetPoint;
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

    swerveModulePosition.angle =
        Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    swerveModulePosition.distanceMeters =
        (drivePosition / MotorParameters.DRIVE_MOTOR_GEAR_RATIO * MotorParameters.METERS_PER_REV);

    return swerveModulePosition;
  }

  /**
   * Gets the current state of the swerve module.
   *
   * @return SwerveModuleState, The current state of the swerve module, including the angle and
   *     speed.
   */
  public SwerveModuleState getState() {
    state.angle = Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());
    state.speedMetersPerSecond =
        (driveMotor.getRotorVelocity().getValueAsDouble()
            / MotorParameters.DRIVE_MOTOR_GEAR_RATIO
            * MotorParameters.METERS_PER_REV);
    return state;
  }

  /**
   * Sets the state of the swerve module.
   *
   * @param desiredState The desired state of the swerve module.
   */
  public void setState(SwerveModuleState desiredState) {
    // Get the current angle
    Rotation2d currentAngle =
        Rotation2d.fromRotations(canCoder.getAbsolutePosition().getValueAsDouble());

    // Optimize the desired state based on current angle
    desiredState.optimize(currentAngle);

    // Set the angle for the steer motor
    double angleToSet = desiredState.angle.getRotations();
    steerMotor.setControl(positionSetter.withPosition(angleToSet));

    // Set the velocity for the drive motor
    double velocityToSet =
        (desiredState.speedMetersPerSecond
            * (MotorParameters.DRIVE_MOTOR_GEAR_RATIO / MotorParameters.METERS_PER_REV));
    driveMotor.setControl(velocitySetter.withVelocity(velocityToSet));

    // Log the actual and set values for debugging
    logs(
        () -> {
          log("drive actual sped", driveMotor.getVelocity().getValueAsDouble());
          log("drive set sped", velocityToSet);
          log("steer actual angle", canCoder.getAbsolutePosition().getValueAsDouble());
          log("steer set angle", angleToSet);
          log("desired state after optimize", desiredState.angle.getRotations());
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
   * Sets the drive PID values.
   *
   * @param pid The PID object containing the PID values.
   * @param velocity The velocity value.
   */
  public void setDrivePID(PIDController pid, double velocity) {
    driveConfigs.Slot0.kP = pid.getP();
    driveConfigs.Slot0.kI = pid.getI();
    driveConfigs.Slot0.kD = pid.getD();
    driveConfigs.Slot0.kV = velocity;
    driveMotor.getConfigurator().apply(driveConfigs);
  }

  /**
   * Sets the steer PID values.
   *
   * @param pid The PID object containing the PID values.
   * @param velocity The velocity value.
   */
  public void setSteerPID(PIDController pid, double velocity) {
    steerConfigs.Slot0.kP = pid.getP();
    steerConfigs.Slot0.kI = pid.getI();
    steerConfigs.Slot0.kD = pid.getD();
    steerConfigs.Slot0.kV = velocity;
    steerMotor.getConfigurator().apply(steerConfigs);
  }

  public void applyTelePIDValues() {
    driveConfigs.Slot0.kP = driveP.get();
    driveConfigs.Slot0.kI = driveI.get();
    driveConfigs.Slot0.kD = driveD.get();
    driveConfigs.Slot0.kV = driveV.get();

    steerConfigs.Slot0.kP = steerP.get();
    steerConfigs.Slot0.kI = steerI.get();
    steerConfigs.Slot0.kD = steerD.get();
    steerConfigs.Slot0.kV = steerV.get();

    driveMotor.getConfigurator().apply(driveConfigs);
    steerMotor.getConfigurator().apply(steerConfigs);
  }

  /** Sets the PID values for teleoperation mode. */
  public void setTelePID() {
    setDrivePID(DRIVE_PINGU_TELE.toPIDController(), DRIVE_PINGU_TELE.getV());
    setSteerPID(STEER_PINGU_TELE.toPIDController(), STEER_PINGU_TELE.getV());
  }

  /** Sets the PID values for autonomous mode. */
  public void setAutoPID() {
    setDrivePID(
        PinguParameters.DRIVE_PINGU_AUTO.toPIDController(),
        PinguParameters.DRIVE_PINGU_AUTO.getV());
  }

  /** Resets the drive motor position to zero. */
  public void resetDrivePosition() {
    driveMotor.setPosition(0.0);
  }

  public void initializeLoggedNetworkPID() {
    driveP = new LoggedNetworkNumber("/Tuning/Swerve/Drive P", driveConfigs.Slot0.kP);
    driveI = new LoggedNetworkNumber("/Tuning/Swerve/Drive I", driveConfigs.Slot0.kI);
    driveD = new LoggedNetworkNumber("/Tuning/Swerve/Drive D", driveConfigs.Slot0.kD);
    driveV = new LoggedNetworkNumber("/Tuning/Swerve/Drive V", driveConfigs.Slot0.kV);

    steerP = new LoggedNetworkNumber("/Tuning/Swerve/Steer P", steerConfigs.Slot0.kP);
    steerI = new LoggedNetworkNumber("/Tuning/Swerve/Steer I", steerConfigs.Slot0.kI);
    steerD = new LoggedNetworkNumber("/Tuning/Swerve/Steer D", steerConfigs.Slot0.kD);
    steerV = new LoggedNetworkNumber("/Tuning/Swerve/Steer V", steerConfigs.Slot0.kV);
  }

  public void updateTelePID() {
    DRIVE_PINGU_TELE.setP(driveP.get());
    DRIVE_PINGU_TELE.setI(driveI.get());
    DRIVE_PINGU_TELE.setD(driveD.get());

    STEER_PINGU_TELE.setP(steerP.get());
    STEER_PINGU_TELE.setI(steerI.get());
    STEER_PINGU_TELE.setD(steerD.get());

    applyTelePIDValues();
  }

  /**
   * Initializes alerts for disconnected components.
   *
   * @param driveId The ID of the drive motor.
   * @param steerId The ID of the steer motor.
   * @param canCoderId The ID of the CANcoder.
   */
  public void initializeAlarms(int driveId, int steerId, int canCoderId) {
    driveDisconnectedAlert =
        new Alert("Disconnected drive motor " + Integer.toString(driveId), AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor " + Integer.toString(steerId), AlertType.kError);
    canCoderDisconnectedAlert =
        new Alert("Disconnected CANCoder " + Integer.toString(canCoderId), AlertType.kError);

    driveDisconnectedAlert.set(!driveMotor.isConnected());
    turnDisconnectedAlert.set(!steerMotor.isConnected());
    canCoderDisconnectedAlert.set(!canCoder.isConnected());
  }
}
