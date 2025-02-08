package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.Alert.AlertType.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.*;

/**
 * The PivotSubsystem class is a subsystem that interfaces with the pivot system to provide control
 * over the pivot motors. This subsystem is a Singleton, meaning that only one instance of this
 * class is created and shared across the entire robot code.
 */
public class Climber extends SubsystemBase {
  /** Creates a new Pivot. */
  private final TalonFX climberMotor;

  private final SoftwareLimitSwitchConfigs climberMotorSoftLimitConfig;

  private final PositionVoltage pos_request;
  private final VelocityVoltage vel_voltage;

  private final VoltageOut voltageOut;

  private Alert climberMotorDisconnectedAlert;

  // private double absPos = 0;

  /**
   * The Singleton instance of this PivotSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final Climber INSTANCE = new Climber();

  /**
   * Returns the Singleton instance of this PivotSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * PivotSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Climber getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this PivotSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private Climber() {
    climberMotor = new TalonFX(CLIMBER_MOTOR_ID);

    MotorOutputConfigs pivotOutputConfigs = new MotorOutputConfigs(); // TODO: Never used?

    Slot0Configs pivotConfigs = new Slot0Configs();

    // Configurations
    TalonFXConfiguration climberMotorConfiguration = new TalonFXConfiguration();

    climberMotor.getConfigurator().apply(climberMotorConfiguration);
    climberMotor.getConfigurator().apply(pivotConfigs);

    pivotOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    pivotConfigs.kP = ClimberParameters.CLIMBER_PID_P;
    pivotConfigs.kI = ClimberParameters.CLIMBER_PID_I;
    pivotConfigs.kD = ClimberParameters.CLIMBER_PID_D;
    pivotConfigs.kV = ClimberParameters.CLIMBER_PID_V;
    // pivotConfigs.kF = PivotConstants.PIVOT_PID_F;

    climberMotor.getConfigurator().apply(pivotConfigs);

    CurrentLimitsConfigs climberMotorCurrentConfig = new CurrentLimitsConfigs();
    ClosedLoopRampsConfigs climberMotorRampConfig = new ClosedLoopRampsConfigs();
    climberMotorSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    climberMotorCurrentConfig.SupplyCurrentLimit = 100;
    climberMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    climberMotorCurrentConfig.StatorCurrentLimit = 100;
    climberMotorCurrentConfig.StatorCurrentLimitEnable = true;

    climberMotor.getConfigurator().apply(climberMotorCurrentConfig);

    climberMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    climberMotor.getConfigurator().apply(climberMotorRampConfig);

    climberMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
    climberMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
    climberMotorSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    climberMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    climberMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    climberMotorConfiguration.SoftwareLimitSwitch = climberMotorSoftLimitConfig;

    climberMotor.getConfigurator().apply(climberMotorSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    pos_request = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    climberMotor.setPosition(0);

    initializeAlarms();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    logs("Pivot Motor Position", climberMotor.getPosition().getValueAsDouble());
  }

  /** Stops the pivot motor */
  public void stopMotor() {
    climberMotor.stopMotor();
    voltageOut.Output = -0.014;
    climberMotor.setControl(voltageOut);
  }

  /**
   * Set the position of the left and right pivot motors
   *
   * @param motorPos Motor position
   */
  public void setMotorPosition(double motorPos) {
    climberMotor.setControl(pos_request.withPosition(motorPos));
  }

  /**
   * Get the position of the elevator motor
   *
   * @return double, the position of the elevator motor
   */
  public double getPivotPosValue() {
    return climberMotor.getPosition().getValue().magnitude();
  }

  /** Soft resets the encoders on the elevator motors */
  public void resetEncoders() {
    climberMotor.setPosition(0);
  }

  /** Toggles the soft stop for the elevator motor */
  public void toggleSoftStop() {
    ClimberParameters.isSoftLimitEnabled = !ClimberParameters.isSoftLimitEnabled;
    climberMotorSoftLimitConfig.ReverseSoftLimitEnable = ClimberParameters.isSoftLimitEnabled;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    climberMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    climberMotor.getConfigurator().apply(climberMotorSoftLimitConfig);
  }

  /**
   * Move the pivot motor based on the velocity
   *
   * @param velocity double, The velocity to move the pivot motor
   */
  public void movePivot(double velocity) {
    double deadband = 0.001;
    if (Math.abs(velocity) >= deadband) {
      climberMotor.setControl(vel_voltage.withVelocity(velocity));
    } else {
      this.stopMotor();
    }
  }

  /**
   * Set the pivot motor to a specific position
   *
   * @param pos double, The position to set the pivot motor to
   */
  public void setPivot(double pos) {
    climberMotor.setControl(vel_voltage.withVelocity(pos));
  }

  // public void recalibrateEncoders() {
  // PivotGlobalValues.offset = PivotGlobalValues.PIVOT_NEUTRAL_ANGLE -
  // getAbsoluteEncoder();
  // }

  /**
   * Initializes alarms for the climber motor. This method sets up an alert for a disconnected
   * climber motor and logs the connection status.
   */
  public void initializeAlarms() {
    climberMotorDisconnectedAlert =
        new Alert("Disconnected pivot motor " + CLIMBER_MOTOR_ID, kError);

    climberMotorDisconnectedAlert.set(!climberMotor.isConnected());

    logs("Disconnected climberMotor " + climberMotor.getDeviceID(), climberMotor.isConnected());
  }
}
