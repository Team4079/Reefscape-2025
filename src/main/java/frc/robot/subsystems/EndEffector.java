package frc.robot.subsystems;

import static frc.robot.utils.Register.Dash.*;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.*;

/**
 * The EndEffectorSubsystem class is a subsystem that interfaces with the arm system to provide
 * control over the arm motors. This subsystem is a Singleton, meaning that only one instance of
 * this class is created and shared across the entire robot code.
 */
public class EndEffector extends SubsystemBase {
  /** Creates a new end effector. */
  private final TalonFX endEffectorMotor;

  private final SoftwareLimitSwitchConfigs endEffectorMotorSoftLimitConfig;

  private final VelocityVoltage vel_voltage;

  private final VoltageOut voltageOut;

  // private double absPos = 0;

  /**
   * The Singleton instance of this EndEffectorSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final EndEffector INSTANCE = new EndEffector();

  /**
   * Returns the Singleton instance of this EndEffectorSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * armSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static EndEffector getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this armSubsystem. This constructor is private since this class is a
   * Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private EndEffector() {
    endEffectorMotor = new TalonFX(MotorParameters.END_EFFECTOR_MOTOR_ID);

    MotorOutputConfigs endEffectorMotorOutputConfigs = new MotorOutputConfigs();

    Slot0Configs endEffectorMotorConfigs = new Slot0Configs();

    // Configurations
    TalonFXConfiguration endEffectorMotorConfiguration = new TalonFXConfiguration();

    endEffectorMotor.getConfigurator().apply(endEffectorMotorConfiguration);
    endEffectorMotor.getConfigurator().apply(endEffectorMotorConfigs);

    endEffectorMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    endEffectorMotorConfigs.kP = EndEffectorParameters.END_EFFECTOR_PID_P;
    endEffectorMotorConfigs.kI = EndEffectorParameters.END_EFFECTOR_PID_I;
    endEffectorMotorConfigs.kD = EndEffectorParameters.END_EFFECTOR_PID_D;
    endEffectorMotorConfigs.kV = EndEffectorParameters.END_EFFECTOR_PID_V;
    // armConfigs.kF = armConstants.arm_PID_F;

    endEffectorMotor.getConfigurator().apply(endEffectorMotorConfigs);

    CurrentLimitsConfigs endEffectorMotorCurrentConfig = new CurrentLimitsConfigs();
    ClosedLoopRampsConfigs endEffectorMotorRampConfig = new ClosedLoopRampsConfigs();
    endEffectorMotorSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    endEffectorMotorCurrentConfig.SupplyCurrentLimit = 100;
    endEffectorMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    endEffectorMotorCurrentConfig.StatorCurrentLimit = 100;
    endEffectorMotorCurrentConfig.StatorCurrentLimitEnable = true;

    endEffectorMotor.getConfigurator().apply(endEffectorMotorCurrentConfig);

    endEffectorMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    endEffectorMotor.getConfigurator().apply(endEffectorMotorRampConfig);

    endEffectorMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
    endEffectorMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
    endEffectorMotorSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    endEffectorMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    endEffectorMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    endEffectorMotorConfiguration.SoftwareLimitSwitch = endEffectorMotorSoftLimitConfig;

    endEffectorMotor.getConfigurator().apply(endEffectorMotorSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    PositionVoltage pos_voltage = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    endEffectorMotor.setPosition(0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    log("End Effector Motor Position", endEffectorMotor.getPosition().getValueAsDouble());
  }

  /** Stops the arm motor */
  public void stopMotor() {
    endEffectorMotor.stopMotor();
    voltageOut.Output = -0.014;
    endEffectorMotor.setControl(voltageOut);
  }

  /**
   * Get the position of the end effector motor
   *
   * @return double, the position of the end effector motor
   */
  public double getEndEffectorPosValue() {
    return endEffectorMotor.getPosition().getValue().magnitude();
  }

  /** Soft resets the encoders on the end effector motors */
  public void resetEncoders() {
    endEffectorMotor.setPosition(0);
  }

  /** Toggles the soft stop for the end effector motor */
  public void toggleSoftStop() {
    EndEffectorParameters.isSoftLimitEnabled = !EndEffectorParameters.isSoftLimitEnabled;
    endEffectorMotorSoftLimitConfig.ReverseSoftLimitEnable =
        EndEffectorParameters.isSoftLimitEnabled;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    endEffectorMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    endEffectorMotor.getConfigurator().apply(endEffectorMotorSoftLimitConfig);
  }

  /**
   * Move the end effector motor based on the velocity
   *
   * @param velocity double, The velocity to move the end effector motor
   */
  public void moveArm(double velocity) {
    final double deadband = 0.001;
    if (Math.abs(velocity) >= deadband) {
      endEffectorMotor.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));
    } else {
      this.stopMotor();
    }
  }
}
