package frc.robot.subsystems;

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
import org.littletonrobotics.junction.Logger;

/**
 * The ArmSubsystem class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this
 * class is created and shared across the entire robot code.
 */
public class ArmSubsystem extends SubsystemBase {
  /** Creates a new arm. */
  private TalonFX armMotor;

  // Configurations
  private TalonFXConfiguration armMotorConfiguration;
  private Slot0Configs armConfigs;
  private MotorOutputConfigs armOutputConfigs;
  private CurrentLimitsConfigs armMotorCurrentConfig;
  private ClosedLoopRampsConfigs armMotorRampConfig;
  private SoftwareLimitSwitchConfigs armMotorSoftLimitConfig;

  private PositionVoltage pos_reqest;
  private VelocityVoltage vel_voltage;

  private VoltageOut voltageOut;

  private double deadband = 0.001;

  // private double absPos = 0;

  /**
   * The Singleton instance of this ArmSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final ArmSubsystem INSTANCE = new ArmSubsystem();

  /**
   * Returns the Singleton instance of this armSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * armSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static ArmSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this armSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private ArmSubsystem() {
    armMotor = new TalonFX(MotorParameters.ARM_MOTOR_ID);

    armOutputConfigs = new MotorOutputConfigs();

    armConfigs = new Slot0Configs();

    armMotorConfiguration = new TalonFXConfiguration();

    armMotor.getConfigurator().apply(armMotorConfiguration);
    armMotor.getConfigurator().apply(armConfigs);

    armOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    armConfigs.kP = ArmParameters.ARM_PID_P;
    armConfigs.kI = ArmParameters.ARM_PID_I;
    armConfigs.kD = ArmParameters.ARM_PID_D;
    armConfigs.kV = ArmParameters.ARM_PID_V;
    // armConfigs.kF = armConstants.arm_PID_F;

    armMotor.getConfigurator().apply(armConfigs);

    armMotorCurrentConfig = new CurrentLimitsConfigs();
    armMotorRampConfig = new ClosedLoopRampsConfigs();
    armMotorSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    armMotorCurrentConfig.SupplyCurrentLimit = 100;
    armMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    armMotorCurrentConfig.StatorCurrentLimit = 100;
    armMotorCurrentConfig.StatorCurrentLimitEnable = true;

    armMotor.getConfigurator().apply(armMotorCurrentConfig);

    armMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    armMotor.getConfigurator().apply(armMotorRampConfig);

    armMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
    armMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
    armMotorSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    armMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    armMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    armMotorConfiguration.SoftwareLimitSwitch = armMotorSoftLimitConfig;

    armMotor.getConfigurator().apply(armMotorSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    pos_reqest = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    armMotor.setPosition(0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    Logger.recordOutput("arm Motor Position", armMotor.getPosition().getValueAsDouble());
    Logger.recordOutput("arm SoftLimit", this.getSoftLimit());
  }

  /** Stops the arm motor */
  public void stopMotor() {
    armMotor.stopMotor();
    voltageOut.Output = -0.014;
    armMotor.setControl(voltageOut);
  }

  /**
   * Get the position of the elevator motor
   *
   * @return double, the position of the elevator motor
   */
  public double getArmPosValue() {
    return armMotor.getPosition().getValue().magnitude();
  }

  /**
   * Soft resets the encoders on the elevator motors
   *
   * @return void
   */
  public void resetEncoders() {
    armMotor.setPosition(0);
  }

  /**
   * Toggles the soft stop for the elevator motor
   *
   * @return void
   */
  public void toggleSoftStop() {
    ArmParameters.SOFT_LIMIT_ENABLED = !ArmParameters.SOFT_LIMIT_ENABLED;
    armMotorSoftLimitConfig.ReverseSoftLimitEnable = ArmParameters.SOFT_LIMIT_ENABLED;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    armMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    armMotor.getConfigurator().apply(armMotorSoftLimitConfig);
  }

  /**
   * Move the arm motor based on the velocity
   *
   * @param velocity double, The velocity to move the arm motor
   * @return void
   */
  public void moveArm(double velocity) {
    if (Math.abs(velocity) >= deadband) {
      armMotor.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));
    } else {
      this.stopMotor();
    }
  }

  /**
   * Toggles the soft limit for the elevator motor
   *
   * @return void
   */
  public void toggleLimit() {
    ArmParameters.IS_SOFTLIMIT = !ArmParameters.IS_SOFTLIMIT;
  }
  
  /**
   * Get the soft limit for the arm motor
   *
   * @return boolean, The soft limit state for the arm motor
   */
  public boolean getSoftLimit() {
    return ArmParameters.IS_SOFTLIMIT;
  }
}
