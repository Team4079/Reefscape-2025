package frc.robot.subsystems;

// import static frc.robot.utils.Dash.log;
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
 * The PivotSubsystem class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this class
 * is created and shared across the entire robot code.
 */
public class AlgaeManipulator extends SubsystemBase {
  /** Creates a new end effector. */
  private final TalonFX algaeManipulatorMotor;

  private final SoftwareLimitSwitchConfigs algaeManipulatorMotorSoftLimitConfig;

  private final VelocityVoltage vel_voltage;

  private final VoltageOut voltageOut;

  private Alert algaeManipulatorMotorDisconnectedAlert;

  // private double absPos = 0;

  /**
   * The Singleton instance of this PivotSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final AlgaeManipulator INSTANCE = new AlgaeManipulator();

  /**
   * Returns the Singleton instance of this PivotSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * armSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static AlgaeManipulator getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this armSubsystem. This constructor is private since this class is a
   * Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private AlgaeManipulator() {
    algaeManipulatorMotor = new TalonFX(ALGAE_MANIPULATOR_MOTOR_ID);

    MotorOutputConfigs algaeManipulatorMotorOutputConfigs = new MotorOutputConfigs();

    Slot0Configs algaeManipulatorMotorConfigs = new Slot0Configs();

    // Configurations
    TalonFXConfiguration algaeManipulatorMotorConfiguration = new TalonFXConfiguration();

    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorConfiguration);
    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorConfigs);

    algaeManipulatorMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    algaeManipulatorMotorConfigs.kP = AlgaeManipulatorParameters.ALGAE_MANIPULATOR_PID_P;
    algaeManipulatorMotorConfigs.kI = AlgaeManipulatorParameters.ALGAE_MANIPULATOR_PID_I;
    algaeManipulatorMotorConfigs.kD = AlgaeManipulatorParameters.ALGAE_MANIPULATOR_PID_D;
    algaeManipulatorMotorConfigs.kV = AlgaeManipulatorParameters.ALGAE_MANIPULATOR_PID_V;
    // armConfigs.kF = armConstants.arm_PID_F;

    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorConfigs);

    CurrentLimitsConfigs algaeManipulatorMotorCurrentConfig = new CurrentLimitsConfigs();
    ClosedLoopRampsConfigs algaeManipulatorMotorRampConfig = new ClosedLoopRampsConfigs();
    algaeManipulatorMotorSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    algaeManipulatorMotorCurrentConfig.SupplyCurrentLimit = 100;
    algaeManipulatorMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    algaeManipulatorMotorCurrentConfig.StatorCurrentLimit = 100;
    algaeManipulatorMotorCurrentConfig.StatorCurrentLimitEnable = true;

    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorCurrentConfig);

    algaeManipulatorMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorRampConfig);

    algaeManipulatorMotorSoftLimitConfig.ForwardSoftLimitEnable = true;
    algaeManipulatorMotorSoftLimitConfig.ReverseSoftLimitEnable = true;
    algaeManipulatorMotorSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    algaeManipulatorMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    algaeManipulatorMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    algaeManipulatorMotorConfiguration.SoftwareLimitSwitch = algaeManipulatorMotorSoftLimitConfig;

    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    PositionVoltage pos_voltage = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    algaeManipulatorMotor.setPosition(0);

    initializeAlarms();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    logs("End Effector Motor Position", algaeManipulatorMotor.getPosition().getValueAsDouble());
  }

  /** Stops the arm motor */
  public void stopMotor() {
    algaeManipulatorMotor.stopMotor();
    voltageOut.Output = -0.014;
    algaeManipulatorMotor.setControl(voltageOut);
  }

  /**
   * Get the position of the end effector motor
   *
   * @return double, the position of the end effector motor
   */
  public double getPivotPosValue() {
    return algaeManipulatorMotor.getPosition().getValue().magnitude();
  }

  /** Soft resets the encoders on the end effector motors */
  public void resetEncoders() {
    algaeManipulatorMotor.setPosition(0);
  }

  /** Toggles the soft stop for the end effector motor */
  public void toggleSoftStop() {
    AlgaeManipulatorParameters.isSoftLimitEnabled = !AlgaeManipulatorParameters.isSoftLimitEnabled;
    algaeManipulatorMotorSoftLimitConfig.ReverseSoftLimitEnable =
        AlgaeManipulatorParameters.isSoftLimitEnabled;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    algaeManipulatorMotorSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    algaeManipulatorMotor.getConfigurator().apply(algaeManipulatorMotorSoftLimitConfig);
  }

  /**
   * Move the end effector motor based on the velocity
   *
   * @param velocity double, The velocity to move the end effector motor
   */
  public void moveArm(double velocity) {
    final double deadband = 0.001;
    if (Math.abs(velocity) >= deadband) {
      algaeManipulatorMotor.setControl(vel_voltage.withVelocity(velocity));
    } else {
      this.stopMotor();
    }
  }

  public void initializeAlarms() {
    algaeManipulatorMotorDisconnectedAlert =
        new Alert("Disconnected end effector motor " + ALGAE_MANIPULATOR_MOTOR_ID, kError);

    algaeManipulatorMotorDisconnectedAlert.set(!algaeManipulatorMotor.isConnected());

    logs(
        "Disconnected algaeManipulatorMotor " + algaeManipulatorMotor.getDeviceID(),
        algaeManipulatorMotor.isConnected());
  }
}
