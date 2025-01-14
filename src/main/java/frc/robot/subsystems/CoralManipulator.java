// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters;
import frc.robot.utils.RobotParameters.CoralManipulatorParameters;
import frc.robot.utils.RobotParameters.MotorParameters;

public class CoralManipulator extends SubsystemBase {
  private final TalonFX coralManipulatorMotorUp;
  private final TalonFX coralManipulatorMotorDown;

  private SoftwareLimitSwitchConfigs upSoftLimitConfig;
  private SoftwareLimitSwitchConfigs downSoftLimitConfig;

  private final VoltageOut voltageOut;

  private final DigitalInput coralSensor;

  private boolean motorsRunning = false;

  /**
   * The Singleton instance of this CoralManipulatorSubsystem. Code should use the {@link
   * #getInstance()} method to get the single instance (rather than trying to construct an instance
   * of this class.)
   */
  private static final CoralManipulator INSTANCE = new CoralManipulator();

  /**
   * Returns the Singleton instance of this CoralManipulatorSubsystem. This static method should be
   * used, rather than the constructor, to get the single instance of this class. For example:
   * {@code CoralManipulatorSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static CoralManipulator getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this CoralManipulatorSubsystem. This constructor is private since
   * this class is a Singleton. Code should use the {@link #getInstance()} method to get the
   * singleton instance.
   */
  private CoralManipulator() {
    coralManipulatorMotorUp = new TalonFX(MotorParameters.CORAL_MANIPULATOR_MOTOR_UP_ID);
    coralManipulatorMotorDown = new TalonFX(MotorParameters.CORAL_MANIPULATOR_MOTOR_DOWN_ID);

    coralSensor = new DigitalInput(CoralManipulatorParameters.CORAL_SENSOR_ID);

    MotorOutputConfigs coralManipulatorConfigs = new MotorOutputConfigs();

    TalonFXConfigurator coralManipulatorUpConfigurator = coralManipulatorMotorUp.getConfigurator();
    TalonFXConfigurator coralManipulatorDownConfigurator =
        coralManipulatorMotorDown.getConfigurator();

    Slot0Configs coralManipulatorUpConfigs = new Slot0Configs();
    Slot0Configs coralManipulatorDownConfigs = new Slot0Configs();

    TalonFXConfiguration coralManipulatorUpConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration coralManipulatorDownConfiguration = new TalonFXConfiguration();

    coralManipulatorMotorUp.getConfigurator().apply(coralManipulatorUpConfiguration);
    coralManipulatorMotorDown.getConfigurator().apply(coralManipulatorDownConfiguration);

    coralManipulatorConfigs.NeutralMode = NeutralModeValue.Brake;
    coralManipulatorUpConfigurator.apply(coralManipulatorConfigs);
    coralManipulatorDownConfigurator.apply(coralManipulatorConfigs);

    coralManipulatorUpConfigs.kP = CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PIDV.getP();
    coralManipulatorUpConfigs.kI =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PIDV.getI();
    coralManipulatorUpConfigs.kD =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PIDV.getD();
    coralManipulatorUpConfigs.kV =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_UP_PIDV.getV();

    coralManipulatorDownConfigs.kP =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PIDV.getP();
    coralManipulatorDownConfigs.kI =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PIDV.getI();
    coralManipulatorDownConfigs.kD =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PIDV.getD();
    coralManipulatorDownConfigs.kV =
        RobotParameters.CoralManipulatorParameters.CORAL_MANIPULATOR_DOWN_PIDV.getV();

    coralManipulatorMotorUp.getConfigurator().apply(coralManipulatorUpConfigs);
    coralManipulatorMotorDown.getConfigurator().apply(coralManipulatorDownConfigs);

    CurrentLimitsConfigs upMotorCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs downMotorCurrentConfig = new CurrentLimitsConfigs();

    ClosedLoopRampsConfigs upMotorRampConfig = new ClosedLoopRampsConfigs();
    ClosedLoopRampsConfigs downMotorRampConfig = new ClosedLoopRampsConfigs();

    upMotorCurrentConfig.SupplyCurrentLimit = 100;
    upMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    upMotorCurrentConfig.StatorCurrentLimit = 100;
    upMotorCurrentConfig.StatorCurrentLimitEnable = true;

    downMotorCurrentConfig.SupplyCurrentLimit = 100;
    downMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    downMotorCurrentConfig.StatorCurrentLimit = 100;
    downMotorCurrentConfig.StatorCurrentLimitEnable = true;

    coralManipulatorMotorUp.getConfigurator().apply(upMotorCurrentConfig);
    coralManipulatorMotorDown.getConfigurator().apply(downMotorCurrentConfig);

    upMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    downMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    coralManipulatorMotorUp.getConfigurator().apply(upMotorRampConfig);
    coralManipulatorMotorDown.getConfigurator().apply(downMotorRampConfig);

    // on
    coralManipulatorUpConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    coralManipulatorDownConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // absoluteEncoder = new DigitalInput(9);

    VelocityTorqueCurrentFOC vel_voltage = new VelocityTorqueCurrentFOC(0);
    PositionTorqueCurrentFOC pos_reqest = new PositionTorqueCurrentFOC(0);
    voltageOut = new VoltageOut(0);

    new PositionDutyCycle(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    /**
     * If the coral sensor is triggered, set the hasPiece boolean to true. (hasPiece = true,
     * sensorDetect = true), motors spinning If the manipulator has a piece, but the sensor no
     * longer detects it, stop the motors. (hasPiece = true, sensorDetect = false), motors stop If
     * the manipulator should start, but the motors are not running, start the motors (hasPiece =
     * false, sensorDetect = false), motors spinning by setting if it has a piece to false, due to
     * the fact that the manipulator should not have a piece after the motors are started again.
     *
     * <p>The manipulator motors should be on by default, as per Aaron's request.
     */
    if (coralSensor.get()) {
      this.setHasPiece(true);
    }

    if (!coralSensor.get() && CoralManipulatorParameters.hasPiece) {
      if (this.motorsRunning) {
        // Stop the motors if the manipulator has a piece, but the sensor no longer detects it
        // May require a delay of 100-500ms to prevent the motors from stopping too early
        this.stopMotors();
      }
    } else {
      // Will run if the sensor doesn't detect the piece, and it doesn't have a piece concurrently
      // Will also run if the coral sensor detects a piece, and it has a piece
      if (!this.motorsRunning) {
        this.startMotors();
      }
    }
  }

  /** Stops the coral manipulator motors */
  public void stopMotors() {
    coralManipulatorMotorUp.stopMotor();
    coralManipulatorMotorDown.stopMotor();
    voltageOut.Output = -0.014;
    coralManipulatorMotorUp.setControl(voltageOut);
    coralManipulatorMotorDown.setControl(voltageOut);
    this.motorsRunning = false;
  }

  /** Starts the coral manipulator motors */
  public void startMotors() {
    voltageOut.Output = 0.014;
    coralManipulatorMotorUp.setControl(voltageOut);
    coralManipulatorMotorDown.setControl(voltageOut);
    this.motorsRunning = true;
  }

  public void setHasPiece(boolean hasPiece) {
    CoralManipulatorParameters.hasPiece = hasPiece;
  }
}
