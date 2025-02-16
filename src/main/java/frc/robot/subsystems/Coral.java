// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.Alert.AlertType.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;

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
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.CoralManipulatorParameters;

public class Coral extends SubsystemBase {
  private final TalonFX coralFeederMotor;

  private SoftwareLimitSwitchConfigs upSoftLimitConfig;
  private SoftwareLimitSwitchConfigs downSoftLimitConfig;

  private final VoltageOut voltageOut;

  private final DigitalInput coralSensor;

  private boolean motorsRunning = false;

  private Alert coralFeederDisconnectedAlert;

  /**
   * The Singleton instance of this CoralManipulatorSubsystem. Code should use the {@link
   * #getInstance()} method to get the single instance (rather than trying to construct an instance
   * of this class.)
   */
  private static final Coral INSTANCE = new Coral();

  /**
   * Returns the Singleton instance of this CoralManipulatorSubsystem. This static method should be
   * used, rather than the constructor, to get the single instance of this class. For example:
   * {@code CoralManipulatorSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Coral getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this CoralManipulatorSubsystem. This constructor is private since
   * this class is a Singleton. Code should use the {@link #getInstance()} method to get the
   * singleton instance.
   */
  private Coral() {
    coralFeederMotor = new TalonFX(CORAL_FEEDER_ID);

    coralSensor = new DigitalInput(CoralManipulatorParameters.CORAL_SENSOR_ID);

    MotorOutputConfigs coralManipulatorConfigs = new MotorOutputConfigs();

    TalonFXConfigurator coralManipulatorUpConfigurator = coralFeederMotor.getConfigurator();

    Slot0Configs coralFeederConfigs = new Slot0Configs();

    TalonFXConfiguration coralFeederConfiguration = new TalonFXConfiguration();

    coralManipulatorConfigs.NeutralMode = NeutralModeValue.Brake;
    coralManipulatorUpConfigurator.apply(coralManipulatorConfigs);

    coralFeederConfigs.kP = CORAL_FEEDER_PINGU.getP();
    coralFeederConfigs.kI = CORAL_FEEDER_PINGU.getI();
    coralFeederConfigs.kD = CORAL_FEEDER_PINGU.getD();
    coralFeederConfigs.kV = CORAL_FEEDER_PINGU.getV();

    coralFeederMotor.getConfigurator().apply(coralFeederConfigs);

    CurrentLimitsConfigs upMotorCurrentConfig = new CurrentLimitsConfigs();

    ClosedLoopRampsConfigs upMotorRampConfig = new ClosedLoopRampsConfigs();

    upMotorCurrentConfig.SupplyCurrentLimit = 40;
    upMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    upMotorCurrentConfig.StatorCurrentLimit = 40;
    upMotorCurrentConfig.StatorCurrentLimitEnable = true;

    coralFeederMotor.getConfigurator().apply(upMotorCurrentConfig);

    upMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    coralFeederMotor.getConfigurator().apply(upMotorRampConfig);
    // on
    coralFeederConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    coralFeederMotor.getConfigurator().apply(coralFeederConfiguration);
    // absoluteEncoder = new DigitalInput(9);

    VelocityTorqueCurrentFOC vel_voltage = new VelocityTorqueCurrentFOC(0);
    PositionTorqueCurrentFOC pos_reqest = new PositionTorqueCurrentFOC(0);
    voltageOut = new VoltageOut(0);

    new PositionDutyCycle(0);

    coralFeederDisconnectedAlert =
        new Alert("Disconnected coral feeder motor " + CORAL_FEEDER_ID, kError);
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
        // Stop the motors if the manipulator has a piece, but the sensor no longer
        // detects it
        // May require a delay of 100-500ms to prevent the motors from stopping too
        // early
        this.stopMotors();
      }
    } else {
      // Will run if the sensor doesn't detect the piece, and it doesn't have a piece
      // concurrently
      // Will also run if the coral sensor detects a piece, and it has a piece
      if (!this.motorsRunning) {
        this.startMotors();
      }
    }

    coralFeederDisconnectedAlert.set(!coralFeederMotor.isConnected());

    // No need to call logs as alert should put on NT automatically
//    logs(
//        () -> {
//          log(
//              "Disconnected coralFeederMotor " + coralFeederMotor.getDeviceID(),
//              coralFeederMotor.isConnected());
//        });
  }

  /** Stops the coral manipulator motors */
  public void stopMotors() {
    coralFeederMotor.stopMotor();
    voltageOut.Output = 0.0;
    coralFeederMotor.setControl(voltageOut);
    this.motorsRunning = false;
  }

  /** Starts the coral manipulator motors */
  public void startMotors() {
    voltageOut.Output = 0.014;
    coralFeederMotor.setControl(voltageOut);
    this.motorsRunning = true;
  }

  public void setHasPiece(boolean hasPiece) {
    CoralManipulatorParameters.hasPiece = hasPiece;
  }
}
