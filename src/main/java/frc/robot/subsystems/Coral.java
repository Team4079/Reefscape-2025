// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.Alert.AlertType.*;
import static frc.robot.utils.CoralState.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CoralState;
import frc.robot.utils.RobotParameters.CoralManipulatorParameters;

public class Coral extends SubsystemBase {
  private final TalonFX coralFeederMotor;
  private final TalonFX coralScoreMotor;

  private final VoltageOut voltageOut;

  private final DigitalInput coralSensor;

  private boolean motorsRunning = false;

  private final Alert coralFeederDisconnectedAlert;
  private final Alert coralScoreDisconnectedAlert;

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
    coralScoreMotor = new TalonFX(CORAL_SCORE_ID);

    coralSensor = new DigitalInput(CoralManipulatorParameters.CORAL_SENSOR_ID);

    TalonFXConfiguration coralFeederConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration coralScoreConfiguration = new TalonFXConfiguration();

    coralFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    coralScoreConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    coralFeederConfiguration.Slot0.kP = CORAL_FEEDER_PINGU.getP();
    coralFeederConfiguration.Slot0.kI = CORAL_FEEDER_PINGU.getI();
    coralFeederConfiguration.Slot0.kD = CORAL_FEEDER_PINGU.getD();
    coralFeederConfiguration.Slot0.kV = CORAL_FEEDER_PINGU.getV();

    coralFeederMotor.getConfigurator().apply(coralFeederConfiguration);
    coralScoreMotor.getConfigurator().apply(coralFeederConfiguration);

    CurrentLimitsConfigs upMotorCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs coralScoreCurrentConfig = new CurrentLimitsConfigs();

    upMotorCurrentConfig.SupplyCurrentLimit = 40;
    upMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    upMotorCurrentConfig.StatorCurrentLimit = 40;
    upMotorCurrentConfig.StatorCurrentLimitEnable = true;

    coralScoreCurrentConfig.SupplyCurrentLimit = 40;
    coralScoreCurrentConfig.SupplyCurrentLimitEnable = true;
    coralScoreCurrentConfig.StatorCurrentLimit = 40;
    coralScoreCurrentConfig.StatorCurrentLimitEnable = true;

    coralFeederMotor.getConfigurator().apply(upMotorCurrentConfig);
    coralScoreMotor.getConfigurator().apply(coralScoreCurrentConfig);

    // on
    coralFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    coralScoreConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    coralScoreMotor.getConfigurator().apply(coralScoreConfiguration);
    coralFeederMotor.getConfigurator().apply(coralFeederConfiguration);

    voltageOut = new VoltageOut(0);

    coralFeederDisconnectedAlert =
        new Alert("Disconnected coral feeder motor " + CORAL_FEEDER_ID, kError);
    coralScoreDisconnectedAlert =
        new Alert("Disconnected coral score motor " + CORAL_SCORE_ID, kError);
  }

  /**
   * If the coral sensor is triggered, set the hasPiece boolean to true. (hasPiece = true,
   * sensorDetect = true), motors spinning If the manipulator has a piece, but the sensor no
   * longer detects it, stop the motors. (hasPiece = true, sensorDetect = false), motors stop If
   * the manipulator should start, but the motors are not running, start the motors (hasPiece =
   * false, sensorDetect = false), motors spinning by setting if it has a piece to false, due to
   * the fact that the manipulator should not have a piece after the motors are started again.
   * <p>
   * TODO: Make it so a button can disable and enable the coral
   * <p>
   * The manipulator motors should be on by default, as per Aaron's request.
   */
  @Override
  public void periodic() {
    if (isCoralIntaking && !algaeIntaking) {
      if (!getCoralSensor() && !hasPiece) {
        coralState = CORAL_INTAKE;
      } else if (getCoralSensor() && !hasPiece) {
        // Stop the motors if the manipulator has a piece, but the sensor no longer
        // detects it
        coralState = CORAL_SLOW;
        setHasPiece(true);
      } else if (!getCoralSensor() && hasPiece) {
        coralState = CORAL_HOLD;
        isCoralIntaking = false;
      }
    }
    logs(
        () -> {
          log("Coral/isCoralIntaking", isCoralIntaking);
          log("Coral/Coral Sensor", getCoralSensor());
          log("Coral/Has Piece", hasPiece);
          log("Coral/motorsRunning", this.motorsRunning);
          log("Coral/Coral State", coralState.toString());
        });

    coralFeederDisconnectedAlert.set(!coralFeederMotor.isConnected());
    coralScoreDisconnectedAlert.set(!coralScoreMotor.isConnected());

    switch (coralState) {
      case CORAL_INTAKE:
        this.startCoralIntake();
        break;
      case CORAL_HOLD:
        this.stopMotors();
        break;
      case CORAL_SLOW:
        this.slowCoralIntake();
        break;
      case CORAL_RELEASE:
        this.scoreCoral();
        break;
      case ALGAE_INTAKE:
        this.algaeIntake();
        break;
      case ALGAE_HOLD:
        this.slowAlgaeScoreMotors();
        break;
      case ALGAE_RELEASE:
        this.ejectAlgae();
        break;
    }
  }

  /** Stops the coral manipulator motors */
  public void stopMotors() {
    coralFeederMotor.stopMotor();
    coralScoreMotor.stopMotor();
    this.motorsRunning = false;
  }

  /** Starts the coral manipulator motors */
  public void startCoralIntake() {
    voltageOut.Output = 5.0;
    coralFeederMotor.setControl(voltageOut);
    coralScoreMotor.setControl(voltageOut);
    this.setHasPiece(false);
    isCoralIntaking = true;
  }

  public void poopOut() {
    // TODO: Implement
  }

  /** Scores the coral motors */
  public void scoreCoral() {
    voltageOut.Output = 5.0;
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
    this.setHasPiece(false);
  }

  /** Starts the coral manipulator motors */
  public void slowCoralIntake() {
    voltageOut.Output = 3.75;
    coralScoreMotor.setControl(voltageOut);
    coralFeederMotor.setControl(voltageOut);
    this.setHasPiece(true);
  }

  /** Starts the coral manipulator motors */
  public void reverseMotors() {
    voltageOut.Output = -4.5;
    coralFeederMotor.setControl(voltageOut);
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
  }

  public void algaeIntake() {
    this.stopMotors();
    voltageOut.Output = 4.5;
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
  }

  public void setHasPiece(boolean hasPiece) {
    CoralManipulatorParameters.hasPiece = hasPiece;
  }

  /** Spins the intake to intake algae */
  public void spinIntakeAlgae() {
    voltageOut.Output = 4.0;
    coralScoreMotor.setControl(voltageOut);
  }

  public void slowAlgaeScoreMotors() {
    voltageOut.Output = 0.5;
    coralScoreMotor.setControl(voltageOut);
  }

  /** Ejects the algae */
  public void ejectAlgae() {
    voltageOut.Output = -4.0;
    coralScoreMotor.setControl(voltageOut);
  }

  /**
   * Gets the state of the coral manipulator
   *
   * @return The state of the coral manipulator
   */
  public boolean getCoralSensor() {
    return !coralSensor.get();
  }

  /**
   * Sets the state of the coral manipulator
   *
   * @param state The state to set the coral manipulator to
   */
  public void setState(CoralState state) {
    coralState = state;
  }
}
