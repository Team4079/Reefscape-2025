package frc.robot.subsystems;

import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.algaeIntaking;
import static frc.robot.utils.RobotParameters.CoralManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.emu.CoralState.*;
import static frc.robot.utils.pingu.LogPingu.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.CoralManipulatorParameters;
import frc.robot.utils.pingu.*;

public class Coral extends SubsystemBase {
  private final TalonFX coralFeederMotor;
  private final TalonFX coralScoreMotor;
  private final TalonFX starFeederMotor;

  private final VoltageOut voltageOut;
  private final VoltageOut voltageOutFeeder;

  private final DigitalInput coralSensor;

  private boolean motorsRunning = false;

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
    starFeederMotor = new TalonFX(STAR_FEEDER_ID);

    coralSensor = new DigitalInput(CoralManipulatorParameters.CORAL_SENSOR_ID);

    TalonFXConfiguration coralFeederConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration coralScoreConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration starFeederConfiguration = new TalonFXConfiguration();

    coralFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    coralScoreConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    starFeederConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    coralFeederConfiguration.Slot0.kP = CORAL_FEEDER_PINGU.getP();
    coralFeederConfiguration.Slot0.kI = CORAL_FEEDER_PINGU.getI();
    coralFeederConfiguration.Slot0.kD = CORAL_FEEDER_PINGU.getD();
    coralFeederConfiguration.Slot0.kV = CORAL_FEEDER_PINGU.getV();

    coralFeederMotor.getConfigurator().apply(coralFeederConfiguration);
    coralScoreMotor.getConfigurator().apply(coralFeederConfiguration);

    CurrentLimitsConfigs upMotorCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs coralScoreCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs starFeederCurrentConfig = new CurrentLimitsConfigs();

    upMotorCurrentConfig.SupplyCurrentLimit = 40;
    upMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    upMotorCurrentConfig.StatorCurrentLimit = 40;
    upMotorCurrentConfig.StatorCurrentLimitEnable = true;

    coralScoreCurrentConfig.SupplyCurrentLimit = 40;
    coralScoreCurrentConfig.SupplyCurrentLimitEnable = true;
    coralScoreCurrentConfig.StatorCurrentLimit = 40;
    coralScoreCurrentConfig.StatorCurrentLimitEnable = true;

    starFeederCurrentConfig.SupplyCurrentLimit = 40;
    starFeederCurrentConfig.SupplyCurrentLimitEnable = true;
    starFeederCurrentConfig.StatorCurrentLimit = 40;
    starFeederCurrentConfig.StatorCurrentLimitEnable = true;

    coralFeederMotor.getConfigurator().apply(upMotorCurrentConfig);
    coralScoreMotor.getConfigurator().apply(coralScoreCurrentConfig);
    starFeederMotor.getConfigurator().apply(starFeederCurrentConfig);

    // on
    coralFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    coralScoreConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    starFeederConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    coralScoreMotor.getConfigurator().apply(coralScoreConfiguration);
    coralFeederMotor.getConfigurator().apply(coralFeederConfiguration);
    starFeederMotor.getConfigurator().apply(starFeederConfiguration);

    voltageOut = new VoltageOut(0);
    voltageOutFeeder = new VoltageOut(0);

    AlertPingu.add(coralFeederMotor, "coral feeder");
    AlertPingu.add(coralScoreMotor, "coral score");
    AlertPingu.add(starFeederMotor, "Star Feeder Motor");
  }

  /**
   * If the coral sensor is triggered, set the hasPiece boolean to true. (hasPiece = true,
   * sensorDetect = true), motors spinning If the manipulator has a piece, but the sensor no longer
   * detects it, stop the motors. (hasPiece = true, sensorDetect = false), motors stop If the
   * manipulator should start, but the motors are not running, start the motors (hasPiece = false,
   * sensorDetect = false), motors spinning by setting if it has a piece to false, due to the fact
   * that the manipulator should not have a piece after the motors are started again.
   *
   * <p>The manipulator motors should be on by default, as per Aaron's request.
   */
  @Override
  public void periodic() {
    voltageOutFeeder.Output = 5;
    coralFeederMotor.setControl(voltageOutFeeder);
    starFeederMotor.setControl(voltageOutFeeder);

    if (!algaeIntaking && !coralScoring) {
      if (!getCoralSensor() && !hasPiece) {
        coralState = CORAL_INTAKE;
      } else if (getCoralSensor() && !hasPiece) {
        // Stop the motors if the manipulator has a piece, but the sensor no longer
        // detects it
        coralState = CORAL_SLOW;
        setHasPiece(true);
      } else if (!getCoralSensor() && hasPiece) {
        coralState = CORAL_HOLD;
      } else {
        coralState = CORAL_SLOW;
      }
    }

    logs(
        () -> {
          log("Coral/Coral Sensor", getCoralSensor());
          log("Coral/Has Piece", hasPiece);
          log("Coral/Coral Scoring", coralScoring);
          log("Coral/motorsRunning", this.motorsRunning);
          log("Coral/Coral State", coralState.toString());
        });

    coralState.block.run();
  }

  /** Stops the coral manipulator motors */
  public void stopMotors() {
    //    coralFeederMotor.stopMotor();
    coralScoreMotor.stopMotor();
    this.motorsRunning = false;
  }

  /** Starts the coral manipulator motors */
  public void startCoralIntake() {
    voltageOut.Output = 5.0;
    coralScoreMotor.setControl(voltageOut);
    this.setHasPiece(false);
    //    isCoralIntaking = true;
  }

  /** Scores the coral motors */
  public void scoreCoral() {
    voltageOut.Output = 3.0;
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
    this.setHasPiece(false);
  }

  /** Starts the coral manipulator motors */
  public void slowCoralIntake() {
    coralScoreMotor.setControl(VoltagePingu.setOutput(4.0));
    //    coralFeederMotor.setControl(VoltagePingu.setOutput(4.0));
    this.setHasPiece(true);
  }

  /** Starts the coral manipulator motors */
  public void reverseMotors() {
    coralFeederMotor.setControl(VoltagePingu.setOutput(-4.5));
    coralScoreMotor.setControl(VoltagePingu.setOutput(-4.5));
    starFeederMotor.setControl(VoltagePingu.setOutput(-4.5));
    this.motorsRunning = true;
  }

  /**
   * Activates the algae intake process. This method stops the current motors, sets the voltage
   * output to 4.5, and starts the coral score motor to intake algae.
   */
  public void algaeIntake() {
    this.stopMotors();
    voltageOut.Output = -4.5;
    coralScoreMotor.setControl(voltageOut);
    this.motorsRunning = true;
  }

  /**
   * Sets the state of whether the manipulator has a piece.
   *
   * @param hasPiece true if the manipulator has a piece, false otherwise
   */
  public void setHasPiece(boolean hasPiece) {
    CoralManipulatorParameters.hasPiece = hasPiece;
  }

  /**
   * Sets the voltage output to -3.0 and controls the coral score motor
   * to slow down the algae scoring process.
   */
  public void slowAlgaeScoreMotors() {
    voltageOut.Output = -3.0;
    coralScoreMotor.setControl(voltageOut);
  }

  /**
   * Ejects the algae by setting the voltage output to 4.0 and controlling the coral score motor.
   */
  public void ejectAlgae() {
    voltageOut.Output = 4.0;
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
}
