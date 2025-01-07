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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.ElevatorParameters;
import frc.robot.utils.RobotParameters.MotorParameters;
import frc.robot.utils.RobotParameters.SwerveParameters.Thresholds;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private TalonFX elevatorMotorLeft;
  private TalonFX elevatorMotorRight;

  private TalonFXConfigurator elevatorLeftConfigurator;
  private TalonFXConfigurator elevatorRightConfigurator;

  private TalonFXConfiguration elevatorLeftConfiguration;
  private TalonFXConfiguration elevatorRightConfiguration;

  private Slot0Configs elevatorLeftConfigs;
  private Slot0Configs elevatorRightConfigs;

  private PositionVoltage pos_reqest;
  private VelocityVoltage vel_voltage;

  private MotorOutputConfigs elevatorConfigs;

  private CurrentLimitsConfigs leftMotorCurrentConfig;
  private CurrentLimitsConfigs rightMotorCurrentConfig;

  private ClosedLoopRampsConfigs leftMotorRampConfig;
  private ClosedLoopRampsConfigs rightMotorRampConfig;

  private SoftwareLimitSwitchConfigs leftSoftLimitConfig;
  private SoftwareLimitSwitchConfigs rightSoftLimitConfig;

  private VoltageOut voltageOut;

  private double deadband = 0.001;

  public Elevator() {
    elevatorMotorLeft = new TalonFX(MotorParameters.ELEVATOR_MOTOR_LEFT_ID);
    elevatorMotorRight = new TalonFX(MotorParameters.ELEVATOR_MOTOR_RIGHT_ID);

    elevatorConfigs = new MotorOutputConfigs();

    elevatorLeftConfigurator = elevatorMotorLeft.getConfigurator();
    elevatorRightConfigurator = elevatorMotorRight.getConfigurator();

    elevatorLeftConfigs = new Slot0Configs();
    elevatorRightConfigs = new Slot0Configs();

    elevatorLeftConfiguration = new TalonFXConfiguration();
    elevatorRightConfiguration = new TalonFXConfiguration();

    elevatorMotorLeft.getConfigurator().apply(elevatorLeftConfiguration);
    elevatorMotorRight.getConfigurator().apply(elevatorRightConfiguration);

    elevatorConfigs.NeutralMode = NeutralModeValue.Brake;
    elevatorLeftConfigurator.apply(elevatorConfigs);
    elevatorRightConfigurator.apply(elevatorConfigs);

    elevatorLeftConfigs.kP = ElevatorParameters.ELEVATOR_PID_LEFT_P;
    elevatorLeftConfigs.kI = ElevatorParameters.ELEVATOR_PID_LEFT_I;
    elevatorLeftConfigs.kD = ElevatorParameters.ELEVATOR_PID_LEFT_D;
    elevatorLeftConfigs.kV = ElevatorParameters.ELEVATOR_PID_LEFT_V;

    elevatorRightConfigs.kP = ElevatorParameters.ELEVATOR_PID_RIGHT_P;
    elevatorRightConfigs.kI = ElevatorParameters.ELEVATOR_PID_RIGHT_I;
    elevatorRightConfigs.kD = ElevatorParameters.ELEVATOR_PID_RIGHT_D;
    elevatorRightConfigs.kV = ElevatorParameters.ELEVATOR_PID_RIGHT_V;

    elevatorMotorLeft.getConfigurator().apply(elevatorLeftConfigs);
    elevatorMotorRight.getConfigurator().apply(elevatorRightConfigs);

    leftMotorCurrentConfig = new CurrentLimitsConfigs();
    rightMotorCurrentConfig = new CurrentLimitsConfigs();

    leftMotorRampConfig = new ClosedLoopRampsConfigs();
    rightMotorRampConfig = new ClosedLoopRampsConfigs();

    leftSoftLimitConfig = new SoftwareLimitSwitchConfigs();
    rightSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    leftMotorCurrentConfig.SupplyCurrentLimit = 100;
    leftMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    leftMotorCurrentConfig.StatorCurrentLimit = 100;
    leftMotorCurrentConfig.StatorCurrentLimitEnable = true;

    rightMotorCurrentConfig.SupplyCurrentLimit = 100;
    rightMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    rightMotorCurrentConfig.StatorCurrentLimit = 100;
    rightMotorCurrentConfig.StatorCurrentLimitEnable = true;

    elevatorMotorLeft.getConfigurator().apply(leftMotorCurrentConfig);
    elevatorMotorRight.getConfigurator().apply(rightMotorCurrentConfig);

    leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    elevatorMotorLeft.getConfigurator().apply(leftMotorRampConfig);
    elevatorMotorRight.getConfigurator().apply(rightMotorRampConfig);

    // on
    leftSoftLimitConfig.ForwardSoftLimitEnable = true;
    leftSoftLimitConfig.ReverseSoftLimitEnable = true;
    leftSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    rightSoftLimitConfig.ForwardSoftLimitEnable = true;
    rightSoftLimitConfig.ReverseSoftLimitEnable = true;
    rightSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    elevatorLeftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elevatorRightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    elevatorLeftConfiguration.SoftwareLimitSwitch = leftSoftLimitConfig;
    elevatorRightConfiguration.SoftwareLimitSwitch = rightSoftLimitConfig;

    elevatorMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    elevatorMotorRight.getConfigurator().apply(rightSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    pos_reqest = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    elevatorMotorLeft.setPosition(0);
    elevatorMotorRight.setPosition(0);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // absPos = absoluteEncoder.getPosition();
    // SmartDashboard.putNumber("Absolute Encoder Position", getAbsoluteEncoder());
    if(Thresholds.TEST_MODE) {
      SmartDashboard.putNumber("Elevator Left Position", elevatorMotorLeft.getPosition().getValue().magnitude());
      SmartDashboard.putNumber("Elevator Right Position", elevatorMotorRight.getPosition().getValue().magnitude());
      SmartDashboard.putBoolean("Elevator SoftLimit", getSoftLimitBoolean());
      // SmartDashboard.putBoolean("limit", limit);
    }

    // TODO: wtf does this do, make it do something useful or remove it
    // if (absPos == ElevatorParameters.ELEVATOR_NEUTRAL_POS) {
    //   ElevatorParameters.IS_NEUTRAL = true;
    // }
  }

  /**
   * Stops the elevator motors
   *
   * @return void
   */
  public void stopMotors() {
    elevatorMotorLeft.stopMotor();
    elevatorMotorRight.stopMotor();
    voltageOut.Output = -0.014;
    elevatorMotorLeft.setControl(voltageOut);
    elevatorMotorRight.setControl(voltageOut);
  }

  /**
   * Set the position of the left and right elevator motors
   *
   * @param left Left motor position
   * @param right Right motor position
   * @return void
   */
  public void setMotorPosition(double left, double right) {
    elevatorMotorLeft.setControl(pos_reqest.withPosition(left));
    elevatorMotorRight.setControl(pos_reqest.withPosition(right));
  }

  /**
   * Get the position of the elevator motor
   *  
   * 
   * @param string The motor to get the position of
   * 
   * @return double, the position of the elevator motor
   */
  
  // TODO: Figure out what the .magnitude() method does and document it in this file
  public double getElevatorPosValue(String motor) {
    if (motor.equals("left")) {
      return elevatorMotorLeft.getPosition().getValue().magnitude();
    } else if (motor.equals("right")) {
      return elevatorMotorRight.getPosition().getValue().magnitude();
    } else {
      // This only happens if the string is not "left" or "right"
      return 0.0;
    }
  }

  public double getElevatorPosAvg() {
    return (getElevatorPosValue("left") + getElevatorPosValue("right")) / 2;
  }

  public void resetEncoders() {
    elevatorMotorLeft.setPosition(0);
    elevatorMotorRight.setPosition(0);
  }

  public void toggleSoftStop() {
    ElevatorParameters.soft_limit_enabled = !ElevatorParameters.soft_limit_enabled;
    leftSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.soft_limit_enabled;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    // rightSoftLimitConfig.ForwardSoftLimitEnable = elevatorGlobalValues.soft_limit_enabled;
    rightSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.soft_limit_enabled;
    // rightSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    elevatorMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    elevatorMotorRight.getConfigurator().apply(rightSoftLimitConfig);
  }

  /**
   * // * Get the absolute encoder position // * // * @return double, the absolute encoder position
   * of the elevator motor //
   */
  // public double getAbsoluteEncoder() {
  //   // return actualAbsEnc.getAbsolutePosition() * 2048;
  //   if (absoluteEncoder.getPosition() > 190) {
  //     return 0;
  //   } else {
  //     return absoluteEncoder.getPosition();
  //   }
  // }

  public void moveElevator(double velocity) {
    if (Math.abs(velocity) >= deadband) {
      elevatorMotorLeft.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));
      elevatorMotorRight.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));

      SmartDashboard.putNumber("ElevatorLeft Velo Error", elevatorMotorLeft.get() - velocity);
      SmartDashboard.putNumber("ElevatorRight Velo Error", elevatorMotorRight.get() - velocity);
    } else {
      stopMotors();
    }
  }

  public void setElevator(double pos) {
    elevatorMotorLeft.setControl(vel_voltage.withVelocity(pos));
    elevatorMotorRight.setControl(vel_voltage.withVelocity(pos));
  }

  public void toggleLimit() {
    ElevatorParameters.is_SOFTLIMIT = !ElevatorParameters.is_SOFTLIMIT;
  }

  // public void recalibrateEncoders() {
  //   ElevatorGlobalValues.offset = ElevatorGlobalValues.Elevator_NEUTRAL_ANGLE - getAbsoluteEncoder();
  // }

  public boolean getSoftLimitBoolean() {
    return ElevatorParameters.is_SOFTLIMIT;
  }
}
