package frc.robot.subsystems;

import static frc.robot.utils.Dash.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.*;

/**
 * The ElevatorSubsystem class is a Singleton to control the elevator motors on the robot. The class
 * sets the motor positions, gets the motor positions, stops the motors, and toggles the soft stop
 * for the elevator motor.
 */
public class Elevator extends SubsystemBase {
  public static final String ELEVATOR_STATE_KEY = "Elevator State";

  private final TalonFX elevatorMotorLeft;
  private final TalonFX elevatorMotorRight;

  private final PositionTorqueCurrentFOC pos_request;
  private final VelocityTorqueCurrentFOC vel_voltage;

  private final SoftwareLimitSwitchConfigs leftSoftLimitConfig;
  private final SoftwareLimitSwitchConfigs rightSoftLimitConfig;

  private final VoltageOut voltageOut;

  private ElevatorState currentState = frc.robot.utils.ElevatorState.L1;

  private Alert elevatorLeftDisconnectedAlert;
  private Alert elevatorRightDisconnectedAlert;

  /**
   * The Singleton instance of this ElevatorSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final Elevator INSTANCE = new Elevator();

  /**
   * Returns the Singleton instance of this ElevatorSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * ElevatorSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Elevator getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this ElevatorSubsystem. This constructor is private since this class
   * is a Singleton. Code should use the {@link #getInstance()} method to get the singleton
   * instance.
   */
  private Elevator() {
    elevatorMotorLeft = new TalonFX(MotorParameters.ELEVATOR_MOTOR_LEFT_ID);
    elevatorMotorRight = new TalonFX(MotorParameters.ELEVATOR_MOTOR_RIGHT_ID);

    MotorOutputConfigs elevatorConfigs = new MotorOutputConfigs();

    TalonFXConfigurator elevatorLeftConfigurator = elevatorMotorLeft.getConfigurator();
    TalonFXConfigurator elevatorRightConfigurator = elevatorMotorRight.getConfigurator();

    Slot0Configs elevatorLeftConfigs = new Slot0Configs();
    Slot0Configs elevatorRightConfigs = new Slot0Configs();

    TalonFXConfiguration elevatorLeftConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration elevatorRightConfiguration = new TalonFXConfiguration();

    elevatorMotorLeft.getConfigurator().apply(elevatorLeftConfiguration);
    elevatorMotorRight.getConfigurator().apply(elevatorRightConfiguration);

    elevatorConfigs.NeutralMode = NeutralModeValue.Brake;
    elevatorLeftConfigurator.apply(elevatorConfigs);
    elevatorRightConfigurator.apply(elevatorConfigs);

    elevatorLeftConfigs.kP = RobotParameters.ElevatorParameters.ELEVATOR_PID_LEFT_P;
    elevatorLeftConfigs.kI = RobotParameters.ElevatorParameters.ELEVATOR_PID_LEFT_I;
    elevatorLeftConfigs.kD = RobotParameters.ElevatorParameters.ELEVATOR_PID_LEFT_D;
    elevatorLeftConfigs.kV = RobotParameters.ElevatorParameters.ELEVATOR_PID_LEFT_V;

    elevatorRightConfigs.kP = RobotParameters.ElevatorParameters.ELEVATOR_PID_RIGHT_P;
    elevatorRightConfigs.kI = RobotParameters.ElevatorParameters.ELEVATOR_PID_RIGHT_I;
    elevatorRightConfigs.kD = RobotParameters.ElevatorParameters.ELEVATOR_PID_RIGHT_D;
    elevatorRightConfigs.kV = RobotParameters.ElevatorParameters.ELEVATOR_PID_RIGHT_V;

    elevatorMotorLeft.getConfigurator().apply(elevatorLeftConfigs);
    elevatorMotorRight.getConfigurator().apply(elevatorRightConfigs);

    CurrentLimitsConfigs leftMotorCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs rightMotorCurrentConfig = new CurrentLimitsConfigs();

    ClosedLoopRampsConfigs leftMotorRampConfig = new ClosedLoopRampsConfigs();
    ClosedLoopRampsConfigs rightMotorRampConfig = new ClosedLoopRampsConfigs();

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

    elevatorLeftConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    elevatorRightConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevatorLeftConfiguration.SoftwareLimitSwitch = leftSoftLimitConfig;
    elevatorRightConfiguration.SoftwareLimitSwitch = rightSoftLimitConfig;

    elevatorMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    elevatorMotorRight.getConfigurator().apply(rightSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityTorqueCurrentFOC(0);
    pos_request = new PositionTorqueCurrentFOC(0);
    voltageOut = new VoltageOut(0);

    new PositionDutyCycle(0);

    elevatorMotorLeft.setPosition(0);
    elevatorMotorRight.setPosition(0);

    initializeAlarms();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    log("Elevator Left Position", elevatorMotorLeft.getPosition().getValueAsDouble());
    log("Elevator Right Position", elevatorMotorRight.getPosition().getValueAsDouble());
    logElevatorState();
  }

  /** Move the elevator motor to a specific level */
  public void moveElevatorToLevel() {
    switch (this.currentState) {
      case L2:
        setElevatorPosition(ElevatorParameters.L2, ElevatorParameters.L2);
        break;
      case L3:
        setElevatorPosition(ElevatorParameters.L3, ElevatorParameters.L3);
        break;
      case L4:
        setElevatorPosition(ElevatorParameters.L4, ElevatorParameters.L4);
        break;
      default:
        setElevatorPosition(ElevatorParameters.L1, ElevatorParameters.L1);
        break;
    }
  }

  /** Stops the elevator motors */
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
   */
  public void setElevatorPosition(double left, double right) {
    elevatorMotorLeft.setControl(pos_request.withPosition(left));
    elevatorMotorRight.setControl(pos_request.withPosition(right));
  }

  /** Sets the elevator state */
  public void setState(ElevatorState currentState) {
    this.currentState = currentState;
  }

  /**
   * Get the state of the elevator motor
   *
   * @return ElevatorState, the state of the elevator motor
   */
  public ElevatorState getState() {
    return this.currentState;
  }

  /**
   * Gets the state of the elevator motor as a double in terms of its height in the parameters file
   *
   * @return double, the state of the elevator motor as a double
   */
  public double getStateDouble() {
    return switch (this.currentState) {
      case L2 -> ElevatorParameters.L2;
      case L3 -> ElevatorParameters.L3;
      case L4 -> ElevatorParameters.L4;
      default -> ElevatorParameters.L1;
    };
  }

  /**
   * Get the position of the elevator motor
   *
   * @param motor "left" or "right | The motor to get the position of
   * @return double, the position of the elevator motor and -1.0 if the motor is not "left" or
   *     "right"
   */
  public double getElevatorPosValue(String motor) {
    if (motor.equalsIgnoreCase("left")) {
      return elevatorMotorLeft.getPosition().getValueAsDouble();
    } else if (motor.equalsIgnoreCase("right")) {
      return elevatorMotorRight.getPosition().getValueAsDouble();
    } else {
      // This returns if the motor is not "left" or "right"
      System.out.println(
          "getElevatorPosValue: Invalid motor type, motor type should only be 'left' or 'right'");
      return -1.0;
    }
  }

  /**
   * Sets the state of the elevator motor based on the state local variable value To be used in
   * sequences
   */
  public void logElevatorState() {
    log(ELEVATOR_STATE_KEY, currentState.toString());
  }

  /**
   * Get the average position of the elevator motors
   *
   * @return double, the average position of the elevator motors
   */
  public double getElevatorPosAvg() {
    return (this.getElevatorPosValue("left") + this.getElevatorPosValue("right")) / 2;
  }

  /** Soft resets the encoders on the elevator motors */
  public void resetEncoders() {
    elevatorMotorLeft.setPosition(0);
    elevatorMotorRight.setPosition(0);
  }

  /** Toggles the soft stop for the elevator motor */
  public void toggleSoftStop() {
    ElevatorParameters.isSoftLimitEnabled = !ElevatorParameters.isSoftLimitEnabled;
    leftSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    // rightSoftLimitConfig.ForwardSoftLimitEnable = elevatorGlobalValues.soft_limit_enabled;
    rightSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled;
    // rightSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    elevatorMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    elevatorMotorRight.getConfigurator().apply(rightSoftLimitConfig);
  }

  /**
   * Move the elevator motor at a specific velocity
   *
   * @param velocity double, the velocity to move the elevator motor at
   */
  public void moveElevator(double velocity) {
    final double deadband = 0.001;
    if (Math.abs(velocity) >= deadband) {
      elevatorMotorLeft.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));
      elevatorMotorRight.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));

      log("ElevatorLeft Velo", elevatorMotorLeft.get());
      log("ElevatorRight Velo", elevatorMotorRight.get());

    } else {
      stopMotors();
    }
  }

  /**
   * Sets the elevator motor to a specific position
   *
   * @param pos double, the position to set the elevator motor to
   */
  public void setElevatorPosition(double pos) {
    elevatorMotorLeft.setControl(pos_request.withVelocity(pos));
    elevatorMotorRight.setControl(pos_request.withVelocity(pos));
  }

  public void initializeAlarms() {
    elevatorLeftDisconnectedAlert =
            new Alert("Disconnected drive motor " + Integer.toString(MotorParameters.ELEVATOR_MOTOR_LEFT_ID) + ".", Alert.AlertType.kError);
    elevatorRightDisconnectedAlert = 
            new Alert("Disconnected turn motor " + Integer.toString(MotorParameters.ELEVATOR_MOTOR_RIGHT_ID) + ".", Alert.AlertType.kError);

    elevatorLeftDisconnectedAlert.set(!elevatorMotorLeft.isConnected());
    elevatorRightDisconnectedAlert.set(!elevatorMotorRight.isConnected());

    log("Disconnected elevatorMotorLeft " +  Integer.toString(elevatorMotorLeft.getDeviceID()) + ".", elevatorMotorLeft.isConnected());
    log("Disconnected elevatorMotorRight " + Integer.toString(elevatorMotorRight.getDeviceID()) + ".", elevatorMotorRight.isConnected());
  }
}


