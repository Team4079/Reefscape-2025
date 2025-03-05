package frc.robot.subsystems;

import static com.ctre.phoenix6.signals.InvertedValue.*;
import static frc.robot.utils.ExtensionsKt.*;
import static frc.robot.utils.RobotParameters.ElevatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;
import static frc.robot.utils.emu.ElevatorMotor.*;
import static frc.robot.utils.pingu.LogPingu.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.*;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.emu.*;
import frc.robot.utils.pingu.*;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/**
 * The ElevatorSubsystem class is a Singleton to control the elevator motors on the robot. The class
 * sets the motor positions, gets the motor positions, stops the motors, and toggles the soft stop
 * for the elevator motor.
 */
public class Elevator extends SubsystemBase {

  private final TalonFX elevatorMotorLeft;
  private final TalonFX elevatorMotorRight;

  private final PositionDutyCycle posRequest;
  private final VelocityTorqueCurrentFOC velocityRequest;

  private final SoftwareLimitSwitchConfigs leftSoftLimitConfig;
  private final SoftwareLimitSwitchConfigs rightSoftLimitConfig;

  private LoggedNetworkNumber elevatorP;
  private LoggedNetworkNumber elevatorI;
  private LoggedNetworkNumber elevatorD;
  private LoggedNetworkNumber elevatorV;
  private LoggedNetworkNumber elevatorS;
  private LoggedNetworkNumber elevatorG;
  private LoggedNetworkNumber cruiseV;
  private LoggedNetworkNumber acc;
  private LoggedNetworkNumber jerk;

  private final VoltageOut voltageOut;
  private TalonFXConfiguration elevatorLeftConfigs;
  private TalonFXConfiguration elevatorRightConfigs;

  private MotionMagicConfigs motionMagicConfigs;

  private ElevatorState currentState = ElevatorState.DEFAULT;

  private final MotionMagicVoltage motionMagicVoltage;

  private final DutyCycleOut cycleOut;

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
    elevatorMotorLeft = new TalonFX(ELEVATOR_MOTOR_LEFT_ID);
    elevatorMotorRight = new TalonFX(ELEVATOR_MOTOR_RIGHT_ID);

    TalonFXConfigurator elevatorLeftConfigurator = elevatorMotorLeft.getConfigurator();
    TalonFXConfigurator elevatorRightConfigurator = elevatorMotorRight.getConfigurator();

    elevatorLeftConfigs = new TalonFXConfiguration();
    elevatorRightConfigs = new TalonFXConfiguration();

    elevatorLeftConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevatorRightConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    elevatorLeftConfigs.Slot0.kP = ELEVATOR_PINGU.getP();
    elevatorLeftConfigs.Slot0.kI = ELEVATOR_PINGU.getI();
    elevatorLeftConfigs.Slot0.kD = ELEVATOR_PINGU.getD();
    elevatorLeftConfigs.Slot0.kV = ELEVATOR_PINGU.getV();
    elevatorLeftConfigs.Slot0.kS = ELEVATOR_PINGU.getS();
    elevatorLeftConfigs.Slot0.kG = ELEVATOR_PINGU.getG();

    elevatorRightConfigs.Slot0.kP = ELEVATOR_PINGU.getP();
    elevatorRightConfigs.Slot0.kI = ELEVATOR_PINGU.getI();
    elevatorRightConfigs.Slot0.kD = ELEVATOR_PINGU.getD();
    elevatorRightConfigs.Slot0.kV = ELEVATOR_PINGU.getV();
    elevatorRightConfigs.Slot0.kS = ELEVATOR_PINGU.getS();
    elevatorRightConfigs.Slot0.kG = ELEVATOR_PINGU.getG();

    elevatorMotorLeft.getConfigurator().apply(elevatorLeftConfigs);
    elevatorMotorRight.getConfigurator().apply(elevatorRightConfigs);

    CurrentLimitsConfigs leftMotorCurrentConfig = new CurrentLimitsConfigs();
    CurrentLimitsConfigs rightMotorCurrentConfig = new CurrentLimitsConfigs();

    ClosedLoopRampsConfigs leftMotorRampConfig = new ClosedLoopRampsConfigs();
    ClosedLoopRampsConfigs rightMotorRampConfig = new ClosedLoopRampsConfigs();

    leftSoftLimitConfig = new SoftwareLimitSwitchConfigs();
    rightSoftLimitConfig = new SoftwareLimitSwitchConfigs();

    leftMotorCurrentConfig.SupplyCurrentLimit = 40.79;
    leftMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    leftMotorCurrentConfig.StatorCurrentLimit = 40.79;
    leftMotorCurrentConfig.StatorCurrentLimitEnable = true;

    rightMotorCurrentConfig.SupplyCurrentLimit = 40.79;
    rightMotorCurrentConfig.SupplyCurrentLimitEnable = true;
    rightMotorCurrentConfig.StatorCurrentLimit = 40.79;
    rightMotorCurrentConfig.StatorCurrentLimitEnable = true;

    elevatorMotorLeft.getConfigurator().apply(leftMotorCurrentConfig);
    elevatorMotorRight.getConfigurator().apply(rightMotorCurrentConfig);

    leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0;
    rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.0;

    elevatorMotorLeft.getConfigurator().apply(leftMotorRampConfig);
    elevatorMotorRight.getConfigurator().apply(rightMotorRampConfig);

    // on
    leftSoftLimitConfig.ForwardSoftLimitEnable = true;
    leftSoftLimitConfig.ReverseSoftLimitEnable = true;
    leftSoftLimitConfig.ForwardSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_UP;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_DOWN;

    rightSoftLimitConfig.ForwardSoftLimitEnable = true;
    rightSoftLimitConfig.ReverseSoftLimitEnable = true;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_DOWN;
    rightSoftLimitConfig.ForwardSoftLimitThreshold = ELEVATOR_SOFT_LIMIT_UP;

    elevatorLeftConfigs.MotorOutput.Inverted = CounterClockwise_Positive;
    elevatorRightConfigs.MotorOutput.Inverted = Clockwise_Positive;
    elevatorLeftConfigs.SoftwareLimitSwitch = leftSoftLimitConfig;
    elevatorRightConfigs.SoftwareLimitSwitch = rightSoftLimitConfig;

    elevatorLeftConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1;
    elevatorRightConfigs.MotorOutput.DutyCycleNeutralDeadband = 0.1;

    elevatorMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    elevatorMotorRight.getConfigurator().apply(rightSoftLimitConfig);

    velocityRequest = new VelocityTorqueCurrentFOC(0);
    posRequest = new PositionDutyCycle(0);
    voltageOut = new VoltageOut(0);
    motionMagicVoltage = new MotionMagicVoltage(0);
    cycleOut = new DutyCycleOut(0);

    motionMagicConfigs = elevatorLeftConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = ELEVATOR_MAGIC_PINGU.getVelocity();
    motionMagicConfigs.MotionMagicAcceleration = ELEVATOR_MAGIC_PINGU.getAcceleration();
    motionMagicConfigs.MotionMagicJerk = ELEVATOR_MAGIC_PINGU.getJerk();

    motionMagicVoltage.Slot = 0;
    //    motionMagicVoltage.EnableFOC = true;
    //    motionMagicVoltage.FeedForward = 0;

    // TODO test elevator with FOC, increase accleration, cruise velocity, and graph it all to see
    // how to speed it up
    // reduce timeouts for automatic scoring, autoalign speed it up even more, test in autonomous
    // with 180 command
    // algae and intake prob add or remove idkk yet

    velocityRequest.OverrideCoastDurNeutral = false;

    voltageOut.OverrideBrakeDurNeutral = false;
    voltageOut.EnableFOC = true;

    cycleOut.EnableFOC = false;

    elevatorMotorLeft.setPosition(0);
    elevatorMotorRight.setPosition(0);

    elevatorLeftConfigurator.apply(elevatorLeftConfigs);
    elevatorRightConfigurator.apply(elevatorRightConfigs);

    elevatorLeftConfigurator.apply(motionMagicConfigs);
    elevatorRightConfigurator.apply(motionMagicConfigs);

    AlertPingu.add(elevatorMotorLeft, "left elevator");
    AlertPingu.add(elevatorMotorRight, "right elevator");

    initizalizeLoggedNetworkPID();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    // THIS IS JUST FOR TESTING, in reality, elevator set state is based on
    // what Jayden clicks which will be displayed on leds but not necessarily = currenState
    //    elevatorSetState = currentState;
    setElevatorPosition(currentState);

    logs(
        () -> {
          log(
              "Elevator/Elevator Left Position",
              elevatorMotorLeft.getPosition().getValueAsDouble());
          log(
              "Elevator/Elevator Right Position",
              elevatorMotorRight.getPosition().getValueAsDouble());
          log(
              "Elevator/Elevator Left Set Speed",
              elevatorMotorLeft.getVelocity().getValueAsDouble());
          log(
              "Elevator/Elevator Right Set Speed",
              elevatorMotorRight.getVelocity().getValueAsDouble());
          log(
              "Elevator/Elevator Left Acceleration",
              elevatorMotorLeft.getAcceleration().getValueAsDouble());
          log(
              "Elevator/Elevator Right Acceleration",
              elevatorMotorRight.getAcceleration().getValueAsDouble());
          log(
              "Elevator/Elevator Supply Voltage",
              elevatorMotorLeft.getSupplyVoltage().getValueAsDouble());
          log(
              "Elevator/Elevator Motor Voltage",
              elevatorMotorLeft.getMotorVoltage().getValueAsDouble());
          log("Elevator/Elevator State", currentState.toString());
          log("Elevator/Elevator To Be State", elevatorToBeSetState.toString());
          log(
              "Elevator/Elevator Stator Current",
              elevatorMotorLeft.getStatorCurrent().getValueAsDouble());
          log(
              "Elevator/Elevator Supply Current",
              elevatorMotorLeft.getSupplyCurrent().getValueAsDouble());
          log(
              "Elevator/Elevator Stall Current",
              elevatorMotorLeft.getMotorStallCurrent().getValueAsDouble());
        });
  }

  /** Stops the elevator motors */
  public void stopMotors() {
    elevatorMotorLeft.stopMotor();
    elevatorMotorRight.stopMotor();
    voltageOut.Output = -0.014;
    elevatorMotorLeft.setControl(voltageOut);
    elevatorMotorRight.setControl(voltageOut);
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
    return this.currentState.pos;
  }

  /**
   * Get the position of the elevator motor
   *
   * @param motor "left" or "right | The motor to get the position of
   * @return double, the position of the elevator motor and -1.0 if the motor is not "left" or
   *     "right"
   */
  public double getElevatorPosValue(ElevatorMotor motor) {
    return switch (motor) {
      case LEFT -> elevatorMotorLeft.getPosition().getValueAsDouble();
      case RIGHT -> elevatorMotorRight.getPosition().getValueAsDouble();
    };
  }

  /**
   * Get the average position of the elevator motors
   *
   * @return double, the average position of the elevator motors
   */
  public double getElevatorPosAvg() {
    return (this.getElevatorPosValue(LEFT) + this.getElevatorPosValue(RIGHT)) / 2;
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
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    rightSoftLimitConfig.ReverseSoftLimitEnable = ElevatorParameters.isSoftLimitEnabled;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    elevatorMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    elevatorMotorRight.getConfigurator().apply(rightSoftLimitConfig);
  }

  /**
   * Move the elevator motor at a specific velocity
   *
   * @param speed double, the velocity to move the elevator motor at
   */
  public void moveElevator(double speed) {
    final double deadband = 0.001;
    double velocity = -speed * 0.3309;
    if (Math.abs(velocity) >= deadband) {
      elevatorMotorLeft.setControl(cycleOut.withOutput(velocity));
      elevatorMotorRight.setControl(cycleOut.withOutput(velocity));
    } else {
      stopMotors();
    }
  }

  /**
   * Sets the elevator motor to a specific position
   *
   * @param state the {@link ElevatorState} to set the elevator motor to
   */
  public void setElevatorPosition(ElevatorState state) {
    elevatorMotorLeft.setControl(motionMagicVoltage.withPosition(state.pos));
    elevatorMotorRight.setControl(motionMagicVoltage.withPosition(state.pos));
  }

  public void initizalizeLoggedNetworkPID() {
    elevatorP =
        new LoggedNetworkNumber("Tuning/Elevator/Elevator P", elevatorRightConfigs.Slot0.kP);
    elevatorI =
        new LoggedNetworkNumber("Tuning/Elevator/Elevator I", elevatorRightConfigs.Slot0.kI);
    elevatorD =
        new LoggedNetworkNumber("Tuning/Elevator/Elevator D", elevatorRightConfigs.Slot0.kD);
    elevatorV =
        new LoggedNetworkNumber("Tuning/Elevator/Elevator V", elevatorRightConfigs.Slot0.kV);
    elevatorS =
        new LoggedNetworkNumber("Tuning/Elevator/Elevator S", elevatorRightConfigs.Slot0.kS);
    elevatorG =
        new LoggedNetworkNumber("Tuning/Elevator/Elevator G", elevatorRightConfigs.Slot0.kG);

    cruiseV =
        new LoggedNetworkNumber(
            "Tuning/Elevator/MM Cruise Velocity", motionMagicConfigs.MotionMagicCruiseVelocity);
    acc =
        new LoggedNetworkNumber(
            "Tuning/Elevator/MM Acceleration", motionMagicConfigs.MotionMagicAcceleration);
    jerk = new LoggedNetworkNumber("Tuning/Elevator/MM Jerk", motionMagicConfigs.MotionMagicJerk);
  }

  /**
   * Updates the PID values for the elevator motors. This method sets the PID values for the
   * elevator motors and updates the Motion Magic configurations.
   */
  public void updateElevatorPID() {
    ELEVATOR_PINGU.setP(elevatorP);
    ELEVATOR_PINGU.setI(elevatorI);
    ELEVATOR_PINGU.setD(elevatorD);
    ELEVATOR_PINGU.setV(elevatorV);
    ELEVATOR_PINGU.setS(elevatorS);
    ELEVATOR_PINGU.setG(elevatorG);

    ELEVATOR_MAGIC_PINGU.setVelocity(cruiseV);
    ELEVATOR_MAGIC_PINGU.setAcceleration(acc);
    ELEVATOR_MAGIC_PINGU.setJerk(jerk);

    applyElevatorPIDValues();
  }

  /**
   * Applies the PID values to the elevator motors. This method sets the PID values for both the
   * left and right elevator motor configurations and applies the Motion Magic configurations.
   */
  public void applyElevatorPIDValues() {
    // Set the PID values for the left elevator motor configuration
    setPingu(elevatorLeftConfigs, ELEVATOR_PINGU);

    // Set the PID values for the right elevator motor configuration
    setPingu(elevatorRightConfigs, ELEVATOR_PINGU);

    // Update the Motion Magic configurations with the current PID values
    motionMagicConfigs = elevatorLeftConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = cruiseV.get();
    motionMagicConfigs.MotionMagicAcceleration = acc.get();
    motionMagicConfigs.MotionMagicJerk = jerk.get();

    // Apply the updated configurations to the left elevator motor
    elevatorMotorLeft.getConfigurator().apply(elevatorLeftConfigs);
    elevatorMotorRight.getConfigurator().apply(elevatorRightConfigs);

    // Apply the Motion Magic configurations to the left and right elevator motors
    elevatorMotorLeft.getConfigurator().apply(motionMagicConfigs);
    elevatorMotorRight.getConfigurator().apply(motionMagicConfigs);
  }

  /**
   * Calibrates the elevator motor. This method calibrates the elevator motor by moving the motor up
   * until it stalls.
   */
  public void calibrateElevator() {
    while (elevatorMotorLeft.getMotorStallCurrent().getValueAsDouble() < 3.0) {
      elevatorMotorLeft.setControl(voltageOut.withOutput(0.5));
      elevatorMotorRight.setControl(voltageOut.withOutput(0.5));
    }
  }
}
