package frc.robot.subsystems;

import static frc.robot.utils.Dash.*;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.*;
import frc.robot.utils.RobotParameters.*;
import frc.robot.utils.RobotParameters.SwerveParameters.*;

public class PivotSubsystem extends SubsystemBase {
  /** Creates a new Pivot. */
  private TalonFX pivotMotorLeft;

  private TalonFX pivotMotorRight;

  private TalonFXConfiguration pivotLeftConfiguration;
  private TalonFXConfiguration pivotRightConfiguration;

  private Slot0Configs pivotLeftConfigs;
  private Slot0Configs pivotRightConfigs;

  private PositionVoltage pos_reqest;
  private VelocityVoltage vel_voltage;

  private MotorOutputConfigs pivotConfigs;

  private CurrentLimitsConfigs leftMotorCurrentConfig;
  private CurrentLimitsConfigs rightMotorCurrentConfig;

  private ClosedLoopRampsConfigs leftMotorRampConfig;
  private ClosedLoopRampsConfigs rightMotorRampConfig;

  private SoftwareLimitSwitchConfigs leftSoftLimitConfig;
  private SoftwareLimitSwitchConfigs rightSoftLimitConfig;

  private VoltageOut voltageOut;

  private double deadband = 0.001;

  private double absPos;

  /**
   * The Singleton instance of this PivotSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final PivotSubsystem INSTANCE = new PivotSubsystem();

  /**
   * Returns the Singleton instance of this PivotSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * PivotSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static PivotSubsystem getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this PivotSubsystem. This constructor is private since this class is
   * a Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private PivotSubsystem() {
    pivotMotorLeft = new TalonFX(MotorParameters.PIVOT_MOTOR_LEFT_ID);
    pivotMotorRight = new TalonFX(MotorParameters.PIVOT_MOTOR_RIGHT_ID);

    pivotConfigs = new MotorOutputConfigs();

    pivotLeftConfigs = new Slot0Configs();
    pivotRightConfigs = new Slot0Configs();

    pivotLeftConfiguration = new TalonFXConfiguration();
    pivotRightConfiguration = new TalonFXConfiguration();

    pivotMotorLeft.getConfigurator().apply(pivotLeftConfiguration);
    pivotMotorLeft.getConfigurator().apply(pivotConfigs);
    pivotMotorRight.getConfigurator().apply(pivotRightConfiguration);
    pivotMotorRight.getConfigurator().apply(pivotConfigs);

    pivotConfigs.NeutralMode = NeutralModeValue.Brake;

    pivotLeftConfigs.kP = PivotConstants.PIVOT_PID_LEFT_P;
    pivotLeftConfigs.kI = PivotConstants.PIVOT_PID_LEFT_I;
    pivotLeftConfigs.kD = PivotConstants.PIVOT_PID_LEFT_D;
    pivotLeftConfigs.kV = PivotConstants.PIVOT_PID_LEFT_V;
    // pivotLeftConfigs.kF = PivotConstants.PIVOT_PID_LEFT_F;

    pivotRightConfigs.kP = PivotConstants.PIVOT_PID_RIGHT_P;
    pivotRightConfigs.kI = PivotConstants.PIVOT_PID_RIGHT_I;
    pivotRightConfigs.kD = PivotConstants.PIVOT_PID_RIGHT_D;
    pivotRightConfigs.kV = PivotConstants.PIVOT_PID_RIGHT_V;
    // pivotRightConfigs.kF = PivotConstants.PIVOT_PID_RIGHT_F;

    pivotMotorLeft.getConfigurator().apply(pivotLeftConfigs);
    pivotMotorRight.getConfigurator().apply(pivotRightConfigs);

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

    pivotMotorLeft.getConfigurator().apply(leftMotorCurrentConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorCurrentConfig);

    leftMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;
    rightMotorRampConfig.DutyCycleClosedLoopRampPeriod = 0.1;

    pivotMotorLeft.getConfigurator().apply(leftMotorRampConfig);
    pivotMotorRight.getConfigurator().apply(rightMotorRampConfig);

    leftSoftLimitConfig.ForwardSoftLimitEnable = true;
    leftSoftLimitConfig.ReverseSoftLimitEnable = true;
    leftSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    rightSoftLimitConfig.ForwardSoftLimitEnable = true;
    rightSoftLimitConfig.ReverseSoftLimitEnable = true;
    rightSoftLimitConfig.ForwardSoftLimitThreshold = 40;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0.2;

    pivotLeftConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    pivotRightConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    pivotLeftConfiguration.SoftwareLimitSwitch = leftSoftLimitConfig;
    pivotRightConfiguration.SoftwareLimitSwitch = rightSoftLimitConfig;

    pivotMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    pivotMotorRight.getConfigurator().apply(rightSoftLimitConfig);

    // absoluteEncoder = new DigitalInput(9);

    vel_voltage = new VelocityVoltage(0);
    pos_reqest = new PositionVoltage(0);
    voltageOut = new VoltageOut(0);
    new PositionDutyCycle(0);

    pivotMotorLeft.setPosition(0);
    pivotMotorRight.setPosition(0);

    SmartDashboard.putNumber("Pivot Angle Value", 46);
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    dash(
        pairOf("Pivot Left Position", pivotMotorLeft.getPosition().getValueAsDouble()),
        pairOf("Pivot Right Position", pivotMotorRight.getPosition().getValueAsDouble()),
        pairOf("Pivot SoftLimit", getSoftLimit()));

    // TODO: wtf does this do, make it do something useful or remove it
    // if (absPos == PivotConstants.PIVOT_NEUTRAL_POS) {
    //   PivotConstants.IS_NEUTRAL = true;
    // }
  }

  /** Stops the pivot motors */
  public void stopMotors() {
    pivotMotorLeft.stopMotor();
    pivotMotorRight.stopMotor();
    voltageOut.Output = -0.014;
    pivotMotorLeft.setControl(voltageOut);
    pivotMotorRight.setControl(voltageOut);
  }

  /**
   * Set the position of the left and right pivot motors
   *
   * @param left Left motor position
   * @param right Right motor position
   */
  public void setMotorPosition(double left, double right) {
    pivotMotorLeft.setControl(pos_reqest.withPosition(left));
    pivotMotorRight.setControl(pos_reqest.withPosition(right));
  }

  /**
   * Get the position of the pivot motor
   *
   * @return double, the position of the pivot motor
   */
  public double getPivotLeftPos() {
    return pivotMotorLeft.getPosition().getValueAsDouble();
  }

  public double getPivotRightPos() {
    return pivotMotorRight.getPosition().getValueAsDouble();
  }

  public double getPivotPos() {
    return (getPivotLeftPos() + getPivotRightPos()) / 2;
  }

  /**
   * Run distance through a best fit line and return the value
   *
   * @param distance The distance
   * @return double, the position of the pivot motor
   */
  public double shootPos(double distance) {
    // line function
    // TODO: do stuf
    return 0.0;
  }

  public void resetEncoders() {
    pivotMotorLeft.setPosition(0);
    pivotMotorRight.setPosition(0);
  }

  public void toggleSoftStop() {
    PivotConstants.SOFT_LIMIT_ENABLED = !PivotConstants.SOFT_LIMIT_ENABLED;
    leftSoftLimitConfig.ReverseSoftLimitEnable = PivotConstants.SOFT_LIMIT_ENABLED;
    // leftSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    leftSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    // rightSoftLimitConfig.ForwardSoftLimitEnable = PivotGlobalValues.soft_limit_enabled;
    rightSoftLimitConfig.ReverseSoftLimitEnable = PivotConstants.SOFT_LIMIT_ENABLED;
    // rightSoftLimitConfig.ForwardSoftLimitThreshold = 1100;
    rightSoftLimitConfig.ReverseSoftLimitThreshold = 0;

    pivotMotorLeft.getConfigurator().apply(leftSoftLimitConfig);
    pivotMotorRight.getConfigurator().apply(rightSoftLimitConfig);
  }

  /**
   * // * Get the absolute encoder position // * // * @return double, the absolute encoder position
   * of the pivot motor //
   */
  // public double getAbsoluteEncoder() {
  //   // return actualAbsEnc.getAbsolutePosition() * 2048;
  //   if (absoluteEncoder.getPosition() > 190) {
  //     return 0;
  //   }
  //   else {
  //     return absoluteEncoder.getPosition();
  //   }
  // }

  public void movePivot(double velocity) {
    if (Math.abs(velocity) >= deadband) {
      pivotMotorLeft.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));
      pivotMotorRight.setControl(vel_voltage.withVelocity(velocity * 500 * 0.75));

      SmartDashboard.putNumber("PivotLeft Velo Error", pivotMotorLeft.get() - velocity);
      SmartDashboard.putNumber("PivotRight Velo Error", pivotMotorLeft.get() - velocity);
    } else {
      stopMotors();
    }
  }

  public void setPivot(double pos) {
    pivotMotorLeft.setControl(vel_voltage.withVelocity(pos));
    pivotMotorRight.setControl(vel_voltage.withVelocity(pos));
  }

  public void toggleLimit() {
    PivotConstants.IS_SOFTLIMIT = !PivotConstants.IS_SOFTLIMIT;
  }

  // public void recalibrateEncoders() {
  //   PivotGlobalValues.offset = PivotGlobalValues.PIVOT_NEUTRAL_ANGLE - getAbsoluteEncoder();
  // }

  public double getPivotPositionAvg() {
    return (getPivotLeftPos() + getPivotRightPos()) / 2;
  }

  public boolean getSoftLimit() {
    return PivotConstants.IS_SOFTLIMIT;
  }
}
