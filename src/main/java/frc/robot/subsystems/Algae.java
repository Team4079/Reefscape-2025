package frc.robot.subsystems;

// import static frc.robot.utils.Dash.log;
import static edu.wpi.first.wpilibj.Alert.AlertType.*;
import static frc.robot.utils.Register.Dash.*;
import static frc.robot.utils.RobotParameters.AlgaeManipulatorParameters.*;
import static frc.robot.utils.RobotParameters.MotorParameters.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.AlgaePivotState;
import frc.robot.utils.RobotParameters.*;

/**
 * The PivotSubsystem class is a subsystem that interfaces with the arm system to provide control
 * over the arm motors. This subsystem is a Singleton, meaning that only one instance of this class
 * is created and shared across the entire robot code.
 */
public class Algae extends SubsystemBase {
  /** Creates a new end effector. */
  private final TalonFX algaePivotMotor;

  private final TalonFX algaeIntakeMotor;

  private final VoltageOut voltageOut;

  private Alert algaeManipulatorMotorDisconnectedAlert;

  // private double absPos = 0;

  /**
   * The Singleton instance of this PivotSubsystem. Code should use the {@link #getInstance()}
   * method to get the single instance (rather than trying to construct an instance of this class.)
   */
  private static final Algae INSTANCE = new Algae();

  /**
   * Returns the Singleton instance of this PivotSubsystem. This static method should be used,
   * rather than the constructor, to get the single instance of this class. For example: {@code
   * armSubsystem.getInstance();}
   */
  @SuppressWarnings("WeakerAccess")
  public static Algae getInstance() {
    return INSTANCE;
  }

  /**
   * Creates a new instance of this armSubsystem. This constructor is private since this class is a
   * Singleton. Code should use the {@link #getInstance()} method to get the singleton instance.
   */
  private Algae() {
    algaePivotMotor = new TalonFX(ALGAE_PIVOT_MOTOR_ID);
    algaeIntakeMotor = new TalonFX(ALGAE_INTAKE_MOTOR_ID);

    TalonFXConfiguration algaePivotConfiguration = new TalonFXConfiguration();
    TalonFXConfiguration algaeIntakeConfiguration = new TalonFXConfiguration();

    algaePivotConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeIntakeConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    algaePivotConfiguration.Slot0.kP = AlgaeManipulatorParameters.ALGAE_PINGU.getP();
    algaePivotConfiguration.Slot0.kI = AlgaeManipulatorParameters.ALGAE_PINGU.getI();
    algaePivotConfiguration.Slot0.kD = AlgaeManipulatorParameters.ALGAE_PINGU.getD();

    algaePivotConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    algaeIntakeConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    algaePivotMotor.getConfigurator().apply(algaePivotConfiguration);
    algaeIntakeMotor.getConfigurator().apply(algaeIntakeConfiguration);

    algaePivotConfiguration.CurrentLimits.SupplyCurrentLimit = 100;
    algaePivotConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaePivotConfiguration.CurrentLimits.StatorCurrentLimit = 100;
    algaePivotConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimit = 100;
    algaeIntakeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimit = 100;
    algaeIntakeConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;

    algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0;
    algaePivotConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0;
    algaePivotConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    algaePivotMotor.getConfigurator().apply(algaePivotConfiguration);
    algaeIntakeMotor.getConfigurator().apply(algaeIntakeConfiguration);

    //    algaeManipulatorMotorConfiguration.MotorOutput.Inverted =
    // InvertedValue.Clockwise_Positive;
    //
    //    algaeManipulatorMotorConfiguration.SoftwareLimitSwitch =
    // algaeManipulatorMotorSoftLimitConfig;

    voltageOut = new VoltageOut(0);

    algaePivotMotor.setPosition(0);

    initializeAlarms();
  }

  // This method will be called once per scheduler run
  @Override
  public void periodic() {
    setPivotState(pivotState);
    logs(
        () -> {
          log("/Algae/Algae Pivot Motor Position", getPivotPosValue());
          log("/Algae/Algae State", pivotState.toString());
        });
  }

  /** Stops the arm motor */
  public void stopIntake() {
    algaeIntakeMotor.stopMotor();
  }

  public void startIntake() {
    voltageOut.Output = 4.0;
    algaeIntakeMotor.setControl(voltageOut);
  }

  /**
   * Sets the pivot state *
   *
   * @param state the state to set the algae pivot
   */
  public void setPivotState(AlgaePivotState state) {
    algaePivotMotor.setPosition(state.pos);
  }

  /**
   * Get the position of the end effector motor
   *
   * @return double, the position of the end effector motor
   */
  public double getPivotPosValue() {
    return algaePivotMotor.getPosition().getValueAsDouble();
  }

  public void initializeAlarms() {
    algaeManipulatorMotorDisconnectedAlert =
        new Alert("Disconnected end effector motor " + ALGAE_PIVOT_MOTOR_ID, kError);

    algaeManipulatorMotorDisconnectedAlert.set(!algaePivotMotor.isConnected());

    logs(
        "Disconnected algaeManipulatorMotor " + algaePivotMotor.getDeviceID(),
        algaePivotMotor.isConnected());
  }
}
