package frc.robot;

import static edu.wpi.first.wpilibj.RobotController.*;
import static edu.wpi.first.wpilibj.Threads.*;
import static frc.robot.commands.Kommand.flipPidgey;
import static frc.robot.utils.RobotParameters.LiveRobotValues.*;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.Kommand.*;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Swerve;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.RobotParameters;
import frc.robot.utils.RobotParameters.FieldParameters.*;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
@SuppressWarnings("resource")
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  private Timer garbageTimer;
  private Timer batteryTimer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Set a metadata value
    RobotParameters.Info.logInfo();

    // Calls LEDs to activate
    LED.getInstance();

    // Records useful but random info
    Logger.recordMetadata("Reefscape", "Logging");

    // Set the pathfinder
    Pathfinding.setPathfinder(new LocalADStarAK());

    if (isReal()) {
      // Log to NetworkTables
      Logger.addDataReceiver(new NT4Publisher());

      // WARNING: PowerDistribution resource leak
      // Enables power distribution logging
      new PowerDistribution(1, ModuleType.kRev);

    } else {
      // Run as fast as possible
      setUseTiming(false);

      // Pull the replay log from AdvantageScope (or prompt the user)
      String logPath = LogFileUtil.findReplayLog();

      // Read replay log
      Logger.setReplaySource(new WPILOGReader(logPath));

      // Save outputs to a new log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
    }

    // Start the logger
    Logger.start();

    // Call addCoralPosList
    RobotPoses.addCoralPosList();

    // Initialize the garbage timer
    garbageTimer = new Timer();
    batteryTimer = new Timer();
    garbageTimer.start();

    // Configure auto builder
    Swerve.getInstance().configureAutoBuilder();

    // Initialize the robot container
    robotContainer = new RobotContainer();

    // Schedule the warmup command
    PathfindingCommand.warmupCommand().schedule();

    CommandScheduler.getInstance().enable();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    setCurrentThreadPriority(true, 99);

    CommandScheduler.getInstance().run();
    if (garbageTimer.advanceIfElapsed(5)) System.gc();

    // Checks for low battery
    if (getBatteryVoltage() < LOW_BATTERY_VOLTAGE) {
      batteryTimer.start();
      if (batteryTimer.advanceIfElapsed(1.5)) {
        lowBattery = true;
      }
    } else {
      batteryTimer.stop();
      lowBattery = false;
    }

    setCurrentThreadPriority(false, 99);
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. **/
  @Override
  public void autonomousInit() {
    //    autonomousCommand = robotContainer.networkChooser.getSelected();
    flipPidgey();
    autonomousCommand = new PathPlannerAuto("4l4autoA");
    autonomousCommand.schedule();
  }

  /** This function is called once when teleop mode is initialized. */
  @Override
  public void teleopInit() {
    if (autonomousCommand != null) autonomousCommand.cancel();
//    flipPidgey();
  }

  /** This function is called once when test mode is initialized. */
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
