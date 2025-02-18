package frc.robot.utils;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * A class that implements the Pathfinder interface using the LocalADStar algorithm. This class is
 * responsible for calculating paths for a robot to follow on a field, taking into account dynamic
 * obstacles and other constraints.
 *
 * <p>Usage in FRC: This class is used in FRC (FIRST Robotics Competition) to navigate the robot
 * autonomously on the field. It calculates the optimal path from a start position to a goal
 * position while avoiding obstacles. The path is updated dynamically based on the current state of
 * the field and the robot's position.
 */
public class LocalADStarAK implements Pathfinder {
  private final ADStarIO io = new ADStarIO();

  /**
   * Get if a new path has been calculated since the last time a path was retrieved. This method
   * checks if a new path is available and logs the current state.
   *
   * @return True if a new path is available.
   */
  @Override
  public boolean isNewPathAvailable() {
    if (!Logger.hasReplaySource()) {
      io.updateIsNewPathAvailable();
    }

    Logger.processInputs("LocalADStarAK", io);

    return io.isNewPathAvailable;
  }

  /**
   * Get the most recently calculated path. This method retrieves the current path based on the
   * provided constraints and goal end state.
   *
   * @param constraints The path constraints to use when creating the path.
   * @param goalEndState The goal end state to use when creating the path.
   * @return The PathPlannerPath created from the points calculated by the pathfinder.
   */
  @Override
  public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goalEndState) {
    if (!Logger.hasReplaySource()) {
      io.updateCurrentPathPoints(constraints, goalEndState);
    }

    Logger.processInputs("LocalADStarAK", io);

    if (io.currentPathPoints.isEmpty()) {
      return null;
    }

    return PathPlannerPath.fromPathPoints(io.currentPathPoints, constraints, goalEndState);
  }

  /**
   * Set the start position to pathfind from. This method sets the initial position for the
   * pathfinding algorithm.
   *
   * @param startPosition Start position on the field. If this is within an obstacle it will be
   *     moved to the nearest non-obstacle node.
   */
  @Override
  public void setStartPosition(Translation2d startPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setStartPosition(startPosition);
    }
  }

  /**
   * Set the goal position to pathfind to. This method sets the target position for the pathfinding
   * algorithm.
   *
   * @param goalPosition Goal position on the field. If this is within an obstacle it will be moved
   *     to the nearest non-obstacle node.
   */
  @Override
  public void setGoalPosition(Translation2d goalPosition) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setGoalPosition(goalPosition);
    }
  }

  /**
   * Set the dynamic obstacles that should be avoided while pathfinding. This method updates the
   * list of obstacles that the pathfinding algorithm should avoid.
   *
   * @param obs A List of Translation2d pairs representing obstacles. Each Translation2d represents
   *     opposite corners of a bounding box.
   * @param currentRobotPos The current position of the robot. This is needed to change the start
   *     position of the path to properly avoid obstacles.
   */
  @Override
  public void setDynamicObstacles(
      List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
    if (!Logger.hasReplaySource()) {
      io.adStar.setDynamicObstacles(obs, currentRobotPos);
    }
  }

  /**
   * A class that handles the input/output operations for the LocalADStar pathfinding algorithm.
   * Implements the LoggableInputs interface to allow logging of pathfinding data. This class is
   * responsible for managing the state of the pathfinding algorithm, including whether a new path
   * is available and the current path points.
   */
  private static class ADStarIO implements LoggableInputs {
    public LocalADStar adStar = new LocalADStar();
    public boolean isNewPathAvailable = false;
    public List<PathPoint> currentPathPoints = Collections.emptyList();

    @Override
    public void toLog(LogTable table) {
      table.put("IsNewPathAvailable", isNewPathAvailable);

      double[] pointsLogged = new double[currentPathPoints.size() * 2];
      int idx = 0;
      for (PathPoint point : currentPathPoints) {
        pointsLogged[idx] = point.position.getX();
        pointsLogged[idx + 1] = point.position.getY();
        idx += 2;
      }

      table.put("CurrentPathPoints", pointsLogged);
    }

    @Override
    public void fromLog(LogTable table) {
      isNewPathAvailable = table.get("IsNewPathAvailable", false);

      double[] pointsLogged = table.get("CurrentPathPoints", new double[0]);

      List<PathPoint> pathPoints = new ArrayList<>(pointsLogged.length / 2);
      for (int i = 0; i < pointsLogged.length; i += 2) {
        pathPoints.add(
            new PathPoint(new Translation2d(pointsLogged[i], pointsLogged[i + 1]), null));
      }

      currentPathPoints = pathPoints;
    }

    /**
     * Updates the isNewPathAvailable flag by querying the LocalADStar instance. This method checks
     * if a new path has been calculated by the pathfinding algorithm.
     */
    public void updateIsNewPathAvailable() {
      isNewPathAvailable = adStar.isNewPathAvailable();
    }

    /**
     * Updates the current path points by querying the LocalADStar instance with the provided
     * constraints and goal end state. This method retrieves the latest path points calculated by
     * the pathfinding algorithm.
     *
     * @param constraints The path constraints to use when creating the path.
     * @param goalEndState The goal end state to use when creating the path.
     */
    public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goalEndState) {
      PathPlannerPath currentPath = adStar.getCurrentPath(constraints, goalEndState);

      if (currentPath != null) {
        currentPathPoints = currentPath.getAllPathPoints();
      } else {
        currentPathPoints = Collections.emptyList();
      }
    }
  }
}
