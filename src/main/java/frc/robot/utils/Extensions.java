package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

/** Utility class for extension functions. */
public class Extensions {
  /**
   * Converts a 2D translation to a 3D translation.
   *
   * @param translation2d The 2D translation to convert.
   * @param z The z-coordinate for the 3D translation.
   * @return The resulting 3D translation.
   */
  public static Translation3d dimensionIncrease(Translation2d translation2d, double z) {
    return new Translation3d(translation2d.getX(), translation2d.getY(), z);
  }
}
