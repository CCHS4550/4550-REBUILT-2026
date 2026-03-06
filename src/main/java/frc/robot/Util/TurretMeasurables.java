package frc.robot.Util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;

public class TurretMeasurables {

  public Rotation2d elevationAngle;
  public Rotation2d rotationAngle;
  public double shooterRadiansPerSec;

  public TurretMeasurables(
      Rotation2d elevationAngle, Rotation2d rotationAngle, double shooterRadiansPerSec) {
    this.elevationAngle = elevationAngle;
    this.rotationAngle = rotationAngle;
    this.shooterRadiansPerSec = shooterRadiansPerSec;
  }

  public TurretMeasurables() {
    this(new Rotation2d(), new Rotation2d(), 0.0);
  }

  /**
   * Converts spherical turret coordinates into a 3D Cartesian velocity vector. X/Y represent the
   * field grid, Z represents height.
   */
  public Vector<N3> getVector() {
    double horizontalMag = shooterRadiansPerSec * Math.cos(elevationAngle.getRadians());
    return VecBuilder.fill(
        horizontalMag * Math.cos(rotationAngle.getRadians()), // X
        horizontalMag * Math.sin(rotationAngle.getRadians()), // Y
        shooterRadiansPerSec * Math.sin(elevationAngle.getRadians()) // Z (up)
        );
  }

  /**
   * Re-calculates the turret state from a modified Cartesian velocity vector.
   *
   * <p>The zero-vector guard handles the edge case where robot velocity exactly cancels the
   * horizontal shot components (x == 0 AND y == 0). WPILib's Rotation2d(x, y) constructor throws an
   * exception in that case, so we fall back to the pre-compensation rotation angle and set speed to
   * 0 rather than crashing. This should only occur in simulation or on a stationary robot with a
   * perfectly centred target.
   */
  public void updateWithCartesianVector(Vector<N3> updateVector) {
    double x = updateVector.get(0);
    double y = updateVector.get(1);
    double z = updateVector.get(2);

    this.shooterRadiansPerSec = Math.sqrt(x * x + y * y + z * z);

    // Guard: Rotation2d(x, y) throws if both components are zero.
    // Fall back to the existing rotationAngle so the turret holds its
    // last valid heading rather than crashing.
    if (Math.abs(x) < 1e-6 && Math.abs(y) < 1e-6) {
      // rotationAngle is intentionally left unchanged
      this.elevationAngle = new Rotation2d(); // 0° — safest default
      this.shooterRadiansPerSec = 0.0;
      return;
    }

    this.rotationAngle = new Rotation2d(x, y);

    if (this.shooterRadiansPerSec > 1e-6) {
      this.elevationAngle =
          new Rotation2d(Math.asin(Math.max(-1.0, Math.min(1.0, z / this.shooterRadiansPerSec))));
    } else {
      this.elevationAngle = new Rotation2d();
    }
  }
}
