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
    // Find the horizontal shadow (projection) of the vector on the floor
    double horizontalMag = shooterRadiansPerSec * Math.cos(elevationAngle.getRadians());

    return VecBuilder.fill(
        horizontalMag * Math.cos(rotationAngle.getRadians()), // X
        horizontalMag * Math.sin(rotationAngle.getRadians()), // Y
        shooterRadiansPerSec * Math.sin(elevationAngle.getRadians()) // Z (Up)
        );
  }

  /** Re-calculates the turret state from a modified Cartesian velocity vector. */
  public void updateWithCartesianVector(Vector<N3> updateVector) {
    double x = updateVector.get(0);
    double y = updateVector.get(1);
    double z = updateVector.get(2);

    this.shooterRadiansPerSec = Math.sqrt(x * x + y * y + z * z);
    // WPILib's Rotation2d handles atan2(y, x) internally
    this.rotationAngle = new Rotation2d(x, y);

    // Protect against divide-by-zero if speed is completely 0
    if (this.shooterRadiansPerSec > 1e-6) {
      this.elevationAngle = new Rotation2d(Math.asin(z / this.shooterRadiansPerSec));
    } else {
      this.elevationAngle = new Rotation2d();
    }
  }
}
