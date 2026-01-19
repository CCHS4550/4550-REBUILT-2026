package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;

public class TurretState {
  // instance variables
  private Rotation2d elevationAngle;
  private Rotation2d rotationAngle;
  private AngularVelocity shooterAngularVelocity;
  private double timestamp;

  // constructor
  public TurretState(
      Rotation2d elevationAngle,
      AngularVelocity elevationVelocityRadPerSec,
      Rotation2d rotationAngle,
      AngularVelocity rotationVelocityRadPerSec,
      AngularVelocity shooterAngularVelocity,
      double timestamp) {
    this.elevationAngle = elevationAngle;
    this.rotationAngle = rotationAngle;
    this.shooterAngularVelocity = shooterAngularVelocity;
    this.timestamp = timestamp;
  }

  // methods - return so and so
  public Rotation2d getElevationAngle() {
    return elevationAngle;
  }

  public Rotation2d getRotationAngle() {
    return rotationAngle;
  }

  public AngularVelocity getShooterAngularVelocity() {
    return shooterAngularVelocity;
  }

  public double getTimestamp() {
    return timestamp;
  }
}
