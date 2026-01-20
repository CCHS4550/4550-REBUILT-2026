package frc.robot.Util;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constant.Constants;

public class TurretMeasurables {
  // instance variables
  public Rotation2d elevationAngle;
  public Rotation2d rotationAngle;
  public double shooterRadiansPerSec;

  // constructor
  public TurretMeasurables(
      Rotation2d elevationAngle,
      Rotation2d rotationAngle,
      double shooterRadiansPerSec) {
    this.elevationAngle = elevationAngle;
    this.rotationAngle = rotationAngle;
    this.shooterRadiansPerSec = shooterRadiansPerSec;
  }

  public TurretMeasurables(
    Rotation2d elevationAngle, Rotation2d rotationAngle){
      this.elevationAngle = elevationAngle;
      this.elevationAngle = rotationAngle;
      this.shooterRadiansPerSec = Constants.TurretConstants.SHOOTER_MAX_RADIANS_PER_SEC;
    }
}
