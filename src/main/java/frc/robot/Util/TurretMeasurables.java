package frc.robot.Util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constant.Constants;

public class TurretMeasurables {
  // instance variables
  public Rotation2d elevationAngle;
  public Rotation2d rotationAngle;
  public double shooterRadiansPerSec;

  // constructor
  public TurretMeasurables(
      Rotation2d elevationAngle, Rotation2d rotationAngle, double shooterRadiansPerSec) {
    this.elevationAngle = elevationAngle;
    this.rotationAngle = rotationAngle;
    this.shooterRadiansPerSec = shooterRadiansPerSec;
  }

  public TurretMeasurables(Rotation2d elevationAngle, Rotation2d rotationAngle) {
    this.elevationAngle = elevationAngle;
    this.rotationAngle = rotationAngle;
    this.shooterRadiansPerSec = Constants.TurretConstants.SHOOTER_MAX_RADIANS_PER_SEC;
  }

  public Vector<N3> getVector() {
    return VecBuilder.fill(
        shooterRadiansPerSec
            * Math.cos(rotationAngle.getRadians())
            * Math.sin((Math.PI / 2) - elevationAngle.getRadians()),
        shooterRadiansPerSec
            * Math.sin(rotationAngle.getRadians())
            * Math.cos((Math.PI / 2) - elevationAngle.getRadians()),
        shooterRadiansPerSec * Math.cos((Math.PI / 2) - elevationAngle.getRadians()));
  }

  public void updateWithCartesianVector(Vector<N3> updateVector) {
    this.shooterRadiansPerSec =
        Math.sqrt(
            Math.pow(updateVector.get(0), 2)
                + Math.pow(updateVector.get(1), 2)
                + Math.pow(updateVector.get(2), 2));
    this.rotationAngle = new Rotation2d(Math.atan2(updateVector.get(1), updateVector.get(0)));
    this.elevationAngle = new Rotation2d(Math.acos(updateVector.get(2) / shooterRadiansPerSec));
  }
}
