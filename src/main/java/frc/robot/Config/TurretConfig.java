package frc.robot.Config;

public class TurretConfig {
  public double rotationKp;
  public double rotationKi;
  public double rotationKd;
  public double rotationKs;
  public double rotationKv;

  public double elevationKp;
  public double elevationKi;
  public double elevationKd;
  public double elevationKs;
  public double elevationKv;

  public double shooterKp;
  public double shooterKi;
  public double shooterKd;
  public double shooterKs;
  public double shooterKv;

  public TurretConfig withRotationKp(double rotationKp) {
    this.rotationKp = rotationKp;
    return this;
  }

  public TurretConfig withRotationKi(double rotationKi) {
    this.rotationKi = rotationKi;
    return this;
  }

  public TurretConfig withRotationKd(double rotationKd) {
    this.rotationKd = rotationKd;
    return this;
  }

  public TurretConfig withRotationKs(double rotationKs) {
    this.rotationKs = rotationKs;
    return this;
  }

  public TurretConfig withRotationKv(double rotationKv) {
    this.rotationKv = rotationKv;
    return this;
  }

  public TurretConfig withElevationKp(double elevationKp) {
    this.elevationKp = elevationKp;
    return this;
  }

  public TurretConfig withElevationKi(double elevationKi) {
    this.elevationKi = elevationKi;
    return this;
  }

  public TurretConfig withElevationKd(double elevationKd) {
    this.elevationKd = elevationKd;
    return this;
  }

  public TurretConfig withElevationKs(double elevationKs) {
    this.elevationKs = elevationKs;
    return this;
  }

  public TurretConfig withElevationKv(double elevationKv) {
    this.elevationKv = elevationKv;
    return this;
  }

  public TurretConfig withShooterKp(double shooterKp) {
    this.shooterKp = shooterKp;
    return this;
  }

  public TurretConfig withShooterKi(double shooterKi) {
    this.shooterKi = shooterKi;
    return this;
  }

  public TurretConfig withShooterKd(double shooterKd) {
    this.shooterKd = shooterKd;
    return this;
  }

  public TurretConfig withShooterKs(double shooterKs) {
    this.shooterKs = shooterKs;
    return this;
  }

  public TurretConfig withShooterKv(double shooterKv) {
    this.shooterKv = shooterKv;
    return this;
  }
}
