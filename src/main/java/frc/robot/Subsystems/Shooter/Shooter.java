// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Shooter;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Constant.FieldConstants;
import frc.robot.tools.ShotCalculator.ShotSolution;
import frc.robot.tools.ShotLogger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;

  public enum ShooterState {
    DEFAULT,
    IDLE,
    PHYSICS_SHOOT,
    NORMAL_SHOOT,
    ZERO,
    STOW
  }

  private ShooterState wantedState = ShooterState.IDLE;
  private ShooterState systemState = ShooterState.IDLE;
  private Translation3d _trajectorySetpoint = new Translation3d(0, 0, 0);
  private Rotation2d idleTurretAngle = new Rotation2d(0.0);
  private ShotSolution wantedShotSolution = new ShotSolution(idleTurretAngle, 0.0, idleTurretAngle, 0.0, 0.0);
  private BruinRobotConfig config = new BruinRobotConfig();
  public List<ShotLogger> shotLog = new ArrayList<>();

  private double lastRPM = 0.0;
  private double lastShotTs = 0.0;

  Translation3d target = new Translation3d(FieldConstants.getScoringPose().getX(), FieldConstants.getScoringPose().getY(), FieldConstants.HUB_HEIGHT);
  Pose2d turretPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
  Pose2d robotPose = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
  Translation3d turretFieldPosition = new Translation3d(0.0, 0.0, 0.0);

  private Debouncer readyToShootDebouncer = new Debouncer(0.15, Debouncer.DebounceType.kFalling);

  public Shooter() {
    this.io = new ShooterIOComp(config);
  }

  public void setWantedState(ShooterState wantedState) {
    this.wantedState = wantedState;
  }

  public void zero(){
    setTurretAngle(Rotation2d.fromDegrees(0));
    moveHoodToAngle(Rotation2d.fromDegrees(Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS));
    setFlywheelRPM(0);
}

  public void setWantedState(ShooterState wantedState, Translation3d trajectorySetpoint) {
    this.wantedState = wantedState;
    this._trajectorySetpoint = trajectorySetpoint;
  }

  public void setWantedState(ShooterState wantedState, ShotSolution wantedShotSolution) {
    this.wantedState = wantedState;
    this.wantedShotSolution = wantedShotSolution;
  }

  private ShooterState handleStateTransition() {
    switch (wantedState) {
      case DEFAULT:
        return ShooterState.DEFAULT;
      case IDLE:
        return ShooterState.IDLE;
      case PHYSICS_SHOOT:
        return ShooterState.PHYSICS_SHOOT;
      case NORMAL_SHOOT:
        return ShooterState.NORMAL_SHOOT;
      case ZERO:
        return ShooterState.ZERO;
      case STOW:
        return ShooterState.STOW;
      default:
        return ShooterState.IDLE;
    }
  }

  private void physicsShoot() {
    moveHoodToAngle(io.getHoodAngleSetpointForTrajectory(_trajectorySetpoint));
    setTurretAngle(io.getTurretAngleSetpointForTrajectory(_trajectorySetpoint));
    setFlywheelRPM(io.getFlywheelRPMSetpointForTrajectory(_trajectorySetpoint));
  }

  private void normalShoot() {
    Logger.recordOutput("Shooter/Wanted Turret Angle", wantedShotSolution.turretAngle.getDegrees());
    if (wantedShotSolution.robotVelocity.isPresent()) {
      Rotation2d prediction = io.getFutureSetpointEstimate(wantedShotSolution.turretAngle,
          wantedShotSolution.robotVelocity.get().omegaRadiansPerSecond, 0.1);
      Logger.recordOutput("Shooter/Predicted Turret Angle",
          prediction.getDegrees());
      setTurretAngle(prediction);
    } else {
      setTurretAngle(wantedShotSolution.turretAngle);

    }
    moveHoodToAngle(wantedShotSolution.hoodAngle);
    setFlywheelRPM(wantedShotSolution.flywheelRPM);
  }

  public boolean readyToShoot() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(wantedShotSolution.hoodAngle)
            .getRadians());
    double turretAngleError;
    if (wantedShotSolution.robotVelocity.isPresent()) {
      Rotation2d prediction = io.getFutureSetpointEstimate(wantedShotSolution.turretAngle,
          wantedShotSolution.robotVelocity.get().omegaRadiansPerSecond, 0.1);
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(prediction)
              .getRadians());
    } else {
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(wantedShotSolution.turretAngle)
              .getRadians());

    }

    double turretPrecisionRequired = Math
        .atan((FieldConstants.HUB_RADIUS - FieldConstants.BALL_WIDTH) / wantedShotSolution.distanceToTarget);
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - wantedShotSolution.flywheelRPM);

    Logger.recordOutput("Shooter/Hood Error", hoodAngleError);
    Logger.recordOutput("Shooter/Hood Precision Required", Constants.TurretConstants.HOOD_PRECISION);
    Logger.recordOutput("Shooter/Turret Error", turretAngleError);
    Logger.recordOutput("Shooter/Turret Precision Required", turretPrecisionRequired);
    Logger.recordOutput("Shooter/Flywheel RPM Error", flywheelRPMError);
    Logger.recordOutput("Shooter/Flywheel RPM Precision Required", Constants.TurretConstants.FLYWHEEL_RPM_PRECISION);
    boolean ready = hoodAngleError < Constants.TurretConstants.HOOD_PRECISION
        && turretAngleError < turretPrecisionRequired
        && flywheelRPMError < Constants.TurretConstants.FLYWHEEL_RPM_PRECISION;
    Logger.recordOutput("Shooter/Ready To Shoot Raw", ready);
    boolean debouncedReady = readyToShootDebouncer.calculate(ready);
    Logger.recordOutput("Shooter/Ready To Shoot Filtered", debouncedReady);
    return debouncedReady;
  }

  public boolean readyToPass() {
    double hoodAngleError = Math
        .abs(getHoodAngle()
            .minus(wantedShotSolution.hoodAngle)
            .getRadians());
    double turretAngleError;
    if (wantedShotSolution.robotVelocity.isPresent()) {
      Rotation2d prediction = io.getFutureSetpointEstimate(wantedShotSolution.turretAngle,
          wantedShotSolution.robotVelocity.get().omegaRadiansPerSecond, 0.2);
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(prediction)
              .getRadians());
    } else {
      turretAngleError = Math.abs(
          getRobotRelativeTurretAngle()
              .minus(wantedShotSolution.turretAngle)
              .getRadians());

    }
    double turretPrecisionRequired = Math
        .atan(((33.39 / 39.37) - FieldConstants.BALL_WIDTH) / wantedShotSolution.distanceToTarget);
    Logger.recordOutput("Shooter/Turret Precision Required", Math.toDegrees(turretPrecisionRequired));
    double flywheelRPMError = Math
        .abs(getFlywheelRPM()
            - wantedShotSolution.flywheelRPM);
    return hoodAngleError < Constants.TurretConstants.HOOD_FEED_PRECISION
        && turretAngleError < turretPrecisionRequired
        && flywheelRPMError < Constants.TurretConstants.FLYWHEEL_RPM_FEED_PRECISION;
  }

  public Rotation2d getHoodAngle() {
    return io.getHoodAngle();
  }

  public Rotation2d getRobotRelativeTurretAngle() {
    return io.getTurretAngle();
  }

  public double getRelativeRobotRelativeTurretAngle() {
    return io.getRelativeTurretAngleRadians();
  }

  public double getFlywheelRPM() {
    return io.getFlywheelRPM();
  }

  public void moveHoodToAngle(Rotation2d angle) {
    io.moveHoodToAngle(angle);
  }

//   public void setHoodAngle(Rotation2d angle) {
//   io.setHoodAngle(angle);
//   }

  public void setTurretAngle(Rotation2d angle) {
    io.setTurretAngle(getRelativeAngleFromRotation2d(angle));
  }

  public void setFlywheelRPM(double rpm) {
    io.setFlywheelRPM(rpm);
  }

  public void zeroTurretToEncoder() {
    io.zeroTurretToEncoder();
  }

  public Translation3d getCurrentShooterTrajectory() {
    double mag = io.getFlywheelRPM() / (2.0/39.37);
    Rotation2d hoodAngle = io.getHoodAngle();
    double vz = mag * hoodAngle.getSin();
    double vr = mag * hoodAngle.getCos();
    Rotation2d turretAngle = io.getTurretAngle();
    double vx = vr * turretAngle.getCos();
    double vy = vr * turretAngle.getSin();
    return new Translation3d(vx, vy, vz);
  }

  protected double getRelativeAngleFromRotation2d(Rotation2d angle) {
    double relativeAngle = angle.getRadians();
    double currentTurretAngle = io.getRelativeTurretAngleRadians();
    double delta = relativeAngle - currentTurretAngle;
    delta %= 2 * Math.PI;
    if (delta > Math.PI) {
      delta -= 2 * Math.PI;
    } else if (delta < -Math.PI) {
      delta += 2 * Math.PI;
    }
    double wanted = currentTurretAngle + delta;
    return clampAngleToTurretRange(wanted);
  }

  protected double clampAngleToTurretRange(double radians) {
    double clampedAngle = radians;
    while (clampedAngle < Constants.TurretConstants.ROTATION_MINANGLE.getRadians()) {
      clampedAngle += 2 * Math.PI;
    }
    while (clampedAngle > Constants.TurretConstants.ROTATION_MAXANGLE.getRadians()) {
      clampedAngle -= 2 * Math.PI;
    }
    if (clampedAngle < Constants.TurretConstants.ROTATION_MINANGLE.getRadians()) {
      clampedAngle = Constants.TurretConstants.ROTATION_MINANGLE.getRadians();
    }
    if (clampedAngle > Constants.TurretConstants.ROTATION_MAXANGLE.getRadians()) {
      clampedAngle = Constants.TurretConstants.ROTATION_MAXANGLE.getRadians();
    }
    return clampedAngle;
  }

  public void passIdleTurretAngleToIdle(Rotation2d angle) {
    Logger.recordOutput("Shooter/Idle Turret Angle", angle.getDegrees());
    idleTurretAngle = angle;
  }

  @Override
  public void periodic() {
    io.updateInputs();
    Logger.recordOutput("Shooter/Hood Angle", getHoodAngle().getDegrees());
    Logger.recordOutput("Shooter/Turret Angle", getRobotRelativeTurretAngle().getDegrees());
    Logger.recordOutput("Shooter/Flywheel RPM", getFlywheelRPM());
    Logger.recordOutput("Shooter/Velocity Magnitude",
        Math.sqrt(Math.pow(_trajectorySetpoint.getX(), 2)
            + Math.pow(_trajectorySetpoint.getY(), 2)
            + Math.pow(_trajectorySetpoint.getZ(), 2)));
    Logger.recordOutput("Shooter/Shooter State", systemState);
    Logger.recordOutput("Shooter/Turret Current", io.getTurretCurrent());
    Logger.recordOutput("Shooter/Hood Current", io.getHoodCurrent());
    Logger.recordOutput("Shooter/Flywheel Current", io.getFlywheelCurrent());
    Logger.recordOutput("States/Shooter State", systemState);

    ShooterState newState = handleStateTransition();
    if (newState != systemState) {
      systemState = newState;
    }
    switch (systemState) {
      case DEFAULT:
        trackTarget();
        break;
      case IDLE:
        break;
      case PHYSICS_SHOOT:
        physicsShoot();
        break;
      case NORMAL_SHOOT:
        normalShoot();
        break;
      case ZERO:
        zero();
        break;
      case STOW:
        stow();
        break;
        
      default:
        break;
    }
    if (readyToShoot()) {
      detectAndLogShot();
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Shooter/Shot Log", shotLog.toString());
    }
  }

  public void stow(){
    setTurretAngle(Rotation2d.fromDegrees(90));
    moveHoodToAngle(Rotation2d.fromRadians(Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS));
    setFlywheelRPM(0);
  }

  private void trackTarget() {
    io.setTurretAngle(getRelativeAngleFromRotation2d(idleTurretAngle));
    io.setHoodAngle(new Rotation2d(Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS));
    io.setFlywheelPercent(0.0);
  }

  public void detectAndLogShot() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    double curRPM = getFlywheelRPM();
    double current = io.getFlywheelCurrent();
    double accel = io.getFlywheelAcceleration();
    final double DEBOUNCE_S = Constants.TurretConstants.SHOT_DEBOUNCE_S;
    final double ACCEL_LOW = Constants.TurretConstants.SHOT_ACCEL_LOW;
    final double ACCEL_HIGH = Constants.TurretConstants.SHOT_ACCEL_HIGH;
    final double RPM_LOW = Constants.TurretConstants.SHOT_RPM_DELTA_LOW;
    final double RPM_HIGH = Constants.TurretConstants.SHOT_RPM_DELTA_HIGH;
    final double SPIKE_CURRENT = Constants.TurretConstants.SHOT_SPIKE_CURRENT;

    if (!Double.isFinite(lastRPM) || lastRPM <= 1.0) {
      lastRPM = curRPM;
    } else {
      double rpmDelta = lastRPM - curRPM;
      double timeSinceLast = now - lastShotTs;
      boolean debounceOk = timeSinceLast >= DEBOUNCE_S;
      boolean accelStable = accel >= ACCEL_LOW && accel <= ACCEL_HIGH;
      boolean velStable = rpmDelta >= RPM_LOW && rpmDelta <= RPM_HIGH;
      boolean currentSpike = current >= SPIKE_CURRENT;
      boolean shootingState = systemState == ShooterState.NORMAL_SHOOT || systemState == ShooterState.PHYSICS_SHOOT;

      if (debounceOk && accelStable && velStable && currentSpike && shootingState) {
        shotLog.add(new frc.robot.tools.ShotLogger(now));
        lastShotTs = now;
      }
      // Logger.recordOutput("Shooter/Raw RPM", curRPM);
      // Logger.recordOutput("Shooter/RPM Delta", rpmDelta);
      // Logger.recordOutput("Shooter/Accel (RPM/s)", accel);
      // Logger.recordOutput("Shooter/Shooter Current", current);
      // Logger.recordOutput("Shooter/Accel Stable", accelStable);
      // Logger.recordOutput("Shooter/Vel Stable", velStable);
      // Logger.recordOutput("Shooter/Current Spike", currentSpike);
      // Logger.recordOutput("Shooter/Time Since Last Shot", timeSinceLast);
      Logger.recordOutput("Shooter/System State", systemState.toString());
    }

    lastRPM = curRPM;
    // Logger.recordOutput("Shooter/Last Shot Timestamp", lastShotTs);
    Logger.recordOutput("Shooter/Shot Log Size", shotLog.size());
  }
}