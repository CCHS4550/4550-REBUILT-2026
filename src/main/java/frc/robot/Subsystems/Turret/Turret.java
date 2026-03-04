package frc.robot.Subsystems.Turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constant.Constants;
import frc.robot.Constant.Constants.ShooterCalculationConstants;
import frc.robot.Constant.FieldConstants;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Turret.Elevation.ElevationIO;
import frc.robot.Subsystems.Turret.Elevation.ElevationIOInputsAutoLogged;
import frc.robot.Subsystems.Turret.Rotation.RotationIO;
import frc.robot.Subsystems.Turret.Rotation.RotationIOInputsAutoLogged;
import frc.robot.Subsystems.Turret.Shooter.ShooterIO;
import frc.robot.Subsystems.Turret.Shooter.ShooterIOInputsAutoLogged;
import frc.robot.Util.TurretMeasurables;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  // system states, wanted states -> tracking target?, idle, passing over, etc.
  // TurretState -> how fast shooter is going, rotation angle, and hood angle

  /** This is a ratio comparing turret position error per radian per second. */
  private static final double TURRET_POSITION_ERROR_TO_DRIVEBASE_VELOCITY_PROPORTION = 0.213;

  private static final double MIN_ANGLE = Math.toRadians(-195);

  private static final double MAX_ANGLE = Math.toRadians(195);

  private ElevationIO elevationIO;
  private RotationIO rotationIO;
  private ShooterIO shooterIO;

  private TurretMeasurables currentTurretMeasurables;

  private TurretMeasurables wantedTurretMeasurables; // thing we access to inertia

  // private TurretMeasurables fieldOrientedMeasureables;

  private Pose2d desiredPose;
  private double desiredHeight;

  private Pose2d turretPose;

  private boolean atGoal;

  private ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private RotationIOInputsAutoLogged rotationInputs = new RotationIOInputsAutoLogged();
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private boolean targetIsInDeadzoneFlag = false;

  public enum TurretSystemState {
    IDLE,
    TRACKING_TARGET_HUB,
    ACTIVE_SHOOTING_HUB,
    ACTIVE_SHOOTING_PASS,
    TRACKING_PASS,
    ZERO,
    STOW
  };

  public enum TurretWantedState {
    IDLE,
    SHOOT_SCORE,
    PASS_TO_ALLIANCE,
    PASSIVE_TRACK_HUB,
    PASSIVE_TRACK_PASS,
    ZERO,
    STOW
  };

  @AutoLogOutput private TurretSystemState systemState = TurretSystemState.IDLE;
  @AutoLogOutput private TurretWantedState wantedState = TurretWantedState.IDLE;

  public Turret(ElevationIO elevationIO, RotationIO rotationIO, ShooterIO shooterIO) {

    this.elevationIO = elevationIO;
    this.rotationIO = rotationIO;
    this.shooterIO = shooterIO;

    currentTurretMeasurables = new TurretMeasurables(new Rotation2d(), new Rotation2d(), 0);
    wantedTurretMeasurables = new TurretMeasurables(new Rotation2d(), new Rotation2d());
    // fieldOrientedMeasureables = new TurretMeasurables(null, null, 0);

    atGoal = true;
  }

  @Override
  public void periodic() {
    elevationIO.updateInputs(elevationInputs);
    rotationIO.updateInputs(rotationInputs);
    shooterIO.updateInputs(shooterInputs);

    Logger.processInputs("Subsystems/elevation", elevationInputs);
    Logger.processInputs("Subsystems/rotation", rotationInputs);
    Logger.processInputs("Subsystems/shooter", shooterInputs);

    turretPose =
        Robotstate.getInstance()
            .getRobotPoseFromSwerveDriveOdometry()
            .transformBy(Constants.TurretConstants.TURRET_TRANSFORM);

    currentTurretMeasurables =
        new TurretMeasurables(
            elevationInputs.elevationAngle,
            rotationInputs.rotationAngle,
            shooterInputs.shooterVelocityRadPerSec);

    systemState = handleStateTransitions();
    applyStates();
    atGoal = atSetpoint();
  }

  public TurretSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return TurretSystemState.IDLE;

      case SHOOT_SCORE:
        if (atSetpoint()) {
          return TurretSystemState.ACTIVE_SHOOTING;
        } else {
          return TurretSystemState.TRACKING_TARGET;
        }

      case PASS_TO_ALLIANCE:
        if (atSetpoint()) {
          return TurretSystemState.ACTIVE_PASSING;
        } else {
          return TurretSystemState.TRACKING_TARGET;
        }
      case PASSIVE_TRACK_HUB:
        return TurretSystemState.TRACKING_TARGET;

      case STOW:
        return TurretSystemState.STOW;

      default:
        return TurretSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        rotationIO.setVoltage(0);
        elevationIO.setVoltage(0);
        shooterIO.setVoltage(0);
        atGoal = true;
        break;
      case TRACKING_TARGET_HUB:
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;

        Rotation2d desiredElevationAngle = calculateElevationAngleNoInertia(desiredPose);
        Rotation2d desiredRotationAngle = findFieldCentricAngleToTarget(desiredPose);

        wantedTurretMeasurables =
            new TurretMeasurables(desiredElevationAngle, desiredRotationAngle, 0);

        goToWantedState();

        break;

      case ACTIVE_SHOOTING:
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;

        Rotation2d desiredElevationAngle1 = calculateElevationAngleNoInertia(desiredPose);
        Rotation2d desiredRotationAngle1 = findFieldCentricAngleToTarget(desiredPose);

        wantedTurretMeasurables =
            new TurretMeasurables(
                desiredElevationAngle1,
                desiredRotationAngle1,
                Constants.TurretConstants.GEOMETRY_VELOCITY);

        convertToRobotRelativeNonBounded();
        convertToBoundedTurretAngle();
        goToWantedState();

        break;
      case ACTIVE_PASSING:
        desiredPose = FieldConstants.getPassingPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;

        Rotation2d desiredElevationAngle2 = calculateElevationAngleNoInertia(desiredPose);
        Rotation2d desiredRotationAngle2 = findFieldCentricAngleToTarget(desiredPose);

        wantedTurretMeasurables =
            new TurretMeasurables(
                desiredElevationAngle2,
                desiredRotationAngle2,
                Constants.TurretConstants.GEOMETRY_VELOCITY);

        convertToRobotRelativeNonBounded();
        convertToBoundedTurretAngle();
        goToWantedState();

        break;

      case ZERO:
        wantedTurretMeasurables =
            new TurretMeasurables(
                Rotation2d.fromDegrees(
                    Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS),
                Rotation2d.fromDegrees(0),
                0);
        goToWantedState();
        break;
      case STOW:
        wantedTurretMeasurables.rotationAngle = Rotation2d.fromDegrees(90);
        wantedTurretMeasurables.elevationAngle =
            Rotation2d.fromRadians(
                Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
        wantedTurretMeasurables.shooterRadiansPerSec =
            Constants.TurretConstants.SHOOTER_MAX_RADIANS_PER_SEC / 1.6;
        goToWantedState();
        break;
      default:
        break;
    }
  }

  // formal targets will be found and declared through field constants
  public void setFieldRelativeTarget(TurretMeasurables wantedMeasurables) {
    this.wantedTurretMeasurables = wantedMeasurables;
  }

  private void goToWantedState() {
    rotationIO.setRotationAngle(
        wantedTurretMeasurables.rotationAngle.plus(Rotation2d.fromDegrees(0)));
    elevationIO.setElevationAngle(wantedTurretMeasurables.elevationAngle);
    shooterIO.setVelo(
        AngularVelocity.ofBaseUnits(
            wantedTurretMeasurables.shooterRadiansPerSec, RadiansPerSecond));
  }

  private boolean atSetpoint() {
    // change tolerance!
    return MathUtil.isNear(
            currentTurretMeasurables.elevationAngle.getRadians(),
            wantedTurretMeasurables.elevationAngle.getRadians(),
            0.1)
        && MathUtil.isNear(
            currentTurretMeasurables.rotationAngle.getRadians(),
            wantedTurretMeasurables.rotationAngle.getRadians(),
            0.1)
        && MathUtil.isNear(
            currentTurretMeasurables.shooterRadiansPerSec,
            wantedTurretMeasurables.shooterRadiansPerSec,
            0.1);
  }

  public void setWantedState(TurretWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean getAtGoal() {
    return atGoal;
  }

  public boolean getDeadZoneFlag() {
    return targetIsInDeadzoneFlag;
  }

  @AutoLogOutput
  public Rotation2d findFieldCentricAngleToTarget(Pose2d target) {
    var translationToDesiredPoint = target.getTranslation();

    return translationToDesiredPoint.getAngle();
  }

  @AutoLogOutput
  public Rotation2d calculateElevationAngleNoInertia(Pose2d desiredPose) {
    double lateralDistance = desiredPose.getTranslation();

    double elevationAngle =
        Math.atan2(
            lateralDistance
                + Math.sqrt(
                    Math.pow(lateralDistance, 2)
                        - 2
                            * ShooterCalculationConstants.GRAVITATION_CONSTANT
                            * (desiredHeight - ShooterCalculationConstants.TURRET_HEIGHT)
                            * Math.pow(
                                lateralDistance / ShooterCalculationConstants.GEOMETRY_VELOCITY,
                                2)),
            (ShooterCalculationConstants.GRAVITATION_CONSTANT * lateralDistance)
                / Math.pow(ShooterCalculationConstants.GEOMETRY_VELOCITY, 2));
    return new Rotation2d(elevationAngle);
  }

  // public Rotation2d calculateRotationAngleNoInertia(){
  //     double rotationAngle = Math.atan2(desiredPose.getY() - predictedTurretPose.getY(),
  // desiredPose.getX() - predictedTurretPose.getX());
  //     return new Rotation2d(rotationAngle);
  // }

  // private Rotation2d findAngleAdjustmentForRobotInertia(){return new Rotation2d();}

  private void convertToRobotRelativeNonBounded() {
    Rotation2d adjustedAngle =
        wantedTurretMeasurables.rotationAngle.minus(
            Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getRotation());
  }

  public void convertToBoundedTurretAngle() {
    // --- Turret Rotation (Yaw) ---
    double currentTotalRotation = rotationInputs.totalRotationsUnwrapped * 2 * Math.PI;
    double rotationOffset =
        wantedTurretMeasurables.rotationAngle.getRadians()
            - rotationInputs.rotationAngle.getRadians();

    // Wrap to shortest path (-π to π)
    rotationOffset = ((rotationOffset + Math.PI) % (2 * Math.PI)) - Math.PI;

    double finalRotation = currentTotalRotation + rotationOffset;

    // Clamp to turret rotation limits
    finalRotation = Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, finalRotation));
    wantedTurretMeasurables.rotationAngle = Rotation2d.fromRadians(finalRotation);

    // --- Turret Elevation (Pitch) ---
    double elevationOffset =
        wantedTurretMeasurables.elevationAngle.getRadians()
            - elevationInputs.elevationAngle.getRadians();

    // Wrap to shortest path (-π to π)
    elevationOffset = ((elevationOffset + Math.PI) % (2 * Math.PI)) - Math.PI;

    double finalElevation = elevationInputs.elevationAngle.getRadians() + elevationOffset;

    // Clamp to elevation limits
    finalElevation =
        Math.clamp(
            Constants.TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS,
            finalElevation,
            Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
    wantedTurretMeasurables.elevationAngle = Rotation2d.fromRadians(finalElevation);
  }

  private double normalize(double angle) {
    return MathUtil.inputModulus(angle, -Math.PI, Math.PI);
  }

  private boolean isWithinBounds(double angle) {
    return angle >= MIN_ANGLE && angle <= MAX_ANGLE;
  }

  private double clamp(double angle) {
    return MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
  }

  public void setFlywheelSpeed(AngularVelocity velo) {
    shooterIO.setVelo(velo);
  }

  @AutoLogOutput(key = "Subsystems/elevation")
  public double displayTestingRadiansElevation() {
    return elevationInputs.elevationAngle.getRadians();
  }

  @AutoLogOutput(key = "Subsystems/rotation")
  public double displayTestingRadiansRotation() {
    return rotationInputs.rotationAngle.getRadians();
  }

  public void setWantedTurretMeasurables(TurretMeasurables wanted) {
    this.wantedTurretMeasurables = wanted;
  }

  public void setEncoderPositionAtBottom() {
    elevationIO.setEncoderPositionAtBottom();
  }
}
