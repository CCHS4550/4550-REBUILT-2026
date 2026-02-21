package frc.robot.Subsystems.Turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
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

public class Turret extends SubsystemBase {
  // system states, wanted states -> tracking target?, idle, passing over, etc.
  // TurretState -> how fast shooter is going, rotation angle, and hood angle

  /** This is a ratio comparing turret position error per radian per second. */
  private static final double TURRET_POSITION_ERROR_TO_DRIVEBASE_VELOCITY_PROPORTION = 0.213;

  private static final double MIN_ANGLE = 0.0;

  private static final double MAX_ANGLE = Math.toRadians(359);

  private static final double HYSTERESIS_THRESHOLD = Math.toRadians(20.0);

  private ElevationIO elevationIO;
  private RotationIO rotationIO;
  private ShooterIO shooterIO;

  private TurretMeasurables currentTurretMeasurables;

  private TurretMeasurables wantedTurretMeasurables; // thing we access to inertia
  private TurretMeasurables noInertiaMeasurables;
  // private TurretMeasurables fieldOrientedMeasureables;

  private Pose2d desiredPose;
  private double desiredHeight;

  private Pose2d turretPose;
  private Pose2d predictedTurretPose;
  private ChassisSpeeds chassisSpeeds;
  private Vector<N3> robotSpeedVector;

  private boolean atGoal;

  private ElevationIOInputsAutoLogged elevationInputs = new ElevationIOInputsAutoLogged();
  private RotationIOInputsAutoLogged rotationInputs = new RotationIOInputsAutoLogged();
  private ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private boolean targetIsInDeadzoneFlag = false;

  public enum TurretSystemState {
    TRACKING_TARGET,
    IDLE,
    ACTIVE_SHOOTING,
    ACTIVE_PASSING
  };

  public enum TurretWantedState {
    IDLE,
    SHOOT_SCORE,
    PASS_TO_ALLIANCE,
    PASSIVE_TRACK
  };

  private TurretSystemState systemState = TurretSystemState.IDLE;
  private TurretWantedState wantedState = TurretWantedState.IDLE;

  public Turret(ElevationIO elevationIO, RotationIO rotationIO, ShooterIO shooterIO) {

    this.elevationIO = elevationIO;
    this.rotationIO = rotationIO;
    this.shooterIO = shooterIO;

    currentTurretMeasurables = new TurretMeasurables(null, null, 0);
    wantedTurretMeasurables = new TurretMeasurables(null, null);
    // fieldOrientedMeasureables = new TurretMeasurables(null, null, 0);

    atGoal = true;
  }

  @Override
  public void periodic() {
    elevationIO.updateInputs(elevationInputs);
    rotationIO.updateInputs(rotationInputs);
    shooterIO.updateInputs(shooterInputs);

    chassisSpeeds = Robotstate.getInstance().getRobotChassisSpeeds();
    turretPose =
        Robotstate.getInstance()
            .getRobotPoseFromSwerveDriveOdometry()
            .transformBy(Constants.TurretConstants.TURRET_TRANSFORM);
    predictedTurretPose =
        turretPose.transformBy(
            new Transform2d(
                chassisSpeeds.vxMetersPerSecond * ShooterCalculationConstants.TIME_DELAY,
                chassisSpeeds.vyMetersPerSecond * ShooterCalculationConstants.TIME_DELAY,
                new Rotation2d(
                    chassisSpeeds.omegaRadiansPerSecond * ShooterCalculationConstants.TIME_DELAY)));
    currentTurretMeasurables =
        new TurretMeasurables(
            elevationInputs.elevationAngle,
            rotationInputs.rotationAngle,
            shooterInputs.shooterVelocityRadPerSec);

    systemState = handleStateTransitions();
    applyStates();

    setWantedState(wantedState);
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
      case PASSIVE_TRACK:
        return TurretSystemState.TRACKING_TARGET;

      default:
        return TurretSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        atGoal = true;
        break;
      case TRACKING_TARGET:
        // Rotation2d calculatedAngle = findFieldCentricAngleToTarget(new
        // Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        // convertToBoundedTurretAngle(calculatedAngle.getRadians());
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;
        // put calc here
        noInertiaMeasurables =
            new TurretMeasurables(
                calculateElevationAngleNoInertia(desiredPose),
                findFieldCentricAngleToTarget(desiredPose),
                Constants.ShooterCalculationConstants.GEOMETRY_VELOCITY);
        robotSpeedVector =
            VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0);

        wantedTurretMeasurables.updateWithCartesianVector(
            noInertiaMeasurables.getVector().minus(robotSpeedVector));
        wantedTurretMeasurables.shooterRadiansPerSec = 1;
        convertToRobotRelativeNonBounded();
        convertToBoundedTurretAngle();
        goToWantedState();

        break;
      case ACTIVE_SHOOTING:
        // Rotation2d calculatedAngle = findFieldCentricAngleToTarget(new
        // Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        // convertToBoundedTurretAngle(calculatedAngle.getRadians());
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;
        // put calc here
        noInertiaMeasurables =
            new TurretMeasurables(
                calculateElevationAngleNoInertia(desiredPose),
                findFieldCentricAngleToTarget(desiredPose),
                Constants.ShooterCalculationConstants.GEOMETRY_VELOCITY);
        robotSpeedVector =
            VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0);

        wantedTurretMeasurables.updateWithCartesianVector(
            noInertiaMeasurables.getVector().minus(robotSpeedVector));
        convertToRobotRelativeNonBounded();
        convertToBoundedTurretAngle();
        goToWantedState();

        break;
      case ACTIVE_PASSING:
        // Rotation2d calculatedAngle = findFieldCentricAngleToTarget(new
        // Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        // convertToBoundedTurretAngle(calculatedAngle.getRadians());
        desiredPose = FieldConstants.getScoringPose();
        desiredHeight = FieldConstants.HUB_HEIGHT;
        // put calc here
        noInertiaMeasurables =
            new TurretMeasurables(
                calculateElevationAngleNoInertia(desiredPose),
                findFieldCentricAngleToTarget(desiredPose),
                Constants.ShooterCalculationConstants.GEOMETRY_VELOCITY);
        robotSpeedVector =
            VecBuilder.fill(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0);

        wantedTurretMeasurables.updateWithCartesianVector(
            noInertiaMeasurables.getVector().minus(robotSpeedVector));
        convertToRobotRelativeNonBounded();
        convertToBoundedTurretAngle();
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
    rotationIO.setRotationAngle(wantedTurretMeasurables.rotationAngle);
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
            2.0)
        && MathUtil.isNear(
            currentTurretMeasurables.rotationAngle.getRadians(),
            wantedTurretMeasurables.rotationAngle.getRadians(),
            2.0)
        && MathUtil.isNear(
            currentTurretMeasurables.shooterRadiansPerSec,
            wantedTurretMeasurables.shooterRadiansPerSec,
            2.0);
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

  public Rotation2d findFieldCentricAngleToTarget(Pose2d target) {
    var translationToDesiredPoint =
        target.getTranslation().minus(predictedTurretPose.getTranslation());

    return translationToDesiredPoint.getAngle();
  }

  public Rotation2d calculateElevationAngleNoInertia(Pose2d desiredPose) {
    double lateralDistance =
        desiredPose.getTranslation().getDistance(predictedTurretPose.getTranslation());

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
    wantedTurretMeasurables.rotationAngle =
        adjustedAngle.plus(
            Rotation2d.fromRadians(
                TURRET_POSITION_ERROR_TO_DRIVEBASE_VELOCITY_PROPORTION
                    * Robotstate.getInstance().getRobotChassisSpeeds().omegaRadiansPerSecond));
  }

 public void convertToBoundedTurretAngle() {
    double current = normalize(rotationInputs.rotationAngle.getRadians());
    double target  = normalize(wantedTurretMeasurables.rotationAngle.getRadians());

    // TRUE shortest path (always between -180 and +180 degrees)
    double shortestDelta = MathUtil.inputModulus(target - current, -Math.PI, Math.PI);

    // The long way around the circle
    double longestDelta = shortestDelta - (Math.signum(shortestDelta) * 2.0 * Math.PI);

    double shortEnd = current + shortestDelta;
    double longEnd  = current + longestDelta;

    boolean shortValid = isWithinBounds(shortEnd);
    boolean longValid  = isWithinBounds(longEnd);

    double chosenEnd;

    if (shortValid) {
        // Preferred path is legal
        chosenEnd = shortEnd;
        targetIsInDeadzoneFlag = false;

    } else if (longValid) {
        // Short path crosses hardstop — measure overshoot
        double overshoot = distanceOutsideBounds(shortEnd);

        if (overshoot > HYSTERESIS_THRESHOLD) {
            // Commit to long flip
            chosenEnd = longEnd;
            targetIsInDeadzoneFlag = false; // We are actively moving to a valid target
        } else {
            // Stay pinned at wall
            chosenEnd = clamp(shortEnd);
            targetIsInDeadzoneFlag = true; // Tell the shooter we can't track right now
        }

    } else {
        // Neither valid (extreme edge case, usually only if MIN/MAX are messed up)
        chosenEnd = clamp(shortEnd);
        targetIsInDeadzoneFlag = true; 
    }

    wantedTurretMeasurables.rotationAngle = Rotation2d.fromRadians(clamp(chosenEnd));
}

  private double normalize(double angle) {
    return MathUtil.inputModulus(angle, 0.0, 2.0 * Math.PI);
  }

  private boolean isWithinBounds(double angle) {
    return angle >= MIN_ANGLE && angle <= MAX_ANGLE;
  }

  private double clamp(double angle) {
    return MathUtil.clamp(angle, MIN_ANGLE, MAX_ANGLE);
  }

  private double distanceOutsideBounds(double angle) {
    if (angle < MIN_ANGLE) return MIN_ANGLE - angle;
    if (angle > MAX_ANGLE) return angle - MAX_ANGLE;
    return 0.0;
  }
}
