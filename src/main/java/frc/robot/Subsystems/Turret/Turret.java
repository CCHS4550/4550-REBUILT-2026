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
import edu.wpi.first.math.util.Units;
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
    ACTIVE_PASSING,
    STOW,
    ZERO,
    TESTING,
    PRACTICE_SHOOTING
  };

  public enum TurretWantedState {
    IDLE,
    SHOOT_SCORE,
    PASS_TO_ALLIANCE,
    PASSIVE_TRACK,
    STOW,
    ZERO,
    TESTING,
    PRACTICE_SHOOTING
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

      case TESTING:
        return TurretSystemState.TESTING;
      case ZERO:
        return TurretSystemState.ZERO;
      case STOW:
        return TurretSystemState.STOW;
      case PRACTICE_SHOOTING:
        return TurretSystemState.PRACTICE_SHOOTING;

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

        // rotationIO.setVoltage(0);
        // elevationIO.setVoltage(0);
        // shooterIO.setVoltage(0);
        // atGoal = true;
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

      case TESTING:
        wantedTurretMeasurables =
            new TurretMeasurables(Rotation2d.fromDegrees(70), Rotation2d.fromDegrees(0), 200);
        goToWantedState();
        break;
      case ZERO:
        wantedTurretMeasurables =
            new TurretMeasurables(Rotation2d.fromDegrees(Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS), Rotation2d.fromDegrees(0), 0);
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
      case PRACTICE_SHOOTING:
        shooterIO.setVelo(RadiansPerSecond.of(30));
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
     double currentTotalRadians = (rotationInputs.totalRotationsUnwrapped * 2 * Math.PI);
        double closestOffset = wantedTurretMeasurables.rotationAngle.getRadians() - rotationInputs.rotationAngle.getRadians();
        if (closestOffset > Math.PI) {

            closestOffset -= 2 * Math.PI;

        } else if (closestOffset < -Math.PI) {
            closestOffset += 2 * Math.PI;
        }

        double finalOffset = currentTotalRadians + closestOffset;
        if ((currentTotalRadians + closestOffset) % (2 * Math.PI)
                == (currentTotalRadians - closestOffset)
                        % (2 * Math.PI)) { // If the offset can go either way, go closer to zero
            if (finalOffset > 0) {
                finalOffset = currentTotalRadians - Math.abs(closestOffset);
            } else {
                finalOffset = currentTotalRadians + Math.abs(closestOffset);
            }
        }
        if (finalOffset > Units.degreesToRadians(MAX_ANGLE)) { // if past upper rotation limit
            finalOffset -= (2 * Math.PI);
        } else if (finalOffset < Units.degreesToRadians(MIN_ANGLE)) { // if below lower rotation limit
            finalOffset += (2 * Math.PI);
        }
        wantedTurretMeasurables.rotationAngle= Rotation2d.fromRadians(finalOffset);

        double clampedElevation = MathUtil.clamp(wantedTurretMeasurables.elevationAngle.getDegrees(), Constants.TurretConstants.SHALLOWEST_POSSIBLE_ELEVATION_ANGLE_RADIANS, Constants.TurretConstants.STEEPEST_POSSIBLE_ELEVATION_ANGLE_RADIANS);
        wantedTurretMeasurables.elevationAngle= Rotation2d.fromDegrees(clampedElevation);

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
