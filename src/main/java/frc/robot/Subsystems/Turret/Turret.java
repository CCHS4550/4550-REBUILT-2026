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
import frc.robot.Subsystems.Turret.Elevation.*;
import frc.robot.Subsystems.Turret.Rotation.*;
import frc.robot.Subsystems.Turret.Shooter.*;
import frc.robot.Util.TurretMeasurables;

public class Turret extends SubsystemBase {
  // system states, wanted states -> tracking target?, idle, passing over, etc.
  // TurretState -> how fast shooter is going, rotation angle, and hood angle

  private ElevationIO elevationIO;
  private RotationIO rotationIO;
  private ShooterIO shooterIO;

  private TurretMeasurables currentTurretMeasurables;

  private TurretMeasurables wantedTurretMeasurables; // thing we access to inertia
  private TurretMeasurables noInertiaMeasurables;
  private TurretMeasurables fieldOrientedMeasureables;

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
    fieldOrientedMeasureables = new TurretMeasurables(null, null, 0);

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
        // convertToClosestBoundedTurretAngle(calculatedAngle.getRadians());
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
        wantedTurretMeasurables.shooterRadiansPerSec = 0;
        goToWantedState();

        break;
      case ACTIVE_SHOOTING:
        // Rotation2d calculatedAngle = findFieldCentricAngleToTarget(new
        // Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        // convertToClosestBoundedTurretAngle(calculatedAngle.getRadians());
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
        goToWantedState();

        break;
      case ACTIVE_PASSING:
        // Rotation2d calculatedAngle = findFieldCentricAngleToTarget(new
        // Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        // convertToClosestBoundedTurretAngle(calculatedAngle.getRadians());
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

  /**
   * Sets the robot-relative target angle for the turret. First the closest path from current turret
   * angle to the target angle is calculated. If the path is found to be move outside the bounds,
   * the path will adjust to follow the next closest path.
   *
   * @param targetAngleRadians Target angle in radians
   * @return next absolute angle for the robot to move to
   */
  public void convertToClosestBoundedTurretAngle(double targetAngleRadians) {
    double currentTotalRadians = (rotationInputs.totalRotationsUnwrapped * 2 * Math.PI);
    double closestOffset = targetAngleRadians - rotationInputs.rotationAngle.getRadians();
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
    if (finalOffset
        > Constants.TurretConstants
            .FORWARD_ROTATION_LIMIT_RADIANS) { // if past upper rotation limit
      finalOffset -= (2 * Math.PI);
    } else if (finalOffset
        < Constants.TurretConstants
            .BACKWARDS_ROTATION_LIMIT_RADIANS) { // if below lower rotation limit
      finalOffset += (2 * Math.PI);
    }

    wantedTurretMeasurables.rotationAngle = Rotation2d.fromRadians(finalOffset);
  }
}
