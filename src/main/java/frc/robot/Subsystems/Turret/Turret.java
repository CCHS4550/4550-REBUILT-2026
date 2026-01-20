package frc.robot.Subsystems.Turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private TurretMeasurables wantedTurretMeasurables;

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

    atGoal = true;
  }

  @Override
  public void periodic() {
    elevationIO.updateInputs(elevationInputs);
    rotationIO.updateInputs(rotationInputs);
    shooterIO.updateInputs(shooterInputs);

    systemState = handleStateTransitions();
    applyStates();

    currentTurretMeasurables = new TurretMeasurables(elevationInputs.elevationAngle, rotationInputs.rotationAngle, shooterInputs.shooterVelocityRadPerSec);
    atGoal = atSetpoint();
  }
  
  public TurretSystemState handleStateTransitions() {
     switch (wantedState) {
      case IDLE: return TurretSystemState.IDLE;

      case SHOOT_SCORE: if(atSetpoint()){
        return TurretSystemState.ACTIVE_SHOOTING;
      }
      else{
        return TurretSystemState.TRACKING_TARGET;
      }

      case PASS_TO_ALLIANCE: if(atSetpoint()){
        return TurretSystemState.ACTIVE_PASSING;
      }
      else{
        return TurretSystemState.TRACKING_TARGET;
      }
      case PASSIVE_TRACK: return TurretSystemState.TRACKING_TARGET;

      default: return TurretSystemState.IDLE;
  }

}

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        atGoal = true;
        break;
      case TRACKING_TARGET:
        Rotation2d calculatedAngle = findFieldCentricAngleToTarget(new Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        convertFieldCentricAngletoRobotCentric(calculatedAngle);
        findElevationAngleToTarget(new Pose2d());
        goToWantedState();
        break;
      case ACTIVE_SHOOTING:
        calculatedAngle = findFieldCentricAngleToTarget(new Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        convertFieldCentricAngletoRobotCentric(calculatedAngle);
        findElevationAngleToTarget(new Pose2d());
        goToWantedState();
        break;
      case ACTIVE_PASSING:
        calculatedAngle = findFieldCentricAngleToTarget(new Pose2d()).plus(findAngleAdjustmentForRobotInertia());
        convertFieldCentricAngletoRobotCentric(calculatedAngle);
        findElevationAngleToTarget(new Pose2d());
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

  private void goToWantedState(){
    rotationIO.setRotationAngle(wantedTurretMeasurables.rotationAngle);
    elevationIO.setElevationAngle(wantedTurretMeasurables.elevationAngle);
    shooterIO.setVelo(AngularVelocity.ofBaseUnits(wantedTurretMeasurables.shooterRadiansPerSec, RadiansPerSecond));
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
        && MathUtil.isNear(currentTurretMeasurables.shooterRadiansPerSec, wantedTurretMeasurables.shooterRadiansPerSec, 2.0);
  }

  public void setWantedState(TurretWantedState wantedState) {
    this.wantedState = wantedState;
  }

  public boolean getAtGoal(){
    return atGoal;
  }

  private Rotation2d findFieldCentricAngleToTarget(Pose2d target){return new Rotation2d();}

  private Rotation2d findAngleAdjustmentForRobotInertia(){return new Rotation2d();}

  //This will mutate wantedturretstate
  private void convertFieldCentricAngletoRobotCentric(Rotation2d FieldCentricFacingAngle){}

  //This will mutate wantedturretstate
  private void findElevationAngleToTarget(Pose2d target){}
}
