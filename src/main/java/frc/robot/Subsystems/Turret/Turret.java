package frc.robot.Subsystems.Turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Subsystems.Turret.Elevation.*;
import frc.robot.Subsystems.Turret.Rotation.*;
import frc.robot.Subsystems.Turret.Shooter.*;
import frc.robot.Util.TurretState;

public class Turret extends SubsystemBase {
  // system states, wanted states -> tracking target?, idle, passing over, etc.
  // TurretState -> how fast shooter is going, rotation angle, and hood angle

  private BruinRobotConfig bruinRobotConfig;
  private ElevationIO elevationIO;
  private RotationIO rotationIO;
  private ShooterIO shooterIO;

  private TurretState currentTurretState;
  private TurretState wantedTurretState;

  private boolean atGoal;

  private ElevationIOInputsAutoLogged elevationIOInputsAutoLogged;
  private RotationIOInputsAutoLogged rotationIOInputsAutoLogged;
  private ShooterIOInputsAutoLogged shooterIOInputsAutoLogged;

  public enum TurretSystemState {
    TRACKING_TARGET,
    IDLE,
    ACTIVE_SHOOTING,
    PASSING
  };

  public enum TurretWantedState {
    TRACKING_TARGET,
    IDLE,
    ACTIVE_SHOOTING,
    PASSING
  };

  private TurretSystemState systemState;
  private TurretWantedState wantedState;

  public Turret(ElevationIO elevationIO, RotationIO rotationIO, ShooterIO shooterIO) {
    switch (Constants.currentMode) {
      case REAL:
        elevationIO = new ElevationIOCTRE(bruinRobotConfig);
        rotationIO = new RotationIOCTRE(bruinRobotConfig);
        shooterIO = new ShooterIOCTRE(bruinRobotConfig);
        break;

      case SIM:
        // TBD
        break;

      default:
    }
    currentTurretState = new TurretState(null, null, null, null, null, 0.0);
    wantedTurretState = new TurretState(null, null, null, null, null, 0.0);

    atGoal = true;
  }

  @Override
  public void periodic() {
    elevationIO.updateInputs(elevationIOInputsAutoLogged);
    rotationIO.updateInputs(rotationIOInputsAutoLogged);
    shooterIO.updateInputs(shooterIOInputsAutoLogged);

    currentTurretState =
        new TurretState(
            elevationIOInputsAutoLogged.elevationAngle,
            RadiansPerSecond.of(elevationIOInputsAutoLogged.elevationVelocityRadPerSec),
            rotationIOInputsAutoLogged.rotationAngle,
            RadiansPerSecond.of(rotationIOInputsAutoLogged.rotationVelocityRadPerSec),
            RadiansPerSecond.of(shooterIOInputsAutoLogged.shooterVelocityRadPerSec),
            Timer.getFPGATimestamp());

    systemState = handleStateTransitions();
    atGoal = atSetpoint();
  }

  public TurretSystemState handleStateTransitions() {
    return switch (wantedState) {
      case IDLE -> TurretSystemState.IDLE;
      case TRACKING_TARGET -> TurretSystemState.TRACKING_TARGET;

      case PASSING -> TurretSystemState.PASSING;

      default -> TurretSystemState.IDLE;
    };
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        atGoal = true;
        break;
      case TRACKING_TARGET:
        atGoal = atSetpoint();

        if (!atGoal) {
          applyTurretState(wantedTurretState);
        } else {
          applyTurretState(currentTurretState);
        }
        break;
      case PASSING:
        // this is basically just TRACKING_TARGET, but the target is somewhere else.
        // basically the TurretState will come from the ShotCalculator, which will return a
        // TurretState based on the
        // position of the bot, turret, etc., and will determine the target based on the WantedState
        // of the Turret Class
        // ex. if we're tracking, it'll shoot out angles, but not shoot velo, and so one from there
        break;
      default:
        break;
    }
  }

  public Command applyTurretState(TurretState wantedTurretState) {
    return new RunCommand(
        () -> {
          elevationIO.setElevationAngle(wantedTurretState.getElevationAngle());
          rotationIO.setRotationAngle(wantedTurretState.getRotationAngle());
          shooterIO.setRadPerSec(wantedTurretState.getShooterAngularVelocity());
        },
        this);
  }

  public void setFieldRelativeTarget(TurretState wantedState) {
    this.wantedTurretState = wantedState;
  }

  public boolean atSetpoint() {
    // change tolerance!
    double currentShooterAngVelo =
        currentTurretState.getShooterAngularVelocity().in(RadiansPerSecond);
    double wantedShooterAngVelo =
        wantedTurretState.getShooterAngularVelocity().in(RadiansPerSecond);

    return MathUtil.isNear(
            currentTurretState.getElevationAngle().getRadians(),
            wantedTurretState.getElevationAngle().getRadians(),
            2.0)
        && MathUtil.isNear(
            currentTurretState.getRotationAngle().getRadians(),
            wantedTurretState.getRotationAngle().getRadians(),
            2.0)
        && MathUtil.isNear(currentShooterAngVelo, wantedShooterAngVelo, 2.0);
  }

  public void setWantedState(TurretWantedState wantedState) {
    this.wantedState = wantedState;
  }
}
