package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robotstate;
import frc.robot.Subsystems.Agitator.Agitator;
import frc.robot.Subsystems.Agitator.Agitator.WantedAgitatorState;
import frc.robot.Subsystems.Drive.SwerveSubsystem;
import frc.robot.Subsystems.Intake.Intake;
import frc.robot.Subsystems.Intake.Intake.WantedIntakeState;
import frc.robot.Subsystems.Kicker.Kicker;
import frc.robot.Subsystems.Kicker.Kicker.KickerWantedState;
import frc.robot.Subsystems.Turret.Turret;
import frc.robot.Subsystems.Turret.Turret.TurretWantedState;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Intake intake;
  private final Kicker kicker;
  private final Turret turret;
  private final Agitator agitator;
  private boolean INTAKE_ACTIVE = false;

  @AutoLogOutput private WantedSuperstructureState wantedState1 = WantedSuperstructureState.IDLE;
  @AutoLogOutput private SystemState systemState = SystemState.IDLE;

  public Superstructure(
      SwerveSubsystem swerveSubsystem,
      Intake intake,
      Kicker kicker,
      Turret turret,
      Agitator agitator) {
    this.swerveSubsystem = swerveSubsystem;
    this.intake = intake;
    this.kicker = kicker;
    this.turret = turret;
    this.agitator = agitator;
  }

  @Override
  public void periodic() {
    systemState = handleStateTransitions();
    applyStates();
  }

  public void setWantedSuperstructureState(
      WantedSuperstructureState wantedState, boolean INTAKE_ACTIVE) {
    this.INTAKE_ACTIVE = INTAKE_ACTIVE;
    this.wantedState1 = wantedState;
  }

  public void setIntakeActive(boolean INTAKE_ACTIVE) {
    this.INTAKE_ACTIVE = INTAKE_ACTIVE;
  }

  private void applyStates() {
    switch (systemState) {
      case IDLE:
        intake.setWantedIntakeState(WantedIntakeState.IDLE);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.IDLE);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case EXTEND_INTAKE:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        break;
      case ZERO:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.ZERO);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case INTAKING_ZERO:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.ZERO);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case STOW:
        intake.setWantedIntakeState(WantedIntakeState.STOWED);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.STOW);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case INTAKING_TRACKING_PASS:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.PASS_TO_ALLIANCE);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case TRACKING_PASS:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.PASS_TO_ALLIANCE);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case INTAKING_ACTIVE_PASS:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        kicker.setWantedKickerState(KickerWantedState.RUNNING);
        turret.setWantedState(TurretWantedState.PASS_TO_ALLIANCE);
        agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);
        break;
      case ACTIVE_PASS:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        kicker.setWantedKickerState(KickerWantedState.RUNNING);
        turret.setWantedState(TurretWantedState.PASS_TO_ALLIANCE);
        agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);
        break;
      case INTAKING_TRACKING_SHOOT:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.SHOOT_SCORE);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case TRACKING_SHOOT:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        kicker.setWantedKickerState(KickerWantedState.IDLE);
        turret.setWantedState(TurretWantedState.SHOOT_SCORE);
        agitator.setWantedAgitatorState(WantedAgitatorState.IDLE);
        break;
      case INTAKING_ACTIVE_SHOOT:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
        kicker.setWantedKickerState(KickerWantedState.RUNNING);
        turret.setWantedState(TurretWantedState.SHOOT_SCORE);
        agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);
        break;
      case ACTIVE_SHOOT:
        intake.setWantedIntakeState(WantedIntakeState.EXTENDED_PASSIVE);
        kicker.setWantedKickerState(KickerWantedState.RUNNING);
        turret.setWantedState(TurretWantedState.SHOOT_SCORE);
        agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);
        break;
      case PRACTICE_INDEXING:
        intake.setWantedIntakeState(WantedIntakeState.IDLE);
        kicker.setWantedKickerState(KickerWantedState.RUNNING);
        turret.setWantedState(TurretWantedState.SHOOT_SCORE);
        agitator.setWantedAgitatorState(WantedAgitatorState.SPINNING);
        break;
    }
  }

  private SystemState handleStateTransitions() {
    switch (wantedState1) {
      case IDLE:
        return SystemState.IDLE;
      case STOW:
        return SystemState.STOW;
      case ZERO:
        if (INTAKE_ACTIVE) {
          return SystemState.INTAKING_ZERO;
        }
        return SystemState.ZERO;
      case EXTEND_INTAKE:
        return SystemState.EXTEND_INTAKE;
      case PASSIVE_TRACKING:
        return returnTrackingTarget();
      case ACTIVE_SHOOT:
        if (INTAKE_ACTIVE) {
          if (turret.getAtGoal()) {
            return SystemState.INTAKING_ACTIVE_SHOOT;
          }
          return SystemState.INTAKING_TRACKING_SHOOT;
        } else if (turret.getAtGoal()) {
          return SystemState.ACTIVE_SHOOT;
        }
        return SystemState.TRACKING_SHOOT;
      case ACTIVE_PASS:
        if (INTAKE_ACTIVE) {
          if (turret.getAtGoal()) {
            return SystemState.INTAKING_ACTIVE_PASS;
          }
          return SystemState.INTAKING_TRACKING_PASS;
        } else if (turret.getAtGoal()) {
          return SystemState.ACTIVE_PASS;
        }
        return SystemState.TRACKING_PASS;
      case PRACTICE_INDEXING:
        break;
    }
    return SystemState.IDLE;
  }

  public enum WantedSuperstructureState {
    IDLE,
    STOW,
    ZERO,
    EXTEND_INTAKE,
    PASSIVE_TRACKING,
    ACTIVE_SHOOT,
    ACTIVE_PASS,
    PRACTICE_INDEXING
  }

  private enum SystemState {
    IDLE,
    STOW,
    INTAKING_ZERO,
    ZERO,
    EXTEND_INTAKE,
    INTAKING_TRACKING_PASS,
    TRACKING_PASS,
    INTAKING_TRACKING_SHOOT,
    TRACKING_SHOOT,
    INTAKING_ACTIVE_SHOOT,
    ACTIVE_SHOOT,
    INTAKING_ACTIVE_PASS,
    ACTIVE_PASS,
    PRACTICE_INDEXING
  }

  private SystemState returnTrackingTarget() {
    double x = Robotstate.getInstance().getRobotPoseFromSwerveDriveOdometry().getX();
    if (x > 4.75 && x < 11.75) {
      if (INTAKE_ACTIVE) {
        return SystemState.INTAKING_TRACKING_PASS;
      }
      return SystemState.TRACKING_PASS;
    }
    if (x <= 4.75 && x >= 0) {
      if (INTAKE_ACTIVE) {
        return SystemState.INTAKING_TRACKING_SHOOT;
      }
      return SystemState.TRACKING_SHOOT;
    }
    if (x >= 11.75 && x <= 16.5) {
      if (INTAKE_ACTIVE) {
        return SystemState.INTAKING_TRACKING_SHOOT;
      }
      return SystemState.TRACKING_SHOOT;
    }
    return SystemState.ZERO;
  }
}
