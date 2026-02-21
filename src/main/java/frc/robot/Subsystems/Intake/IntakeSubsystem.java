package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// This code is Stupid and needs the be cleaned up.
public class IntakeSubsystem extends SubsystemBase {
  private final Timer timer = new Timer();

  public enum WantedIntakeState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    STOWED,
    IDLE
  }

  public enum SystemState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    MOVING_TO_EXTENSION_ACTIVE,
    MOVING_TO_EXTENSION_PASSIVE,
    STOWED,
    MOVING_TO_STOW,
    IDLE_AT_EXTENDED,
    IDLE_AT_STOW
  }

  private SystemState systemState = SystemState.STOWED;
  private WantedIntakeState wantedState = WantedIntakeState.STOWED;
  private final IntakeIO intakeIO;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public IntakeSubsystem(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  private void applyStates() {
    switch (systemState) {
      case EXTENDED_INTAKING:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(6.0);
        break;
      case MOVING_TO_EXTENSION_ACTIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case MOVING_TO_EXTENSION_PASSIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case MOVING_TO_STOW:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(1.5);
        break;
      default:
        intakeIO.setExtensionVoltage(0);
        intakeIO.setSpinnerVoltage(0);
        break;
    }
  }

  public void setWantedIntakeState(WantedIntakeState state) {
    this.wantedState = state;
    switch (state) {
      case STOWED:
        if (systemState != SystemState.STOWED && systemState != SystemState.IDLE_AT_STOW) {
          systemState = SystemState.MOVING_TO_STOW;
          timer.reset();
          timer.start();
        }
        break;
      case EXTENDED_INTAKING:
        if (systemState != SystemState.EXTENDED_INTAKING
            && systemState != SystemState.EXTENDED_PASSIVE
            && systemState != SystemState.IDLE_AT_EXTENDED) {
          systemState = SystemState.MOVING_TO_EXTENSION_ACTIVE;
          timer.reset();
          timer.start();
        } else {
          systemState = SystemState.EXTENDED_INTAKING;
        }
        break;
      case EXTENDED_PASSIVE:
        if (systemState != SystemState.EXTENDED_INTAKING
            && systemState != SystemState.EXTENDED_PASSIVE
            && systemState != SystemState.IDLE_AT_EXTENDED) {
          systemState = SystemState.MOVING_TO_EXTENSION_PASSIVE;
          timer.reset();
          timer.start();
        } else {
          systemState = SystemState.EXTENDED_PASSIVE;
        }
        break;
      case IDLE:
        if (systemState == SystemState.EXTENDED_INTAKING
            || systemState == SystemState.EXTENDED_PASSIVE
            || systemState == SystemState.MOVING_TO_EXTENSION_ACTIVE
            || systemState == SystemState.MOVING_TO_EXTENSION_PASSIVE
            || systemState == SystemState.IDLE_AT_EXTENDED) {
          this.systemState = SystemState.IDLE_AT_EXTENDED;
        } else {
          this.systemState = SystemState.IDLE_AT_STOW;
        }

      default:
        break;
    }
  }

  @AutoLogOutput(key = "Subsystems/Intake")
  public boolean atWantedAngle() {
    if (wantedState == WantedIntakeState.EXTENDED_INTAKING) {
      if (MathUtil.isNear(0.0, inputs.extensionPosRadians, 0.01)) {
        return true;
      }
    }
    if (wantedState == WantedIntakeState.STOWED) {
      if (MathUtil.isNear(Math.PI / 2, inputs.extensionPosRadians, 0.01)) {
        return true;
      }
    }
    return false;
  }
  ;

  private void endTimerStateAndMoveOn() {
    if (systemState == SystemState.MOVING_TO_EXTENSION_PASSIVE && timer.hasElapsed(0.7)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_PASSIVE;
    }
    if (systemState == SystemState.MOVING_TO_EXTENSION_ACTIVE && timer.hasElapsed(0.7)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.EXTENDED_INTAKING;
    }
    if (systemState == SystemState.MOVING_TO_STOW && timer.hasElapsed(0.75)) {
      timer.stop();
      timer.reset();
      systemState = SystemState.STOWED;
    }
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Intake", inputs);

    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/DesiredState", wantedState);
    endTimerStateAndMoveOn();
    applyStates();
  }
}
