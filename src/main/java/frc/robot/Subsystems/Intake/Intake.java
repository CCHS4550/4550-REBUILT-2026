package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final Timer timer = new Timer();

  public enum WantedIntakeState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    STOWED,
    PUMPING,
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
    IDLE_AT_STOW,
    MOVING_TO_UPPER_PUMP,
    MOVING_TO_LOWER_PUMP,
    IDLE_AT_LOWER_PUMP,
    IDLE_AT_UPPER_PUMP,
    UPPER_PUMP_TO_STOW,
    LOWER_PUMP_TO_STOW,
    UPPER_PUMP_TO_EXTENDED_ACTIVE,
    UPPER_PUMP_TO_EXTENDED_PASSIVE,
    LOWER_PUMP_TO_EXTENDED_ACTIVE,
    LOWER_PUMP_TO_EXTENDED_PASSIVE,
    EXTENDED_TO_LOWER_PUMP,
    STOW_TO_UPPER_PUMP
  }

  private enum Location {
    STOWED,
    UPPER_PUMP,
    LOWER_PUMP,
    EXTENDED
  }

  private Location location = Location.STOWED;
  private SystemState systemState = SystemState.STOWED;
  private WantedIntakeState wantedState = WantedIntakeState.STOWED;
  private final IntakeIO intakeIO;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
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
        // FIX #1: All cases below were missing break statements, causing fall-through
        // to default (0V) and motors never running for pump/transition states.
      case MOVING_TO_UPPER_PUMP:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case MOVING_TO_LOWER_PUMP:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case UPPER_PUMP_TO_STOW:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case LOWER_PUMP_TO_STOW:
        intakeIO.setExtensionVoltage(3.0);
        intakeIO.setSpinnerVoltage(1.5);
        break;
      case UPPER_PUMP_TO_EXTENDED_ACTIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case UPPER_PUMP_TO_EXTENDED_PASSIVE:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case LOWER_PUMP_TO_EXTENDED_ACTIVE:
        intakeIO.setExtensionVoltage(-0.5);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case LOWER_PUMP_TO_EXTENDED_PASSIVE:
        intakeIO.setExtensionVoltage(-0.5);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case EXTENDED_TO_LOWER_PUMP:
        intakeIO.setExtensionVoltage(0.1);
        intakeIO.setSpinnerVoltage(0.0);
        break;
      case STOW_TO_UPPER_PUMP:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      default:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(0.0);
        break;
    }
  }

  public void setWantedIntakeState(WantedIntakeState state) {
    // FIX #2: Idempotency guard. Superstructure calls this every 20ms loop tick.
    // Without this, every call restarts the timer and re-evaluates location before
    // the mechanism has actually moved, immediately skipping timed transitions.
    if (state == this.wantedState) return;
    this.wantedState = state;

    switch (state) {
      case STOWED:
        if (location == Location.STOWED) {
          systemState = SystemState.STOWED;
        } else if (location == Location.LOWER_PUMP) {
          systemState = SystemState.LOWER_PUMP_TO_STOW;
          timer.reset();
          timer.start();
        } else if (location == Location.UPPER_PUMP) {
          systemState = SystemState.UPPER_PUMP_TO_STOW;
          timer.reset();
          timer.start();
        } else if (location == Location.EXTENDED) {
          systemState = SystemState.MOVING_TO_STOW;
          timer.reset();
          timer.start();
        }
        // FIX #2 cont: location is NOT updated here. It is only updated in
        // endTimerStateAndMoveOn() once the timed transition actually completes.
        break;

      case EXTENDED_INTAKING:
        if (location == Location.STOWED) {
          systemState = SystemState.MOVING_TO_EXTENSION_ACTIVE;
          timer.reset();
          timer.start();
        } else if (location == Location.LOWER_PUMP) {
          systemState = SystemState.LOWER_PUMP_TO_EXTENDED_ACTIVE;
          timer.reset();
          timer.start();
        } else if (location == Location.UPPER_PUMP) {
          systemState = SystemState.UPPER_PUMP_TO_EXTENDED_ACTIVE;
          timer.reset();
          timer.start();
        } else if (location == Location.EXTENDED) {
          systemState = SystemState.EXTENDED_INTAKING;
        }
        break;

      case EXTENDED_PASSIVE:
        if (location == Location.STOWED) {
          systemState = SystemState.MOVING_TO_EXTENSION_PASSIVE;
          timer.reset();
          timer.start();
        } else if (location == Location.LOWER_PUMP) {
          systemState = SystemState.LOWER_PUMP_TO_EXTENDED_PASSIVE;
          timer.reset();
          timer.start();
        } else if (location == Location.UPPER_PUMP) {
          systemState = SystemState.UPPER_PUMP_TO_EXTENDED_PASSIVE;
          timer.reset();
          timer.start();
        } else if (location == Location.EXTENDED) {
          systemState = SystemState.EXTENDED_PASSIVE;
        }
        break;

      case PUMPING:
        // FIX #4: PUMPING previously fell through to IDLE due to missing break on
        // some branches. Now all branches explicitly break.
        if (location == Location.EXTENDED) {
          systemState = SystemState.EXTENDED_TO_LOWER_PUMP;
          timer.reset();
          timer.start();
        } else if (location == Location.STOWED) {
          systemState = SystemState.STOW_TO_UPPER_PUMP;
          timer.reset();
          timer.start();
        } else if (location == Location.UPPER_PUMP) {
          // Already in pump zone - start ping-pong immediately
          systemState = SystemState.MOVING_TO_LOWER_PUMP;
          timer.reset();
          timer.start();
        } else if (location == Location.LOWER_PUMP) {
          systemState = SystemState.MOVING_TO_UPPER_PUMP;
          timer.reset();
          timer.start();
        }
        break;

      case IDLE:
        if (location == Location.STOWED) {
          systemState = SystemState.IDLE_AT_STOW;
        } else if (location == Location.LOWER_PUMP) {
          systemState = SystemState.IDLE_AT_LOWER_PUMP;
        } else if (location == Location.UPPER_PUMP) {
          systemState = SystemState.IDLE_AT_UPPER_PUMP;
        } else if (location == Location.EXTENDED) {
          systemState = SystemState.IDLE_AT_EXTENDED;
        }
        timer.stop();
        timer.reset();
        break;

      default:
        break;
    }
  }

  @AutoLogOutput(key = "Subsystems/Intake/AtWantedAngle")
  public boolean atWantedAngle() {
    return systemState == SystemState.EXTENDED_INTAKING
        || systemState == SystemState.EXTENDED_PASSIVE
        || systemState == SystemState.STOWED;
  }

  private void endTimerStateAndMoveOn() {
    switch (systemState) {
      case MOVING_TO_EXTENSION_PASSIVE:
        if (timer.hasElapsed(0.7)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.EXTENDED_PASSIVE;
          location = Location.EXTENDED;
        }
        break;

      case MOVING_TO_EXTENSION_ACTIVE:
        if (timer.hasElapsed(0.7)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.EXTENDED_INTAKING;
          location = Location.EXTENDED;
        }
        break;

      case MOVING_TO_STOW:
        if (timer.hasElapsed(0.75)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.STOWED;
          location = Location.STOWED;
        }
        break;

        // FIX #5: Pump ping-pong is gated on wantedState == PUMPING.
        // If the wanted state has changed (e.g. to STOWED), the ping-pong stops
        // at the next timer elapse and location is updated so the next
        // setWantedIntakeState() call (now unblocked by the idempotency guard)
        // can start the correct exit transition from the right location.
      case MOVING_TO_LOWER_PUMP:
        if (timer.hasElapsed(0.25)) {
          timer.reset();
          location = Location.LOWER_PUMP;
          if (wantedState == WantedIntakeState.PUMPING) {
            systemState = SystemState.MOVING_TO_UPPER_PUMP;
            timer.start();
          } else {
            timer.stop();
            systemState = SystemState.IDLE_AT_LOWER_PUMP;
          }
        }
        break;

      case MOVING_TO_UPPER_PUMP:
        if (timer.hasElapsed(0.25)) {
          timer.reset();
          location = Location.UPPER_PUMP;
          if (wantedState == WantedIntakeState.PUMPING) {
            systemState = SystemState.MOVING_TO_LOWER_PUMP;
            timer.start();
          } else {
            timer.stop();
            systemState = SystemState.IDLE_AT_UPPER_PUMP;
          }
        }
        break;

      case LOWER_PUMP_TO_EXTENDED_ACTIVE:
        if (timer.hasElapsed(0.05)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.EXTENDED_INTAKING;
          location = Location.EXTENDED;
        }
        break;

      case LOWER_PUMP_TO_EXTENDED_PASSIVE:
        if (timer.hasElapsed(0.05)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.EXTENDED_PASSIVE;
          location = Location.EXTENDED;
        }
        break;

      case UPPER_PUMP_TO_EXTENDED_ACTIVE:
        if (timer.hasElapsed(0.3)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.EXTENDED_INTAKING;
          location = Location.EXTENDED;
        }
        break;

      case UPPER_PUMP_TO_EXTENDED_PASSIVE:
        if (timer.hasElapsed(0.3)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.EXTENDED_PASSIVE;
          location = Location.EXTENDED;
        }
        break;

      case LOWER_PUMP_TO_STOW:
        if (timer.hasElapsed(0.7)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.STOWED;
          location = Location.STOWED;
        }
        break;

      case UPPER_PUMP_TO_STOW:
        if (timer.hasElapsed(0.4)) {
          timer.stop();
          timer.reset();
          systemState = SystemState.STOWED;
          location = Location.STOWED;
        }
        break;

      case STOW_TO_UPPER_PUMP:
        if (timer.hasElapsed(0.4)) {
          timer.reset();
          location = Location.UPPER_PUMP;
          if (wantedState == WantedIntakeState.PUMPING) {
            systemState = SystemState.MOVING_TO_LOWER_PUMP;
            timer.start();
          } else {
            timer.stop();
            systemState = SystemState.IDLE_AT_UPPER_PUMP;
          }
        }
        break;

        // FIX #3: Timer was 0.0 (always immediately true). Changed to 0.1s
        // to actually apply voltage for at least one cycle before transitioning.
      case EXTENDED_TO_LOWER_PUMP:
        if (timer.hasElapsed(0.1)) {
          timer.reset();
          location = Location.LOWER_PUMP;
          if (wantedState == WantedIntakeState.PUMPING) {
            systemState = SystemState.MOVING_TO_UPPER_PUMP;
            timer.start();
          } else {
            timer.stop();
            systemState = SystemState.IDLE_AT_LOWER_PUMP;
          }
        }
        break;

      default:
        break;
    }
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Intake", inputs);
    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/DesiredState", wantedState);
    Logger.recordOutput("Subsystems/Intake/Location", location);
    endTimerStateAndMoveOn();
    applyStates();
  }
}
