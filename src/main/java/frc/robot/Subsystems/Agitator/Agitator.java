package frc.robot.Subsystems.Agitator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Agitator extends SubsystemBase {

  public enum WantedAgitatorState {
    IDLE,
    SPINNING,
    BACK_SPIN
  }

  public enum SystemState {
    IDLE,
    BACK_SPIN,
    SPINNING
  }

  private final AgitatorIO agitatorIO;
  private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();
  private final Timer timer = new Timer();

  private WantedAgitatorState wantedState = WantedAgitatorState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public Agitator(AgitatorIO agitatorIO) {
    this.agitatorIO = agitatorIO;
  }

  @Override
  public void periodic() {
    agitatorIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Agitator", inputs);

    systemState = handleSystemState();
    applyWantedState();

    Logger.recordOutput("Subsystems/Agitator/SystemState", systemState);
    Logger.recordOutput("Subsystems/Agitator/WantedState", wantedState);
  }

  public void setWantedAgitatorState(WantedAgitatorState state) {
    // Idempotency guard: Superstructure calls this every 20ms.
    // Without this, the timer resets every loop tick and BACK_SPIN never ends.
    if (state == this.wantedState) return;
    this.wantedState = state;

    // Timer lifecycle is managed here on state change, not inside handleSystemState().
    if (state == WantedAgitatorState.SPINNING) {
      timer.reset();
      timer.start();
    } else {
      timer.stop();
      timer.reset();
    }
  }

  private SystemState handleSystemState() {
    switch (wantedState) {
      case IDLE:
        return SystemState.IDLE;
      case SPINNING:
        // Back-spin for 0.3s on entry, then switch to forward spin.
        if (timer.isRunning() && !timer.hasElapsed(0.3)) {
          return SystemState.BACK_SPIN;
        }
        return SystemState.SPINNING;
      case BACK_SPIN:
        return SystemState.BACK_SPIN;
      default:
        return SystemState.IDLE;
    }
  }

  private void applyWantedState() {
    switch (systemState) {
      case IDLE:
        agitatorIO.setVoltage(0.0);
        break;
      case BACK_SPIN:
        agitatorIO.setVoltage(-1.0);
        break;
      case SPINNING:
        agitatorIO.setVoltage(4.0);
        break;
    }
  }
}
