package frc.robot.Subsystems.Kicker;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Kicker extends SubsystemBase {
  private KickerIO kickerIO;

  private KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  public enum KickerSystemState {
    IDLE,
    RUNNING
  }

  public enum KickerWantedState {
    IDLE,
    RUNNING
  }

  private KickerSystemState systemState = KickerSystemState.IDLE;
  private KickerWantedState wantedState = KickerWantedState.IDLE;

  public Kicker(KickerIO io) {
    this.kickerIO = io;
  }

  @Override
  public void periodic() {
    kickerIO.updateInputs(kickerInputs);

    systemState = handleStateTransitions();

    applyStates();
  }

  public KickerSystemState handleStateTransitions() {
    switch (wantedState) {
      case IDLE:
        return KickerSystemState.IDLE;
      case RUNNING:
        return KickerSystemState.RUNNING;
      default:
        return KickerSystemState.IDLE;
    }
  }

  public void applyStates() {
    switch (systemState) {
      case IDLE:
        break;
      case RUNNING:
        // kicking logic
        kickerIO.setVoltage(6); // change to reasonable voltage
        break;

      default:
        break;
    }
  }

  public void setWantedState(KickerWantedState wantedState) {
    this.wantedState = wantedState;
  }
}
