package frc.robot.Subsystems.Agitator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AgitatorSubsystem extends SubsystemBase {
  public enum WantedState {
    IDLE,
    SPINNING
  }

  public enum SystemState {
    IDLE,
    SPINNING
  }

  private SystemState systemState = SystemState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  private final AgitatorIO agitatorIO;

  private AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

  public AgitatorSubsystem(AgitatorIO agitatorIO) {
    this.agitatorIO = agitatorIO;
  }

  @Override
  public void periodic() {
    agitatorIO.updateInputs(inputs);

    systemState = handleSystemState();
    applyWantedState();
  }

  private SystemState handleSystemState() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case SPINNING -> SystemState.SPINNING;
    };
  }

  private void applyWantedState() {
    switch (systemState) {
      case IDLE:
        agitatorIO.setVoltage(0.0);
        break;
      case SPINNING:
        agitatorIO.setVoltage(5.0);
        break;
    }
  }

  public void setWantedState (WantedState wantedState){
    this.wantedState = wantedState;
  }
}
