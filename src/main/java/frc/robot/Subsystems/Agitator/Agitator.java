package frc.robot.Subsystems.Agitator;
public class Agitator extends SubsystemBase implements AgitatorIO {
public class Agitator {
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
  @Override
  public void periodic(){
        systemState = handleSystemState();
        applyWantedState();
  }

  private SystemState handleSystemState() {
    switch (systemState) {
      case IDLE:
        if (wantedState == WantedState.SPINNING) {
          return SystemState.SPINNING;
        }
        break;
      case SPINNING:
        if (wantedState == WantedState.IDLE) {
          return SystemState.IDLE;
        }
        break;
    }
    return systemState;
  }

    private void applyWantedState() {
        switch (systemState) {
        case IDLE:
            agitatorIO.setVoltage(0.0);
            break;
        case SPINNING:
            agitatorIO.setVoltage(12.0);
            break;
        }

    }

    
}
}
