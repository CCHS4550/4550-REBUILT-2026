package frc.robot.Subsystems.Intake;

public class Intake {
    public enum WantedState {
        INTAKING,
        MOVINGUP,
        MOVINGDOWN,
        IDLE
    }

    public enum SystemState {
        INTAKING,
        MOVINGUP,
        MOVINGDOWN,
        IDLE
    }

    private SystemState handleStateTransition() {
      return switch (wantedState) {
        case MOVINGUP -> SystemState.MOVINGUP;
        case MOVINGDOWN-> SystemState.MOVINGDOWN;
        case INTAKING -> SystemState.INTAKING;

        default -> SystemState.IDLE;
        };
    }

    private void applyStates() {
        switch (systemState) {
        default:
        //case IDLE:
            //break;
        case INTAKING:
            //Make the intake (extension) rotate

        case MOVINGUP:
            //Make spinning motor move up
        case MOVINGDOWN:
            //Make spinning moter move down
        }
    }

    public void setState(WantedState state) {
        this.wantedState = state;
    }

    private SystemState systemState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;
}
