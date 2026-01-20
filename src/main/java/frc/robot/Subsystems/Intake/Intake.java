package frc.robot.Subsystems.Intake;

public class Intake {
    public enum WantedState {
        ROTATING,
        IDLE
    }

    public enum SystemState {
        ROTATING,
        IDLE
    }

    private SystemState systemState = SystemState.IDLE;
    private WantedState wantedState = WantedState.IDLE;
}
