package frc.robot.Subsystems.Agitator;

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
    
}
