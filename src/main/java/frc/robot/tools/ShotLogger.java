package frc.robot.tools;

public class ShotLogger {
    private final double timestamp;

    public ShotLogger(double timestamp) {
        this.timestamp = timestamp;
    }

    public double getTimestamp() {
        return timestamp;
    }

    @Override
    public String toString() {
        return String.format("Shot{t=%.3f}", timestamp);
    }
}