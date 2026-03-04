package frc.robot.tools;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunableNumber {
    private String name;
    private double defaultValue;
    private double previousValue;

    public TunableNumber(String name, double defaultValue) {
        this.name = name;
        this.defaultValue = defaultValue;
        this.previousValue = defaultValue;
        SmartDashboard.putNumber(name, defaultValue);
    }

    public double get() {
        previousValue = SmartDashboard.getNumber(name, defaultValue);
        return previousValue;
    }

    public void set(double newValue) {
        SmartDashboard.putNumber(name, newValue);
    }

    public boolean changed() {
        double currentValue = SmartDashboard.getNumber(name, defaultValue);
        return currentValue != previousValue;
    }
}