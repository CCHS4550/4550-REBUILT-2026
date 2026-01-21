package frc.robot.Config;
public class ClimberConfig {
    public double climberKp;
    public double climberKi;
    public double climberKd;

    public ClimberConfig withclimberKp(double climberKp) {
        this.climberKp = climberKp;
        return this;
    }

    public ClimberConfig withclimberKi(double climberKi) {
        this.climberKi = climberKi;
        return this;
    }

    public ClimberConfig withclimberKd(double climberKd) {
        this.climberKd = climberKd;
        return this;
    }
}