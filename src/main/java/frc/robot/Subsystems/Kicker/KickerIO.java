package frc.robot.Subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
  @AutoLog
  public class KickerIOInputs {
    public double kickerVoltage = 0;
    public double kickerPosRad = 0.0;
    public double kickerSupplyCurrent = 0.0;
    public double kickerStatorCurrent = 0.0;
    public double kickerTemperature = 0.0;

    public double kickerVelocityRadPerSec = 0.0;
    public double kickerAccelRadPerSecSquared = 0.0;
  }

  default void updateInputs(KickerIOInputs inputs) {}

  default void setVoltage(double voltage) {}
}
