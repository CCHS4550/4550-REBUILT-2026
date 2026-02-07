package frc.robot.Subsystems.Agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
  @AutoLog
  public class AgitatorIOInputs {
    public double agitatorVoltage = 0.0;
    public double agitatorSupplyCurrent = 0.0;
    public double agitatorStatorCurrent = 0.0;
    public double agitatorPositionRadians = 0.0;

    public double temperature = 0.0;
    public double rotationVelocityRadPerSec = 0.0;
    public double rotationAccelRadPerSecSquared = 0.0;
  }

  public default void updateInputs(AgitatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}
}
