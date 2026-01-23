package frc.robot.Subsystems.Agitator;

import org.littletonrobotics.junction.AutoLog;

public interface AgitatorIO {
  @AutoLog
  public class AgitatorIOInputs {
    public double agitatorVoltage = 0.0;
    public double agitatorCurrent = 0.0;
    public double agitatorSpeedRPS = 0.0;

    public double temperature = 0.0;
    public double rotationVelocityRotPerSec = 0.0;
    public double rotationAccelRotPerSecSquared = 0.0;
    public double rotationAccelRadPerSecSquared = 0.0;
  }

  public default void updateInputs(AgitatorIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stop(){
    setVoltage(0.0);
  }
}
