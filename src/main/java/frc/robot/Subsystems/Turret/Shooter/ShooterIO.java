package frc.robot.Subsystems.Turret.Shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {

  @AutoLog
  public class ShooterIOInputs {
    // assuming there's only 1 shooter motor 4 now
    public double shooterVoltage = 0.0;
    public double shooterSupplyCurrent = 0.0;
    public double shooterStatorCurrent = 0.0;
    public double shooterTemperature = 0.0;

    // maybe rotations per second is better here idk

    public double shooterVelocityRadPerSec = 0.0;
    public double shooterAccelRadPerSecSquared = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setRadPerSec(double radPerSec) {}

  public default void setRotPerSec(double rotPerSec) {}

  public default void setVoltage(double voltage) {}
}
