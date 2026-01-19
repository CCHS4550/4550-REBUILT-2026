package frc.robot.Subsystems.Turret.Rotation;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface RotationIO {

  @AutoLog
  public class RotationIOInputs {

    public double rotationVoltage = 0.0;
    public double rotationSupplyCurrent = 0.0;
    public double rotationStatorCurrent = 0.0;
    public double rotationTemperature = 0.0;
    public double rotationVelocityRotPerSec = 0.0;
    public double rotationAccelRotPerSecSquared = 0.0;
    public double rotationVelocityRadPerSec = 0.0;
    public double rotationAccelRadPerSecSquared = 0.0;

    public Rotation2d rotationAngle = Rotation2d.kZero;
  }

  public default void updateInputs(RotationIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setRotationAngle(Rotation2d angle) {}
}
