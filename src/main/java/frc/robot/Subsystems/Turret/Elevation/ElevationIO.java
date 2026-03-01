package frc.robot.Subsystems.Turret.Elevation;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ElevationIO {

  @AutoLog
  public class ElevationIOInputs {
    public double elevationVoltage = 0.0;
    public double elevationSupplyCurrent = 0.0;
    public double elevationStatorCurrent = 0.0;
    public double elevationTemperature = 0.0;

    public double elevationVelocityRadPerSec = 0.0;
    public double elevationAccelRadPerSecSquared = 0.0;

    public Rotation2d elevationAngle = Rotation2d.kZero;
  }

  default void updateInputs(ElevationIOInputs inputs) {}

  default void setElevationAngle(Rotation2d angle) {}

  default void setVoltage(double voltage) {}

  default void setEncoderPositionAtBottom() {}
}
