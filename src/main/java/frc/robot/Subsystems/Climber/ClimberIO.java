package frc.robot.Subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {

  @AutoLog
  public class ClimberIOInputs {
    public double climberLeftVoltage = 0.0;
    public double climberLeftSupplyCurrent = 0.0;
    public double climberLeftStatorCurrent = 0.0;
    public double climberLeftTemperature = 0.0;

    public double climberRightVoltage = 0.0;
    public double climberRightSupplyCurrent = 0.0;
    public double climberRightStatorCurrent = 0.0;
    public double climberRightTemperature = 0.0;

    public double climberVelocityMetersPerSec = 0.0;
    public double climberHeightMeters = 0.0;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void setHeight(double height) {}
}
