package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public class IntakeIOInputs {
    // Intake1
    public double spinnerIntakeVoltage = 0.0;
    public double spinnerIntakeSupplyCurrent = 0.0;
    public double spinnerIntakeStatorCurrent = 0.0;
    public double spinnerIntakeTemperature = 0.0;

    public double spinnerIntakeVelocityRadPerSec = 0.0;
    public double spinnerIntakeAccelRadPerSecSquared = 0.0;

    // Intake2
    public double extensionIntakeVoltage = 0.0;
    public double extensionPosRadians;
    public double extensionIntakeSupplyCurrent = 0.0;
    public double extensionIntakeStatorCurrent = 0.0;
    public double extensionIntakeTemperature = 0.0;

    public double extensionIntakeVelocityRadPerSec = 0.0;
    public double extensionIntakeAccelRadPerSecSquared = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setExtensionMotorPositionRad(double rad) {}

  public default void setExtensionVoltage(double voltage) {}

  public default void tareExtensionEncoder() {}

  public default void setSpinnerVoltage(double voltage) {}
}
