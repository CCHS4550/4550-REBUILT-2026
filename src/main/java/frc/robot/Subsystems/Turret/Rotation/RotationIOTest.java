package frc.robot.Subsystems.Turret.Rotation;

/**
 * This class exists so we instantialize subsystems in sim to test logic, what is done with values
 * is unimportant
 */
public class RotationIOTest implements RotationIO {
  public RotationIOTest() {}

  @Override
  public void updateInputs(RotationIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setRotationAngle(double radians) {}
}
