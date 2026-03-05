package frc.robot.Subsystems.Turret.Elevation;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This class exists so we instantialize subsystems in sim to test logic, what is done with values
 * is unimportant
 */
public class ElevationIOTest implements ElevationIO {
  public ElevationIOTest() {}

  @Override
  public void updateInputs(ElevationIOInputs inputs) {}

  @Override
  public void setElevationAngle(Rotation2d angle) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setEncoderPositionAtBottom() {}
}
