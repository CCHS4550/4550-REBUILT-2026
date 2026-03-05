package frc.robot.Subsystems.Turret.Shooter;

import edu.wpi.first.units.measure.AngularVelocity;

/**
 * This class exists so we instantialize subsystems in sim to test logic, what is done with values
 * is unimportant
 */
public class ShooterIOTest implements ShooterIO {
  public ShooterIOTest() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void setVelo(AngularVelocity Velo) {}
}
