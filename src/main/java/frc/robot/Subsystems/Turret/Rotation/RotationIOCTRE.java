package frc.robot.Subsystems.Turret.Rotation;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class RotationIOCTRE implements RotationIO {
  private TalonFX rotationMotor;
  private CANcoder rotationEncoder;
  private TalonFXConfiguration rotationConfig;
  private MotionMagicVelocityVoltage rotationController;

  public RotationIOCTRE(int rotationID, int encoderID, String canBusString) {
    rotationMotor = new TalonFX(rotationID, canBusString);
    rotationEncoder = new CANcoder(encoderID);

    rotationConfig = new TalonFXConfiguration();
    rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rotationConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    rotationConfig.Slot0.kP = 0.0;
    rotationConfig.Slot0.kI = 0.0;
    rotationConfig.Slot0.kD = 0.0;

    rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    rotationConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    rotationConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rotationMotor.getConfigurator().apply(rotationConfig);
  }

  @Override
  public void updateInputs(RotationIOInputs inputs) {}

  @Override
  public void setVoltage(double voltage) {}
}
