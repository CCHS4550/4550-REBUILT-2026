package frc.robot.Subsystems.Turret.Rotation;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;

public class RotationIOCTRE implements RotationIO {

  private TalonFX rotationMotor;
  private CANcoder rotationEncoder;
  private TalonFXConfiguration rotationConfig;
  private CANcoderConfiguration encoderConfig;
  private MotionMagicVoltage motionMagicVoltage;
  private final StatusSignal<Angle> rotationAngleRotations;
  private final StatusSignal<Voltage> rotationAppliedVolts;
  private final StatusSignal<Current> rotationSupplyCurrentAmps;
  private final StatusSignal<Current> rotationStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> rotationVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> rotationAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> rotationMotorTemp;

  public RotationIOCTRE(BruinRobotConfig bruinRobotConfig) {
    motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);

    rotationMotor =
        new TalonFX(
            bruinRobotConfig.ROTATION_MOTOR.getDeviceNumber(),
            bruinRobotConfig.ROTATION_MOTOR.getBus()); // creates motor

    rotationEncoder =
        new CANcoder(
            bruinRobotConfig.ROTATION_CANCODER.getDeviceNumber(),
            bruinRobotConfig.ROTATION_CANCODER
                .getBus()); // creates CANCoder, which should be connected to the motor electrically

    // I should probably set up these constants in like RobotConfig, but I just want to try and
    // complete this out
    rotationConfig = new TalonFXConfiguration();
    rotationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rotationConfig.CurrentLimits.SupplyCurrentLimit = 20.0;
    rotationConfig.CurrentLimits.StatorCurrentLimit = 45.0;

    rotationConfig.Slot0.kP = bruinRobotConfig.getTurretConfig().rotationKp;
    rotationConfig.Slot0.kI = bruinRobotConfig.getTurretConfig().rotationKi;
    rotationConfig.Slot0.kD = bruinRobotConfig.getTurretConfig().rotationKd;
    rotationConfig.Slot0.kS = bruinRobotConfig.getTurretConfig().rotationKs;
    rotationConfig.Slot0.kV = bruinRobotConfig.getTurretConfig().rotationKv;
    rotationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rotationConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // Hardware-level protection
    rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    rotationConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Math.toRadians(120.0) / Constants.TurretConstants.ROTATION_POSITION_COEFFICIENT;

    rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    rotationConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Math.toRadians(-120.0) / Constants.TurretConstants.ROTATION_POSITION_COEFFICIENT;
    ;

    rotationConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
    rotationConfig.MotionMagic.MotionMagicAcceleration = 0.3; // some constant idk

    Phoenix6Util.applyAndCheckConfiguration(rotationMotor, rotationConfig, 5);

    encoderConfig = new CANcoderConfiguration();
    encoderConfig
        .MagnetSensor
        .withMagnetOffset(-0.7)
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    rotationEncoder.getConfigurator().apply(encoderConfig);

    rotationMotor.setPosition(0.0);
    rotationAngleRotations = rotationEncoder.getAbsolutePosition();
    rotationAppliedVolts = rotationMotor.getMotorVoltage();
    rotationSupplyCurrentAmps = rotationMotor.getSupplyCurrent();
    rotationStatorCurrentAmps = rotationMotor.getStatorCurrent();
    rotationVelocityRotationsPerSec = rotationEncoder.getVelocity();
    rotationAccelerationRotationsPerSecSquared = rotationMotor.getAcceleration();
    rotationMotorTemp = rotationMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(RotationIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        rotationAppliedVolts,
        rotationSupplyCurrentAmps,
        rotationStatorCurrentAmps,
        rotationMotorTemp,
        rotationAccelerationRotationsPerSecSquared);
    BaseStatusSignal.refreshAll(rotationAngleRotations, rotationVelocityRotationsPerSec);
    inputs.rotationVoltage = rotationAppliedVolts.getValueAsDouble();
    inputs.rotationSupplyCurrent = rotationSupplyCurrentAmps.getValueAsDouble();
    inputs.rotationStatorCurrent = rotationStatorCurrentAmps.getValueAsDouble();
    inputs.rotationTemperature = rotationMotorTemp.getValueAsDouble();

    inputs.rotationVelocityRadPerSec =
        rotationVelocityRotationsPerSec.getValueAsDouble()
            * Constants.TurretConstants.ROTATION_POSITION_COEFFICIENT_TO_ENCODER;
    inputs.rotationAccelRadPerSecSquared =
        rotationAccelerationRotationsPerSecSquared.getValueAsDouble()
            * Constants.TurretConstants.ROTATION_POSITION_COEFFICIENT;

    inputs.rotationAngle =
        Rotation2d.fromRadians(
            rotationAngleRotations.getValueAsDouble()
                * Constants.TurretConstants.ROTATION_POSITION_COEFFICIENT_TO_ENCODER);
  }

  @Override
  public void setRotationAngle(Rotation2d angle) {
    rotationMotor.setControl(
        motionMagicVoltage.withPosition(
            angle.getRadians() / Constants.TurretConstants.ROTATION_POSITION_COEFFICIENT));
  }

  @Override
  public void setVoltage(double volts) {
    rotationMotor.setVoltage(volts);
  }
}
