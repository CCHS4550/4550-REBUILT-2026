package frc.robot.Subsystems.Turret.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class ShooterIOCTRE implements ShooterIO {

  private TalonFX shooterMotor;
  private TalonFX shooterMotor2;

  private TalonFXConfiguration shooterConfig;
  private MotionMagicVelocityVoltage motionMagicVelocityVoltage;

  private final StatusSignal<Voltage> shooterAppliedVoltage;
  private final StatusSignal<Current> shooterSupplyCurrent;
  private final StatusSignal<Current> shooterStatorCurrent;
  private final StatusSignal<AngularVelocity> shooterVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> shooterAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> shooterMotorTemperature;

  public ShooterIOCTRE(BruinRobotConfig bruinRobotConfig) {
    Follower follower =
        new Follower(bruinRobotConfig.SHOOTER_MOTOR.getDeviceNumber(), MotorAlignmentValue.Aligned);
    shooterMotor =
        new TalonFX(
            bruinRobotConfig.SHOOTER_MOTOR.getDeviceNumber(),
            bruinRobotConfig.SHOOTER_MOTOR.getBus());
    shooterMotor2 =
        new TalonFX(
            bruinRobotConfig.SHOOTER_MOTOR_2.getDeviceNumber(),
            bruinRobotConfig.SHOOTER_MOTOR_2.getBus());
    motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(0);

    shooterConfig = new TalonFXConfiguration();
    shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    shooterConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    shooterConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    shooterConfig.Slot0.kP = bruinRobotConfig.getTurretConfig().shooterKp;
    shooterConfig.Slot0.kI = bruinRobotConfig.getTurretConfig().shooterKi;
    shooterConfig.Slot0.kD = bruinRobotConfig.getTurretConfig().shooterKd;
    shooterConfig.Slot0.kS = bruinRobotConfig.getTurretConfig().shooterKs;
    shooterConfig.Slot0.kV = bruinRobotConfig.getTurretConfig().shooterKv;
    shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // change this later!
    shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 100;
    shooterConfig.MotionMagic.MotionMagicAcceleration = 50;

    shooterConfig.MotionMagic.MotionMagicExpo_kV = 8.0;

    Phoenix6Util.applyAndCheckConfiguration(shooterMotor, shooterConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(shooterMotor2, shooterConfig, 5);

    shooterMotor2.setControl(follower);

    shooterAppliedVoltage = shooterMotor.getMotorVoltage();
    shooterSupplyCurrent = shooterMotor.getSupplyCurrent();
    shooterStatorCurrent = shooterMotor.getStatorCurrent();
    shooterVelocityRotationsPerSec = shooterMotor.getVelocity();
    shooterAccelerationRotationsPerSecSquared = shooterMotor.getAcceleration();
    shooterMotorTemperature = shooterMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        shooterAppliedVoltage,
        shooterSupplyCurrent,
        shooterStatorCurrent,
        shooterVelocityRotationsPerSec,
        shooterAccelerationRotationsPerSecSquared,
        shooterMotorTemperature);

    inputs.shooterVoltage = shooterAppliedVoltage.getValueAsDouble();
    inputs.shooterSupplyCurrent = shooterSupplyCurrent.getValueAsDouble();
    inputs.shooterStatorCurrent = shooterStatorCurrent.getValueAsDouble();
    inputs.shooterTemperature = shooterMotorTemperature.getValueAsDouble();
    inputs.shooterVelocityRadPerSec =
        Units.rotationsToRadians(shooterVelocityRotationsPerSec.getValueAsDouble());
    inputs.shooterAccelRadPerSecSquared =
        Units.rotationsToRadians(shooterAccelerationRotationsPerSecSquared.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    shooterMotor.setVoltage(volts);
  }

  @Override
  public void setVelo(AngularVelocity velo) {
    // double velocitySetpoint = velo.magnitude() / 60;
    // shooterMotor.setControl
    // (motionMagicVelocityVoltage.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(true));
    // shooterMotor2.setControl
    // (motionMagicVelocityVoltage.withVelocity(velocitySetpoint).withSlot(0).withEnableFOC(true));

    shooterMotor.setControl(
        motionMagicVelocityVoltage.withVelocity(velo).withSlot(0).withEnableFOC(true));
  }
}
