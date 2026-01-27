package frc.robot.Subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class ClimberIOCTRE implements ClimberIO {
  private TalonFX leftMotor;
  private TalonFXConfiguration leftConfig;
  private MotionMagicVoltage leftMotionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftSupplyCurrentAmps;
  private final StatusSignal<Current> leftStatorCurrentAmps;
  private final StatusSignal<Temperature> leftMotorTemp;

  private TalonFX rightMotor;
  private TalonFXConfiguration rightConfig;
  private MotionMagicVoltage rightMotionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightSupplyCurrentAmps;
  private final StatusSignal<Current> rightStatorCurrentAmps;
  private final StatusSignal<Temperature> rightMotorTemp;

  private double climberHeightMeters;
  private double climberVelocityMetersPerSec;

  public ClimberIOCTRE(BruinRobotConfig bruinRobotConfig) {
    leftMotionMagicVoltage = new MotionMagicVoltage(0.0);
    leftMotor =
        new TalonFX(
            bruinRobotConfig.CLIMBER_LEFT_MOTOR.getDeviceNumber(),
            bruinRobotConfig.CLIMBER_LEFT_MOTOR.getBus());

    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leftConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    leftConfig.Slot0.kI = bruinRobotConfig.getClimberConfig().climberKi;
    leftConfig.Slot0.kP = bruinRobotConfig.getClimberConfig().climberKp;
    leftConfig.Slot0.kD = bruinRobotConfig.getClimberConfig().climberKd;
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
    leftConfig.MotionMagic.MotionMagicAcceleration = 0.3;

    Phoenix6Util.applyAndCheckConfiguration(leftMotor, leftConfig, 5);

    leftAppliedVolts = leftMotor.getMotorVoltage();
    leftSupplyCurrentAmps = leftMotor.getSupplyCurrent();
    leftStatorCurrentAmps = leftMotor.getStatorCurrent();
    leftMotorTemp = leftMotor.getDeviceTemp();

    rightMotionMagicVoltage = new MotionMagicVoltage(0.0);
    rightMotor =
        new TalonFX(
            bruinRobotConfig.CLIMBER_RIGHT_MOTOR.getDeviceNumber(),
            bruinRobotConfig.CLIMBER_RIGHT_MOTOR.getBus());

    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    rightConfig.CurrentLimits.StatorCurrentLimit = 90.0;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rightConfig.Slot0.kP = bruinRobotConfig.getClimberConfig().climberKp;
    rightConfig.Slot0.kI = bruinRobotConfig.getClimberConfig().climberKi;
    rightConfig.Slot0.kD = bruinRobotConfig.getClimberConfig().climberKd;

    rightConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
    rightConfig.MotionMagic.MotionMagicAcceleration = 0.3;

    Phoenix6Util.applyAndCheckConfiguration(rightMotor, rightConfig, 5);

    rightAppliedVolts = rightMotor.getMotorVoltage();
    rightSupplyCurrentAmps = rightMotor.getSupplyCurrent();
    rightStatorCurrentAmps = rightMotor.getStatorCurrent();
    rightMotorTemp = rightMotor.getDeviceTemp();

    // Not sure if this line needs to be in a seperate function
    // leftMotor.setControl(new Follower(rightMotor.getDeviceID(),true));
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leftAppliedVolts,
        leftSupplyCurrentAmps,
        leftStatorCurrentAmps,
        leftMotorTemp,
        rightAppliedVolts,
        rightSupplyCurrentAmps,
        rightStatorCurrentAmps,
        rightMotorTemp);
    inputs.climberLeftVoltage = leftAppliedVolts.getValueAsDouble();
    inputs.climberLeftSupplyCurrent = leftSupplyCurrentAmps.getValueAsDouble();
    inputs.climberLeftStatorCurrent = leftStatorCurrentAmps.getValueAsDouble();
    inputs.climberLeftTemperature = leftMotorTemp.getValueAsDouble();

    inputs.climberRightVoltage = rightAppliedVolts.getValueAsDouble();
    inputs.climberRightSupplyCurrent = rightSupplyCurrentAmps.getValueAsDouble();
    inputs.climberRightStatorCurrent = rightStatorCurrentAmps.getValueAsDouble();
    inputs.climberRightTemperature = rightMotorTemp.getValueAsDouble();

    inputs.climberVelocityMetersPerSec =
        climberVelocityMetersPerSec; // Not sure how to do these yet
    inputs.climberHeightMeters = climberHeightMeters;
  }

  public double heightToRotations(double height) {
    return height; // math for gear ratio and stuff
  }

  @Override
  public void setVoltage(double voltage) {
    rightMotor.setVoltage(voltage);
    leftMotor.setVoltage(voltage);
  }

  @Override
  public void setHeight(double height) {
    rightMotor.setControl(new MotionMagicVoltage(heightToRotations(height)));
    leftMotor.setControl(new MotionMagicVoltage(heightToRotations(height)));
  }
}
