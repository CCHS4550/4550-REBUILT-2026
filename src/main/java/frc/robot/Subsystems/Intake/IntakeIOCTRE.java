package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Constant.Constants;
import frc.robot.Util.Phoenix6Util;

public class IntakeIOCTRE implements IntakeIO {
  private TalonFX spinnerIntakeMotor;
  private TalonFX extensionIntakeMotor;

  private TalonFXConfiguration spinnerConfig;
  private TalonFXConfiguration extensionConfig;

  private MotionMagicVoltage extensionController = new MotionMagicVoltage(0).withSlot(0);

  private final StatusSignal<Voltage> spinnerAppliedVolts;
  private final StatusSignal<Current> spinnerSupplyCurrentAmps;
  private final StatusSignal<Current> spinnerStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> spinnerVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> spinnerAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> spinnerMotorTemp;

  private final StatusSignal<Voltage> extensionAppliedVolts;
  private final StatusSignal<Angle> extensionPosRot;
  private final StatusSignal<Current> extensionSupplyCurrentAmps;
  private final StatusSignal<Current> extensionStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> extensionVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> extensionAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> extensionMotorTemp;

  public IntakeIOCTRE(BruinRobotConfig robotConfig) {
    spinnerIntakeMotor =
        new TalonFX(
            robotConfig.INTAKE_ROLLER.getDeviceNumber(), robotConfig.INTAKE_ROLLER.getBus());

    extensionIntakeMotor =
        new TalonFX(
            robotConfig.INTAKE_EXTENSION.getDeviceNumber(), robotConfig.INTAKE_EXTENSION.getBus());

    spinnerConfig = new TalonFXConfiguration();
    spinnerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    spinnerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spinnerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    spinnerConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    spinnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    spinnerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    extensionConfig = new TalonFXConfiguration();
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    extensionConfig.Slot0.kP = robotConfig.getIntakeConfig().extensionkP;
    extensionConfig.Slot0.kI = robotConfig.getIntakeConfig().extensionkI;
    extensionConfig.Slot0.kD = robotConfig.getIntakeConfig().extensionkD;
    extensionConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    extensionConfig.Slot0.kS = robotConfig.getIntakeConfig().extensionkS;
    extensionConfig.Slot0.kV = robotConfig.getIntakeConfig().extensionkS;
    extensionConfig.Slot0.kG = robotConfig.getIntakeConfig().extensionkG;
    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 10000;
    extensionConfig.MotionMagic.MotionMagicAcceleration = 1000;

    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(spinnerIntakeMotor, spinnerConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(extensionIntakeMotor, extensionConfig, 5);

    spinnerAppliedVolts = spinnerIntakeMotor.getMotorVoltage();
    spinnerSupplyCurrentAmps = spinnerIntakeMotor.getSupplyCurrent();
    spinnerStatorCurrentAmps = spinnerIntakeMotor.getStatorCurrent();
    spinnerVelocityRotationsPerSec = spinnerIntakeMotor.getVelocity();
    spinnerAccelerationRotationsPerSecSquared = spinnerIntakeMotor.getAcceleration();
    spinnerMotorTemp = spinnerIntakeMotor.getDeviceTemp();

    extensionIntakeMotor.setPosition(6.25);
    extensionAppliedVolts = extensionIntakeMotor.getMotorVoltage();
    extensionPosRot = extensionIntakeMotor.getPosition();
    extensionSupplyCurrentAmps = extensionIntakeMotor.getSupplyCurrent();
    extensionStatorCurrentAmps = extensionIntakeMotor.getStatorCurrent();
    extensionVelocityRotationsPerSec = extensionIntakeMotor.getVelocity();
    extensionAccelerationRotationsPerSecSquared = extensionIntakeMotor.getAcceleration();
    extensionMotorTemp = extensionIntakeMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        extensionAppliedVolts,
        extensionPosRot,
        extensionSupplyCurrentAmps,
        extensionStatorCurrentAmps,
        extensionVelocityRotationsPerSec,
        extensionAccelerationRotationsPerSecSquared,
        extensionMotorTemp);

    inputs.extensionIntakeVoltage = extensionAppliedVolts.getValueAsDouble();
    inputs.extensionPosRadians =
        extensionPosRot.getValueAsDouble()
            * Constants.IntakeConstants.EXTENSION_POSITION_COEFFICIENT;
    inputs.extensionIntakeSupplyCurrent = extensionSupplyCurrentAmps.getValueAsDouble();
    inputs.extensionIntakeStatorCurrent = extensionStatorCurrentAmps.getValueAsDouble();
    inputs.extensionIntakeVelocityRadPerSec =
        extensionVelocityRotationsPerSec.getValueAsDouble()
            * Constants.IntakeConstants.EXTENSION_POSITION_COEFFICIENT; // Update with constant
    inputs.extensionIntakeAccelRadPerSecSquared =
        extensionVelocityRotationsPerSec.getValueAsDouble()
            * Constants.IntakeConstants.EXTENSION_POSITION_COEFFICIENT;
    inputs.extensionIntakeTemperature = extensionMotorTemp.getValueAsDouble();

    BaseStatusSignal.refreshAll(
        spinnerAppliedVolts,
        spinnerSupplyCurrentAmps,
        spinnerStatorCurrentAmps,
        spinnerVelocityRotationsPerSec,
        spinnerAccelerationRotationsPerSecSquared,
        spinnerMotorTemp);

    inputs.spinnerIntakeVoltage = spinnerAppliedVolts.getValueAsDouble();
    inputs.spinnerIntakeSupplyCurrent = spinnerSupplyCurrentAmps.getValueAsDouble();
    inputs.spinnerIntakeStatorCurrent = spinnerStatorCurrentAmps.getValueAsDouble();
    inputs.spinnerIntakeVelocityRadPerSec =
        Units.rotationsToRadians(
            spinnerVelocityRotationsPerSec.getValueAsDouble()); // Update with constant
    inputs.spinnerIntakeAccelRadPerSecSquared =
        Units.rotationsToRadians(spinnerAccelerationRotationsPerSecSquared.getValueAsDouble());
    inputs.spinnerIntakeTemperature = spinnerMotorTemp.getValueAsDouble();
  }

  @Override
  public void setExtensionMotorPositionRad(double rad) {
    extensionIntakeMotor.setControl(
        extensionController.withPosition(Units.radiansToRotations(rad)));
  }

  @Override
  public void setExtensionVoltage(double voltage) {
    extensionIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void setSpinnerVoltage(double voltage) {
    spinnerIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void setExtensionNeutralMode(NeutralModeValue neutralMode) {
    extensionIntakeMotor.setNeutralMode(neutralMode);
  }
}
