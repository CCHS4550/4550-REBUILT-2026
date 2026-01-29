package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class IntakeIOCTRE implements IntakeIO {
  private TalonFX spinnerIntakeMotor;
  private TalonFX extensionIntakeMotor;

  private TalonFXConfiguration spinnerConfig;
  private TalonFXConfiguration extensionConfig;

  private TalonFXConfiguration intakeConfig;
  private MotionMagicConfigs motionMagicConfigSpinner;
  private MotionMagicConfigs motionMagicConfigExtension;
  private CANcoderConfiguration encoderConfig;
  private MotionMagicVelocityVoltage spinnerController;
  private MotionMagicVoltage extensionController;

  private final StatusSignal<Voltage> spinnerAppliedVolts;
  private final StatusSignal<Current> spinnerSupplyCurrentAmps;
  private final StatusSignal<Current> spinnerStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> spinnerVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> spinnerAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> spinnerMotorTemp;

  private final StatusSignal<Voltage> extensionAppliedVolts;
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

    // do we really need PID for a roller idk
    spinnerConfig.Slot0.kP = 0.0;
    spinnerConfig.Slot0.kI = 0.0;
    spinnerConfig.Slot0.kD = 0.0;

    spinnerConfig.MotionMagic.MotionMagicCruiseVelocity = 2000;
    spinnerConfig.MotionMagic.MotionMagicAcceleration = 100;
    motionMagicConfigSpinner = spinnerConfig.MotionMagic;

    spinnerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    spinnerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    extensionConfig = new TalonFXConfiguration();
    extensionConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    extensionConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    extensionConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    extensionConfig.MotionMagic.MotionMagicCruiseVelocity = 1000;
    extensionConfig.MotionMagic.MotionMagicAcceleration = 100;
    motionMagicConfigExtension = extensionConfig.MotionMagic;

    extensionConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    extensionConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(spinnerIntakeMotor, spinnerConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(extensionIntakeMotor, extensionConfig, 5);

    // spinnerIntakeMotor.setPosition(0.0);
    spinnerAppliedVolts = spinnerIntakeMotor.getMotorVoltage();
    spinnerSupplyCurrentAmps = spinnerIntakeMotor.getSupplyCurrent();
    spinnerStatorCurrentAmps = spinnerIntakeMotor.getStatorCurrent();
    spinnerVelocityRotationsPerSec = spinnerIntakeMotor.getVelocity();
    spinnerAccelerationRotationsPerSecSquared = spinnerIntakeMotor.getAcceleration();
    spinnerMotorTemp = spinnerIntakeMotor.getDeviceTemp();

    // extensionIntakeMotor.setPosition(0.0);
    extensionAppliedVolts = extensionIntakeMotor.getMotorVoltage();
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
        extensionSupplyCurrentAmps,
        extensionStatorCurrentAmps,
        extensionVelocityRotationsPerSec,
        extensionAccelerationRotationsPerSecSquared,
        extensionMotorTemp);

    inputs.extensionIntakeVoltage = extensionAppliedVolts.getValueAsDouble();
    inputs.extensionIntakeSupplyCurrent = extensionSupplyCurrentAmps.getValueAsDouble();
    inputs.extensionIntakeStatorCurrent = extensionStatorCurrentAmps.getValueAsDouble();
    inputs.extensionIntakeVelocityRadPerSec =
        Units.rotationsToRadians(
            extensionVelocityRotationsPerSec.getValueAsDouble()); // Update with constant
    inputs.extensionIntakeAccelRadPerSecSquared =
        Units.rotationsToRadians(extensionAccelerationRotationsPerSecSquared.getValueAsDouble());
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
        Units.rotationsToRadians(spinnerVelocityRotationsPerSec.getValueAsDouble()); // Update with constant
    inputs.spinnerIntakeAccelRadPerSecSquared =
        Units.rotationsToRadians(spinnerAccelerationRotationsPerSecSquared.getValueAsDouble());
    inputs.spinnerIntakeTemperature = spinnerMotorTemp.getValueAsDouble();
  }

  // Rotations
  public void setExtensionMotorPositionRad(double rad) {

    extensionIntakeMotor.setControl(extensionConfig.withPosition(Units.radiansToRotations(rad)));
  }

  @Override
  public void setSpinnerSpeed(double speedRadPerSec) {
    spinnerIntakeMotor.setControl(spinnerController.withVelocity(speedRadPerSec));
  }

  @Override
  public void setSpinnerVoltage(double voltage) {
    spinnerIntakeMotor.setVoltage(voltage);
  }

  @Override
  public void setExtensionVoltage(double voltage) {
    extensionIntakeMotor.setVoltage(voltage);
  }
}
