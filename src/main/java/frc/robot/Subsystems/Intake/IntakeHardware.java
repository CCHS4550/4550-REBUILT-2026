package frc.robot.Subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeHardware implements IntakeIO {
  private TalonFX spinnerIntakeMotor;
  private TalonFX extensionIntakeMotor;
  private CANcoder spinnerIntakeEncoder;
  private CANcoder extensionIntakeEncoder;
  private TalonFXConfiguration intakeConfig;
  private MotionMagicConfigs motionMagicConfig;
  private CANcoderConfiguration encoderConfig;
  private MotionMagicVoltage intakeController;

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

  public IntakeHardware(
      int spinnerIntakeID,
      int spinnerEncoderID,
      int extensionIntakeID,
      int extensionEncoderID,
      String canBusString) {
    spinnerIntakeMotor = new TalonFX(spinnerIntakeID, canBusString);
    spinnerIntakeEncoder = new CANcoder(spinnerEncoderID);

    extensionIntakeMotor = new TalonFX(extensionIntakeID, canBusString);
    extensionIntakeEncoder = new CANcoder(extensionEncoderID);

    intakeConfig = new TalonFXConfiguration();
    intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    intakeConfig.CurrentLimits.SupplyCurrentLimit = 10.0;
    intakeConfig.CurrentLimits.StatorCurrentLimit = 10.0;

    intakeConfig.Slot0.kP = 0.0;
    intakeConfig.Slot0.kI = 0.0;
    intakeConfig.Slot0.kD = 0.0;

    motionMagicConfig = intakeConfig.MotionMagic;
    motionMagicConfig.MotionMagicCruiseVelocity = 80; // RPS
    motionMagicConfig.MotionMagicAcceleration = 160; // 160 rps/s (0.5 seconds)
    motionMagicConfig.MotionMagicJerk = 1600; // 1600 rps/s/s (0.1 seconds)

    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    intakeConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    intakeConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    spinnerIntakeMotor.getConfigurator().apply(intakeConfig);
    extensionIntakeMotor.getConfigurator().apply(intakeConfig);

    encoderConfig = new CANcoderConfiguration();
    // Config should do something
    spinnerIntakeEncoder.getConfigurator().apply(encoderConfig);
    extensionIntakeEncoder.getConfigurator().apply(encoderConfig);

    spinnerIntakeMotor.setPosition(0.0);
    spinnerAppliedVolts = spinnerIntakeMotor.getMotorVoltage();
    spinnerSupplyCurrentAmps = spinnerIntakeMotor.getSupplyCurrent();
    spinnerStatorCurrentAmps = spinnerIntakeMotor.getStatorCurrent();
    spinnerVelocityRotationsPerSec = spinnerIntakeMotor.getVelocity();
    spinnerAccelerationRotationsPerSecSquared = spinnerIntakeMotor.getAcceleration();
    spinnerMotorTemp = spinnerIntakeMotor.getDeviceTemp();

    extensionIntakeMotor.setPosition(0.0);
    extensionAppliedVolts = extensionIntakeMotor.getMotorVoltage();
    extensionSupplyCurrentAmps = extensionIntakeMotor.getSupplyCurrent();
    extensionStatorCurrentAmps = extensionIntakeMotor.getStatorCurrent();
    extensionVelocityRotationsPerSec = extensionIntakeMotor.getVelocity();
    extensionAccelerationRotationsPerSecSquared = extensionIntakeMotor.getAcceleration();
    extensionMotorTemp = extensionIntakeMotor.getDeviceTemp();

  }

  // Rotations
  public void setSpinnerMotorPositionRots(double rots) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    spinnerIntakeMotor.setControl(m_request.withPosition(rots));
  }

  // Rotations
  public void setExtensionMotorPositionRots(double rots) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    extensionIntakeMotor.setControl(m_request.withPosition(rots));
  }

  public void setSpinnerMotorPositionRads(double rads) {
    setSpinnerMotorPositionRots(rads / (2 * Math.PI));
  }

  public void setExtensionMotorPositionRads(double rads) {
    setExtensionMotorPositionRots(rads / (2 * Math.PI));
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
    inputs.extensionIntakeVelocityRadPerSec = extensionVelocityRotationsPerSec.getValueAsDouble() * 2 * Math.PI; //Update with constant
    inputs.extensionIntakeAccelRadPerSecSquared = extensionAccelerationRotationsPerSecSquared.getValueAsDouble() * 2 * Math.PI;
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
    inputs.spinnerIntakeVelocityRadPerSec = spinnerVelocityRotationsPerSec.getValueAsDouble() * 2 * Math.PI; //Update with constant
    inputs.spinnerIntakeAccelRadPerSecSquared = spinnerAccelerationRotationsPerSecSquared.getValueAsDouble() * 2 * Math.PI;
    inputs.spinnerIntakeTemperature = spinnerMotorTemp.getValueAsDouble();
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
