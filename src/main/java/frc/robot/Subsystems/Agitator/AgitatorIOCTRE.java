package frc.robot.Subsystems.Agitator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class AgitatorIOCTRE implements AgitatorIO {
  private TalonFX agitatorMotor;
  private TalonFXConfiguration agitatorConfig;
  private final StatusSignal<Voltage> agitatorAppliedVolts;
  private final StatusSignal<Current> agitatorSupplyCurrentAmps;
  private final StatusSignal<Current> agitatorStatorCurrentAmps;
  private final StatusSignal<Angle> agitatorPositionRotations;
  private final StatusSignal<AngularVelocity> agitatorVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> agitatorAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> agitatorMotorTemp;

  public AgitatorIOCTRE(BruinRobotConfig bruinRobotConfig) {
    agitatorMotor =
        new TalonFX(
            bruinRobotConfig.AGITATOR_MOTOR.getDeviceNumber(),
            bruinRobotConfig.AGITATOR_MOTOR.getBus());
    agitatorConfig = new TalonFXConfiguration();

    agitatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    agitatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    agitatorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    agitatorConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    agitatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    agitatorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    Phoenix6Util.applyAndCheckConfiguration(agitatorMotor, agitatorConfig, 5);

    agitatorMotor.setPosition(0.0);
    agitatorPositionRotations = agitatorMotor.getPosition();
    agitatorAppliedVolts = agitatorMotor.getMotorVoltage();
    agitatorSupplyCurrentAmps = agitatorMotor.getSupplyCurrent();
    agitatorStatorCurrentAmps = agitatorMotor.getStatorCurrent();
    agitatorVelocityRotationsPerSec = agitatorMotor.getVelocity();
    agitatorAccelerationRotationsPerSecSquared = agitatorMotor.getAcceleration();
    agitatorMotorTemp = agitatorMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(AgitatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        agitatorPositionRotations,
        agitatorAppliedVolts,
        agitatorSupplyCurrentAmps,
        agitatorStatorCurrentAmps,
        agitatorVelocityRotationsPerSec,
        agitatorAccelerationRotationsPerSecSquared,
        agitatorMotorTemp);

    inputs.agitatorPositionRadians =
        agitatorPositionRotations.getValueAsDouble()
            * 2
            * Math.PI; // TODO: Update this later with a total gear reduction in constants
    inputs.agitatorVoltage = agitatorAppliedVolts.getValueAsDouble();
    inputs.agitatorSupplyCurrent = agitatorSupplyCurrentAmps.getValueAsDouble();
    inputs.agitatorStatorCurrent = agitatorStatorCurrentAmps.getValueAsDouble();
    inputs.rotationVelocityRadPerSec =
        agitatorVelocityRotationsPerSec.getValueAsDouble()
            * 2
            * Math.PI; // TODO: Update this later with a total gear reduction in constants
    inputs.rotationAccelRadPerSecSquared =
        agitatorAccelerationRotationsPerSecSquared.getValueAsDouble()
            * 2
            * Math.PI; // TODO: Update this later with a total gear reduction in constants
    inputs.temperature = agitatorMotorTemp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    agitatorMotor.setVoltage(voltage);
  }
}
