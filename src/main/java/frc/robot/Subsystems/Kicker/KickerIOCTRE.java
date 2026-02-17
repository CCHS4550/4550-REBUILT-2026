package frc.robot.Subsystems.Kicker;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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

public class KickerIOCTRE implements KickerIO {
  private TalonFX kickerMotor;

  private TalonFXConfiguration kickerConfig;

  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Angle> kickerPosRot;
  private final StatusSignal<Current> kickerSupplyCurrentAmps;
  private final StatusSignal<Current> kickerStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> kickerVelocityRotPerSec;
  private final StatusSignal<AngularAcceleration> kickerAccelerationRotPerSecSquared;
  private final StatusSignal<Temperature> kickerMotorTemp;

  public KickerIOCTRE(BruinRobotConfig robotConfig) {

    // Hopefully these kicker attributes gets created later in BruinRobotConfig.java
    // kickerMotor = new TalonFX(robotConfig.KICKER_MOTOR.getDeviceNumber(),
    // robotConfig.KICKER_MOTOR.getBus());
    kickerMotor =
        new TalonFX(robotConfig.KICKER_MOTOR.getDeviceNumber(), robotConfig.KICKER_MOTOR.getBus());
    // kickerEncoder = new CANcoder(robotConfig.KICKER_CANCODER.getDeviceNumber(),
    // robotConfig.KICKER_CANCODER.getBus());

    kickerConfig = new TalonFXConfiguration();

    kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kickerConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    kickerConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    kickerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kickerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kickerConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    kickerConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    // do config stuff
    Phoenix6Util.applyAndCheckConfiguration(kickerMotor, kickerConfig, 5);

    kickerMotor.setPosition(0);
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    kickerPosRot = kickerMotor.getPosition();
    kickerSupplyCurrentAmps = kickerMotor.getSupplyCurrent();
    kickerStatorCurrentAmps = kickerMotor.getStatorCurrent();
    kickerVelocityRotPerSec = kickerMotor.getVelocity();
    kickerAccelerationRotPerSecSquared = kickerMotor.getAcceleration();
    kickerMotorTemp = kickerMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        kickerAppliedVolts,
        kickerPosRot,
        kickerSupplyCurrentAmps,
        kickerStatorCurrentAmps,
        kickerVelocityRotPerSec,
        kickerAccelerationRotPerSecSquared,
        kickerMotorTemp);

    inputs.kickerVoltage = kickerAppliedVolts.getValueAsDouble();
    inputs.kickerPosRad =
        kickerPosRot.getValueAsDouble()
            * Constants.LowerChassisConstants.KICKER_POSITION_COEFFICIENT;
    inputs.kickerSupplyCurrent = kickerSupplyCurrentAmps.getValueAsDouble();
    inputs.kickerStatorCurrent = kickerStatorCurrentAmps.getValueAsDouble();
    inputs.kickerTemperature = kickerMotorTemp.getValueAsDouble();

    inputs.kickerVelocityRadPerSec =
        Units.rotationsToRadians(kickerVelocityRotPerSec.getValueAsDouble());
    inputs.kickerAccelRadPerSecSquared =
        Units.rotationsToRadians(kickerAccelerationRotPerSecSquared.getValueAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    kickerMotor.setVoltage(volts);
  }
}
