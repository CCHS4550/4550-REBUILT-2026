package frc.robot.Subsystems.Kicker;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class KickerIOCTRE implements KickerIO {
  private TalonFX kickerMotor;
  private CANcoder kickerEncoder;
  private TalonFXConfiguration kickerConfig;
  private CANcoderConfiguration encoderConfig;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final StatusSignal<Angle> kickerAngleRotations;
  private final StatusSignal<Voltage> kickerAppliedVolts;
  private final StatusSignal<Current> kickerSupplyCurrentAmps;
  private final StatusSignal<Current> kickerStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> kickerVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> kickerAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> kickerMotorTemp;

  public KickerIOCTRE(BruinRobotConfig robotConfig) {

    // Hopefully these kicker attributes gets created later in BruinRobotConfig.java
    // kickerMotor = new TalonFX(robotConfig.KICKER_MOTOR.getDeviceNumber(),
    // robotConfig.KICKER_MOTOR.getBus());
    kickerMotor = new TalonFX(-1, "");
    // kickerEncoder = new CANcoder(robotConfig.KICKER_CANCODER.getDeviceNumber(),
    // robotConfig.KICKER_CANCODER.getBus());
    kickerEncoder = new CANcoder(-1, "");

    kickerConfig = new TalonFXConfiguration();
    // do config stuff
    Phoenix6Util.applyAndCheckConfiguration(kickerMotor, kickerConfig);

    encoderConfig = new CANcoderConfiguration();
    // do more config stuff
    kickerEncoder.getConfigurator().apply(encoderConfig);

    kickerAngleRotations = kickerEncoder.getAbsolutePosition();
    kickerAppliedVolts = kickerMotor.getMotorVoltage();
    kickerSupplyCurrentAmps = kickerMotor.getSupplyCurrent();
    kickerStatorCurrentAmps = kickerMotor.getStatorCurrent();
    kickerVelocityRotationsPerSec = kickerEncoder.getVelocity();
    kickerAccelerationRotationsPerSecSquared = kickerMotor.getAcceleration();
    kickerMotorTemp = kickerMotor.getDeviceTemp();
  }

  @Override
  public void updateInputs(KickerIOInputs inputs) {
    // too lazy to do this rn
  }

  @Override
  public void setVoltage(double volts) {
    kickerMotor.setVoltage(volts);
  }
}
