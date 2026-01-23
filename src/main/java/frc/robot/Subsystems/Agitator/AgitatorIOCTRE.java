package frc.robot.Subsystems.Agitator;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class AgitatorIOCTRE implements AgitatorIO {
  private TalonFX agitatorMotor;
  private TalonFXConfiguration agitatorConfig;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final StatusSignal<Voltage> agitatorAppliedVolts;
  private final StatusSignal<Current> agitatorSupplyCurrentAmps;
  private final StatusSignal<Current> agitatorStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> agitatorVelocityPerSec;
  private final StatusSignal<AngularAcceleration> agitatorAccelerationPerSecSquared;
  private final StatusSignal<Temperature> agitatorMotorTemp;

  public AgitatorIOCTRE(int agitatorID, String canBusString) {
    agitatorMotor = new TalonFX(agitatorID, canBusString);
    agitatorConfig = new TalonFXConfiguration();

    agitatorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    agitatorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    agitatorConfig.CurrentLimits.SupplyCurrentLimit = 0.0;
    agitatorConfig.CurrentLimits.StatorCurrentLimit = 0.0;

    agitatorConfig.Slot0.kP = 0.0;
    agitatorConfig.Slot0.kI = 0.0;
    agitatorConfig.Slot0.kD = 0.0;

    agitatorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    agitatorConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    agitatorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
    agitatorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    agitatorMotor.getConfigurator().apply(agitatorConfig);
  }

    @Override
    public void updateInputs(AgitatorIOInputs inputs) {
        // TODO Auto-generated method stub

    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        agitatorMotor.setControl(PositionVoltage.ofVoltage(voltage));
    }
}
