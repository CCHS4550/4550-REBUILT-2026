package frc.robot.Subsystems.Turret.Elevation;

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



public class ElevationIOCTRE implements ElevationIO {
  private TalonFX elevationMotor;
  private CANcoder elevationEncoder;
  private TalonFXConfiguration elevationConfig;
  private CANcoderConfiguration encoderConfig;
  private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
  private final StatusSignal<Angle> elevationAngleRotations;
  private final StatusSignal<Voltage> elevationAppliedVolts;
  private final StatusSignal<Current> elevationSupplyCurrentAmps;
  private final StatusSignal<Current> elevationStatorCurrentAmps;
  private final StatusSignal<AngularVelocity> elevationVelocityRotationsPerSec;
  private final StatusSignal<AngularAcceleration> elevationAccelerationRotationsPerSecSquared;
  private final StatusSignal<Temperature> elevationMotorTemp;

  public ElevationIOCTRE(BruinRobotConfig bruinRobotConfig) {
    motionMagicVoltage = new MotionMagicVoltage(0.0);
    elevationMotor = new TalonFX(bruinRobotConfig.ELEVATION_MOTOR.getDeviceNumber(), bruinRobotConfig.ELEVATION_MOTOR.getBus()); // creates motor
    elevationEncoder =
        new CANcoder(
            bruinRobotConfig.ELEVATION_CANCODER.getDeviceNumber(), bruinRobotConfig.ELEVATION_CANCODER.getBus()); // creates CANCoder, which should be connected to the motor electrically

    // I should probably set up these constants in like RobotConfig, but I just want to try and
    // complete this out
    elevationConfig = new TalonFXConfiguration();
    elevationConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    elevationConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    elevationConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    elevationConfig.CurrentLimits.StatorCurrentLimit = 90.0;

    elevationConfig.Slot0.kP = bruinRobotConfig.getTurretConfig().elevationKp;
    elevationConfig.Slot0.kI = bruinRobotConfig.getTurretConfig().elevationKi;
    elevationConfig.Slot0.kD = bruinRobotConfig.getTurretConfig().elevationKd;
    elevationConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elevationConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    elevationConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
    elevationConfig.MotionMagic.MotionMagicAcceleration = 0.3; // some constant idk

    Phoenix6Util.applyAndCheckConfiguration(elevationMotor, elevationConfig, 5);

    encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(0.82).withMagnetOffset(0.0).withSensorDirection(SensorDirectionValue.Clockwise_Positive);
    elevationEncoder.getConfigurator().apply(encoderConfig); 

    elevationAngleRotations = elevationEncoder.getAbsolutePosition();
    elevationAppliedVolts = elevationMotor.getMotorVoltage();
    elevationSupplyCurrentAmps = elevationMotor.getSupplyCurrent();
    elevationStatorCurrentAmps = elevationMotor.getStatorCurrent();
    elevationVelocityRotationsPerSec = elevationEncoder.getVelocity();
    elevationAccelerationRotationsPerSecSquared = elevationMotor.getAcceleration();
    elevationMotorTemp = elevationMotor.getDeviceTemp();

  }

  @Override
  public void updateInputs(elevationIOInputs inputs) {
    BaseStatusSignal.refreshAll(elevationAppliedVolts, elevationSupplyCurrentAmps, elevationStatorCurrentAmps, elevationAccelerationRotationsPerSecSquared, elevationMotorTemp);
    BaseStatusSignal.refreshAll(elevationAngleRotations, elevationVelocityRotationsPerSec);
    inputs.elevationVoltage = elevationAppliedVolts.getValueAsDouble();
    inputs.elevationSupplyCurrent = elevationSupplyCurrentAmps.getValueAsDouble();
    inputs.elevationStatorCurrent = elevationStatorCurrentAmps.getValueAsDouble();
    inputs.elevationTemperature = elevationMotorTemp.getValueAsDouble();

    inputs.elevationVelocityRadPerSec = elevationVelocityRotationsPerSec.getValueAsDouble() * Constants.TurretConstants.ELEVATION_POSITION_COEFFICIENT ;
    inputs.elevationAccelRadPerSecSquared = elevationAccelerationRotationsPerSecSquared.getValueAsDouble() * Constants.TurretConstants.ELEVATION_POSITION_COEFFICIENT ;

    inputs.elevationAngle = Rotation2d.fromRotations(elevationAngleRotations.getValueAsDouble());


  }

  @Override
  public void setElevationAngle(Rotation2d angle) {
    elevationMotor.setControl(motionMagicVoltage.withPosition(angle.getRadians() / Constants.TurretConstants.ELEVATION_POSITION_COEFFICIENT));
  }

  @Override
  public void setVoltage(double volts){
    elevationMotor.setVoltage(volts);
  }
}
