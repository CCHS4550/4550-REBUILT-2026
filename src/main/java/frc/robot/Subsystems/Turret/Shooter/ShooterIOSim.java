package frc.robot.Subsystems.Turret.Shooter;

import static edu.wpi.first.units.Units.Amp;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.ObjectInputFilter.Status;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Config.BruinRobotConfig;
import frc.robot.Util.Phoenix6Util;

public class ShooterIOSim implements ShooterIO {

  private TalonFX shooterMotor;
  private TalonFXSimState shooterMotorSim;
  private TalonFX shooterMotor2;
  private TalonFXSimState shooterMotor2Sim;

  private FlywheelSim shooterSimFlywheel;

  double appliedSpeed;

  private TalonFXConfiguration shooterConfig;
  private MotionMagicVelocityVoltage motionMagicVelocityVoltage;

  private final Voltage shooterAppliedVoltage;
  private final Current shooterSupplyCurrent;
  private final AngularVelocity shooterVelocityRotationsPerSec;


  public ShooterIOSim(BruinRobotConfig bruinRobotConfig) {
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
    
    shooterMotorSim = shooterMotor.getSimState();
    shooterMotor2Sim = shooterMotor2.getSimState();
    motionMagicVelocityVoltage = new MotionMagicVelocityVoltage(0.0).withSlot(0);
    appliedSpeed = 0.0;

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
    shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4550;
    shooterConfig.MotionMagic.MotionMagicAcceleration = 0.41;

    Phoenix6Util.applyAndCheckConfiguration(shooterMotor, shooterConfig, 5);
    Phoenix6Util.applyAndCheckConfiguration(shooterMotor2, shooterConfig, 5);

    shooterMotor2.setControl(follower);

    shooterAppliedVoltage = Voltage.ofBaseUnits(shooterMotorSim.getMotorVoltage(), Volts);
    shooterSupplyCurrent = Current.ofBaseUnits(shooterMotorSim.getSupplyCurrent(), Amp);
    // shooterStatorCurrent = Current.ofBaseUnits(shooterMotorSim.getStatorCurrent(), Amp);
    shooterVelocityRotationsPerSec = AngularVelocity.ofBaseUnits(appliedSpeed, RotationsPerSecond);
  
    // type of motor, gear ratio, then moment of inertia
    shooterSimFlywheel =
     new FlywheelSim(
        LinearSystemId.createFlywheelSystem(
            DCMotor.getKrakenX44Foc(2),
            0.003,   // moment of inertia (kg·m²)
            2.0      // gearing
        ),
        DCMotor.getKrakenX44Foc(2),
        2.0
    );
    

}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    

    inputs.shooterVoltage = shooterAppliedVoltage.magnitude();
    inputs.shooterSupplyCurrent = shooterSupplyCurrent.magnitude();
   
    inputs.shooterVelocityRadPerSec =
        Units.rotationsToRadians(shooterVelocityRotationsPerSec.magnitude());
   
  }

  @Override
  public void setVoltage(double volts) {
    shooterMotor.setVoltage(volts);
  }

  @Override
  public void setVelo(AngularVelocity velo) {
    shooterMotor.setControl(motionMagicVelocityVoltage.withVelocity(velo));
    double appliedSpeed = velo.magnitude();
  }
}
