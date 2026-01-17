package frc.robot.Subsystems.Turret.Hood;
import java.io.ObjectInputFilter.Status;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;


public class HoodIOCTRE implements HoodIO{
    
    private TalonFX hoodMotor;
    private CANcoder hoodEncoder;
    private TalonFXConfiguration hoodConfig;
    private MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0).withSlot(0);


    public HoodIOCTRE (int hoodID, int canCoderId, String canBusString){
        hoodMotor = new TalonFX(hoodID, canBusString); // creates motor
        hoodEncoder = new CANcoder(canCoderId); // creates CANCoder, which should be connected to the motor electrically

        hoodConfig = new TalonFXConfiguration();
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        hoodConfig.CurrentLimits.StatorCurrentLimit = 90.0;

        hoodConfig.Slot0.kP = 
        hoodConfig.MotionMagic.MotionMagicAcceleration = 0.3; // some constant idk


        hoodMotor.getConfigurator().apply(hoodConfig);

        
       


    }

    @Override
    public void updateInputs (HoodIOInputs inputs){
        inputs.hoodVelocityRotPerSec = getMotorVelocity().getValueAsDouble();
        inputs.hoodVelocityRadPerSec = Units.rotationsToRadians(getMotorVelocity().getValueAsDouble());


    }


   
    public StatusSignal<Voltage> getMotorVoltage (){
        return hoodMotor.getMotorVoltage();
    }

    public StatusSignal<AngularVelocity> getMotorVelocity (){
        return hoodMotor.getVelocity();
    }

    @Override // masgic shit
    public void setHoodAngle (Rotation2d angle){
        hoodMotor.setControl(motionMagicVoltage.withPosition(angle.getRadians()));
    }


    



    

}
