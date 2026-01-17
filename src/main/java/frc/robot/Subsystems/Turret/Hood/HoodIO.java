package frc.robot.Subsystems.Turret.Hood;
import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;


public interface HoodIO {
    
    @AutoLog
    public class HoodIOInputs {
        public double hoodVoltage = 0.0;
        public double hoodSupplyCurrent = 0.0; 
        public double hoodStatorCurrent = 0.0;
        public double hoodTemperature = 0.0;

        public double hoodVelocityRotPerSec = 0.0;
        public double hoodAccelRotPerSecSquared = 0.0;

        public double hoodVelocityRadPerSec = 0.0;
        public double hoodAccelRadPerSecSquared = 0.0;

        public Rotation2d hoodAngle = Rotation2d.kZero;
    }

    default void updateInputs (HoodIOInputs inputs){}

    default void setHoodAngle (Rotation2d angle) {}

    default void setVoltage (double voltage){}



}
