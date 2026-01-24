package frc.robot.Subsystems.Kicker;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Turret.Elevation.ElevationIO.elevationIOInputs;

public interface KickerIO 
{
    @AutoLog
    public class KickerIOInputs
    {
        public double kickerVoltage = 0;
        public double kickerSupplyCurrent = 0.0;
        public double kickerStatorCurrent = 0.0;
        public double kickerTemperature = 0.0;

        public double kickerVelocityRadPerSec = 0.0;
        public double kickerAccelRadPerSecSquared = 0.0;


    }

    default void updateInputs(KickerIOInputs inputs) {}

    default void setVoltage(double voltage) {}
}
