package frc.robot.Subsystems.Intake;
import org.littletonrobotics.junction.AutoLog;
public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        //Intake1
        public double leftIntakeVoltage = 0.0;
        public double leftIntakeSupplyCurrent = 0.0;
        public double leftIntakeStatorCurrent = 0.0;
        public double leftIntakeTemperature = 0.0;

        public double leftIntakeVelocityRadPerSec = 0.0;
        public double leftIntakeAccelRadPerSecSquared = 0.0;

        //Intake2
        public double rightIntakeVoltage = 0.0;
        public double rightIntakeSupplyCurrent = 0.0;
        public double rightIntakeStatorCurrent = 0.0;
        public double rightIntakeTemperature = 0.0;

        public double rightIntakeVelocityRadPerSec = 0.0;
        public double rightIntakeAccelRadPerSecSquared = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setRadPerSec(double radPerSec) {}

    public default void setRotPerSec(double rotPerSec) {}

    public default void setVoltage(double voltage) {}

    public default void setSpeed(double speed) {}
}



