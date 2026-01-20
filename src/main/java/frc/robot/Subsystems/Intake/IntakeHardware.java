package frc.robot.Subsystems.Intake;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeHardware implements IntakeIO{
    private TalonFX horizontalIntakeMotor;
    private TalonFX verticalintakeMotor;
    private CANcoder horizontalIntakeEncoder;
    private CANcoder verticalintakeEncoder;
    private TalonFXConfiguration intakeConfig;
    private CANcoderConfiguration encoderConfig;
    private MotionMagicVelocityVoltage intakeController;

    public IntakeHardware(int horizontalIntakeID, int horizontalEncoderID, int verticalintakeID, int verticalEncoderID, String canBusString){
        horizontalIntakeMotor = new TalonFX(horizontalIntakeID, canBusString);
        horizontalIntakeEncoder = new CANcoder(horizontalEncoderID);

        verticalintakeMotor = new TalonFX(verticalintakeID, canBusString);
        verticalintakeEncoder = new CANcoder(verticalEncoderID);
        
        intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 90.0;

        intakeConfig.Slot0.kP = 0.0;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.MotionMagic.MotionMagicAcceleration = 0.0;
        intakeConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        horizontalIntakeMotor.getConfigurator().apply(intakeConfig);
        verticalintakeMotor.getConfigurator().apply(intakeConfig);

        encoderConfig = new CANcoderConfiguration();
        //Config should do something
        horizontalIntakeEncoder.getConfigurator().apply(encoderConfig);
        verticalintakeEncoder.getConfigurator().apply(encoderConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void setVoltage(double voltage) {}
}
