package frc.robot.Subsystems.Intake;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeHardware implements IntakeIO{
    private TalonFX leftIntakeMotor;
    private TalonFX rightIntakeMotor;
    private CANcoder leftIntakeEncoder;
    private CANcoder rightIntakeEncoder;
    private TalonFXConfiguration intakeConfig;
    private CANcoderConfiguration encoderConfig;
    private MotionMagicVelocityVoltage intakeController;

    public IntakeHardware(int leftIntakeID, int leftEncoderID, int rightIntakeID, int rightEncoderID, String canBusString){
        leftIntakeMotor = new TalonFX(leftIntakeID, canBusString);
        leftIntakeEncoder = new CANcoder(leftEncoderID);

        rightIntakeMotor = new TalonFX(rightIntakeID, canBusString);
        rightIntakeEncoder = new CANcoder(rightEncoderID);
        
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

        leftIntakeMotor.getConfigurator().apply(intakeConfig);
        rightIntakeMotor.getConfigurator().apply(intakeConfig);

        encoderConfig = new CANcoderConfiguration();
        //Config should do something
        leftIntakeEncoder.getConfigurator().apply(encoderConfig);
        rightIntakeEncoder.getConfigurator().apply(encoderConfig);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void setVoltage(double voltage) {}
}
