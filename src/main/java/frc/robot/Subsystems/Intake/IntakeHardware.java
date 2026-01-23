package frc.robot.Subsystems.Intake;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeHardware implements IntakeIO{
    private TalonFX spinnerIntakeMotor;
    private TalonFX extensionIntakeMotor;
    private CANcoder spinnerIntakeEncoder;
    private CANcoder extensionIntakeEncoder;
    private TalonFXConfiguration intakeConfig;
    private MotionMagicConfigs motionMagicConfig;
    private CANcoderConfiguration encoderConfig;
    private MotionMagicVoltage intakeController;

    public IntakeHardware(int spinnerIntakeID, int spinnerEncoderID, int extensionIntakeID, int extensionEncoderID, String canBusString){
        spinnerIntakeMotor = new TalonFX(spinnerIntakeID, canBusString);
        spinnerIntakeEncoder = new CANcoder(spinnerEncoderID);

        extensionIntakeMotor = new TalonFX(extensionIntakeID, canBusString);
        extensionIntakeEncoder = new CANcoder(extensionEncoderID);
        
        intakeConfig = new TalonFXConfiguration();
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        intakeConfig.CurrentLimits.StatorCurrentLimit = 90.0;

        intakeConfig.Slot0.kP = 0.0;
        intakeConfig.Slot0.kI = 0.0;
        intakeConfig.Slot0.kD = 0.0;

        motionMagicConfig = intakeConfig.MotionMagic;
        motionMagicConfig.MotionMagicCruiseVelocity = 80; //RPS
        motionMagicConfig.MotionMagicAcceleration = 160; //160 rps/s (0.5 seconds)
        motionMagicConfig.MotionMagicJerk = 1600; //1600 rps/s/s (0.1 seconds)

        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        intakeConfig.MotionMagic.MotionMagicAcceleration = 0.0;
        intakeConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
        intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        spinnerIntakeMotor.getConfigurator().apply(intakeConfig);
        extensionIntakeMotor.getConfigurator().apply(intakeConfig);

        encoderConfig = new CANcoderConfiguration();
        //Config should do something
        spinnerIntakeEncoder.getConfigurator().apply(encoderConfig);
        extensionIntakeEncoder.getConfigurator().apply(encoderConfig);
    }

    //Rotations
    public void setSpinnerMotorPositionRots(double rots) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        spinnerIntakeMotor.setControl(m_request.withPosition(rots));
    }

    //Rotations
    public void setExtensionMotorPositionRots(double rots) {
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        extensionIntakeMotor.setControl(m_request.withPosition(rots));
    }

    public void setSpinnerMotorPositionRads(double rads) {
        setSpinnerMotorPositionRots(rads / (2*Math.PI));
    }

    public void setExtensionMotorPositionRads(double rads) {
        setExtensionMotorPositionRots(rads / (2*Math.PI));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {}

    @Override
    public void setSpinnerVoltage(double voltage) {
        spinnerIntakeMotor.setVoltage(voltage);
    }

    @Override
    public void setExtensionVoltage(double voltage) {
        extensionIntakeMotor.setVoltage(voltage);
    }
}
