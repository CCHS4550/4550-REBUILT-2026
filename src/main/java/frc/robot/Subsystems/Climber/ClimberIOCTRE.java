package frc.robot.Subsystems.Climber;
public class ClimberIOCTRE implements ClimberIO{
    private TalonFX leftMotor;
    private CANCoder leftEncoder;
    private TalonFXConfiguration leftConfig;
    private MotionMagicVoltage leftMotionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final StatusSignal<Voltage> leftAppliedVolts;
    private final StatusSignal<Current> leftSupplyCurrentAmps;
    private final StatusSignal<Current> leftStatorCurrentAmps;
    private final StatusSignal<Temperature> leftMotorTemp;

    private TalonFX rightMotor;
    private CANCoder rightEncoder;
    private TalonFXConfiguration rightConfig;
    private MotionMagicVoltage rightMotionMagicVoltage = new MotionMagicVoltage(0).withSlot(0);
    private final StatusSignal<Voltage> rightAppliedVolts;
    private final StatusSignal<Current> rightSupplyCurrentAmps;
    private final StatusSignal<Current> rightStatorCurrentAmps;
    private final StatusSignal<Temperature> rightMotorTemp;

    private CANcoderConfiguration encoderConfig;

    public ClimberIOCTRE(BruinRobotConfig bruinRobotConfig){
        leftMotionMagicVoltage = new MotionMagicVoltage(0.0);
        leftMotor =
            new TalonFX(
                bruinRobotConfig.CLIMBER_LEFT_MOTOR.getDeviceNumber(),
                bruinRobotConfig.CLIMBER_LEFT_MOTOR.getBus());
        leftEncoder =
            new CANcoder(
                bruinRobotConfig.CLIMBER_LEFT_CANCODER.getDeviceNumber(),
                bruinRobotConfig.CLIMBER_LEFT_CANCODER.getBus());

        leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        leftConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        leftConfig.CurrentLimits.StatorCurrentLimit = 90.0;

        leftConfig.Slot0.kI = bruinRobotConfig.getClimberConfig().climberKi;
        leftConfig.Slot0.kP = bruinRobotConfig.getClimberConfig().climberKp;
        leftConfig.Slot0.kD = bruinRobotConfig.getClimberConfig().climberKd;
        leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        leftConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
        leftConfig.MotionMagic.MotionMagicAcceleration = 0.3;

        Phoenix6Util.applyAndCheckConfiguration(leftMotor, leftConfig, 5);

        encoderConfig = new CANcoderConfiguration();
        encoderConfig
            .MagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(0.82)
            .withMagnetOffset(0.0)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        leftEncoder.getConfigurator().apply(encoderConfig);

        leftAppliedVolts = leftMotor.getMotorVoltage();
        leftSupplyCurrentAmps = leftMotor.getSupplyCurrent();
        leftStatorCurrentAmps = leftMotor.getStatorCurrent();
        leftMotorTemp = leftMotor.getDeviceTemp();



        rightMotionMagicVoltage = new MotionMagicVoltage(0.0);
        rightMotor =
            new TalonFX(
                bruinRobotConfig.CLIMBER_RIGHT_MOTOR.getDeviceNumber(),
                bruinRobotConfig.CLIMBER_RIGHT_MOTOR.getBus());
        rightEncoder =
            new CANcoder(
                bruinRobotConfig.CLIMBER_RIGHT_CANCODER.getDeviceNumber(),
                bruinRobotConfig.CLIMBER_RIGHT_CANCODER.getBus());

        rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rightConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
        rightConfig.CurrentLimits.StatorCurrentLimit = 90.0;

        rightConfig.Slot0.kI = bruinRobotConfig.getClimberConfig().climberKi;
        rightConfig.Slot0.kP = bruinRobotConfig.getClimberConfig().climberKp;
        rightConfig.Slot0.kD = bruinRobotConfig.getClimberConfig().climberKd;
        rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
        rightConfig.MotionMagic.MotionMagicAcceleration = 0.3;

        Phoenix6Util.applyAndCheckConfiguration(rightMotor, rightConfig, 5);

        encoderConfig = new CANcoderConfiguration();
        encoderConfig
            .MagnetSensor
            .withAbsoluteSensorDiscontinuityPoint(0.82)
            .withMagnetOffset(0.0)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        rightEncoder.getConfigurator().apply(encoderConfig);

        rightAppliedVolts = rightMotor.getMotorVoltage();
        rightSupplyCurrentAmps = rightMotor.getSupplyCurrent();
        rightStatorCurrentAmps = rightMotor.getStatorCurrent();
        rightMotorTemp = rightMotor.getDeviceTemp();
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            leftAppliedVolts,
            leftSupplyCurrentAmps,
            leftStatorCurrentAmps,
            leftMotorTemp,  

            rightAppliedVolts,
            rightSupplyCurrentAmps,
            rightStatorCurrentAmps,
            rightMotorTemp);
        inputs.climberLeftVoltage = leftAppliedVolts.getValueAsDouble();
        inputs.climberLeftSupplyCurrent = leftSupplyCurrentAmps.getValueAsDouble();
        inputs.climberLeftStatorCurrent = leftStatorCurrentAmps.getValueAsDouble();
        inputs.climberLeftTemperature = leftMotorTemp.getValueAsDouble();
    
        inputs.climberRightVoltage = rightAppliedVolts.getValueAsDouble();
        inputs.climberRightSupplyCurrent = rightSupplyCurrentAmps.getValueAsDouble();
        inputs.climberRightStatorCurrent = rightStatorCurrentAmps.getValueAsDouble();
        inputs.climberRightTemperature = rightMotorTemp.getValueAsDouble();

        inputs.climberVelocityMetersPerSec = 0.0; //Not sure how to do these yet
        inputs.climberHeightMeters = 0.0;
    }

    @Override
    public void setVoltage(double voltage){
        leftMotor.setVoltage(voltage);
        rightMotor.setVoltage(voltage);
    }

    @Override
    public void setHeight(double height){
        //Don't know this one either
    }
}