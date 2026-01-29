package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public enum WantedState {
    INTAKING,
    STOWED,
    IDLE
  }

  public enum SystemState {
    INTAKING,
    STOWED,
    IDLE
  }

  private SystemState systemState = SystemState.IDLE;
  private WantedState wantedState = WantedState.IDLE;

  private final IntakeIO intakeIO;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  private SystemState handleStateTransition() {
    return switch (wantedState) {
      case IDLE -> SystemState.IDLE;
      case STOWED -> SystemState.STOWED;
      case INTAKING -> SystemState.INTAKING;

      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLE:
        intakeIO.setSpinnerVoltage(0);
        intakeIO.setExtensionVoltage(0);
        break;

      case INTAKING:
        intakeIO.setExtensionMotorPositionRad((Math.PI / 2));
        intakeIO.setSpinnerSpeed(1500);
        break;
        // Make the intake (extension) rotate

      case STOWED:
        intakeIO.setExtensionMotorPositionRad(0);
        intakeIO.setSpinnerSpeed(0);
        break;

      default:
        intakeIO.setSpinnerVoltage(0);
        intakeIO.setExtensionVoltage(0);
        break;
    }
  }

  public void setState(WantedState state) {
    this.wantedState = state;
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    systemState = handleStateTransition();

    applyStates();
  }
}
