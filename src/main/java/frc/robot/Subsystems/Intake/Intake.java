package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public enum WantedState {
    INTAKING,
    MOVINGUP,
    MOVINGDOWN,
    IDLE
  }

  public enum SystemState {
    INTAKING,
    MOVINGUP,
    MOVINGDOWN,
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
      case MOVINGUP -> SystemState.MOVINGUP;
      case MOVINGDOWN -> SystemState.MOVINGDOWN;
      case INTAKING -> SystemState.INTAKING;

      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      default:
        // case IDLE:
        // break;
      case INTAKING:
        intakeIO.setSpinnerVoltage(10.0);
        break;
        // Make the intake (extension) rotate

      case MOVINGUP:
        intakeIO.setExtensionVoltage(10.0);
        break;
      case MOVINGDOWN:
        intakeIO.setExtensionVoltage(-10.0);
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
