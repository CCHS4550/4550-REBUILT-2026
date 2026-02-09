package frc.robot.Subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public enum WantedState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
    STOWED,
    IDLE
  }

  public enum SystemState {
    EXTENDED_INTAKING,
    EXTENDED_PASSIVE,
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
      case EXTENDED_INTAKING -> SystemState.EXTENDED_INTAKING;
      case EXTENDED_PASSIVE -> SystemState.EXTENDED_PASSIVE;

      default -> SystemState.IDLE;
    };
  }

  private void applyStates() {
    switch (systemState) {
      case IDLE:
        intakeIO.setSpinnerVoltage(0);
        intakeIO.setExtensionVoltage(0);
        break;
      case EXTENDED_INTAKING:
        intakeIO.setExtensionMotorPositionRad((Math.PI / 2));
        intakeIO.setSpinnerVoltage(3);
        break;
      case EXTENDED_PASSIVE:
        intakeIO.setExtensionMotorPositionRad((Math.PI) / 2);
        intakeIO.setSpinnerVoltage(0);
      case STOWED:
        intakeIO.setExtensionMotorPositionRad(0);
        intakeIO.setSpinnerVoltage(0);
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
