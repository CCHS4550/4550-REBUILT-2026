package frc.robot.Subsystems.Intake;

import edu.wpi.first.math.MathUtil;
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
    MOVING_TO_EXTENSION,
    STOWED,
    MOVING_TO_STOW,
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
    switch(wantedState){
      case EXTENDED_INTAKING : {
        if(atWantedAngle()){
          return SystemState.EXTENDED_INTAKING;
        }
        else{
          return SystemState.MOVING_TO_EXTENSION;
        }
      }
      case EXTENDED_PASSIVE: {
        if(atWantedAngle()){
          return SystemState.EXTENDED_PASSIVE;
        }
        else{
          return SystemState.MOVING_TO_EXTENSION;
        }
      }
      case STOWED : {
         if(atWantedAngle()){
          return SystemState.STOWED;
        }
        else{
          return SystemState.MOVING_TO_STOW;
        }
      }
      case IDLE : return SystemState.IDLE;
      default: return SystemState.IDLE;
    }
  }

  private void applyStates() {
    switch (systemState) {
      case IDLE:
        intakeIO.setSpinnerVoltage(0);
        intakeIO.setExtensionVoltage(0);
        break;
      case EXTENDED_INTAKING:
        intakeIO.setExtensionMotorPositionRad((0.0));
        intakeIO.setSpinnerVoltage(3);
        break;
      case EXTENDED_PASSIVE:
        intakeIO.setExtensionMotorPositionRad(0.0);
        intakeIO.setSpinnerVoltage(0);
        break;
      case MOVING_TO_EXTENSION:
        intakeIO.setExtensionMotorPositionRad((0.0));
        intakeIO.setSpinnerVoltage(1.5);
      case STOWED:
        intakeIO.setExtensionMotorPositionRad(Math.PI / 2);
        intakeIO.setSpinnerVoltage(0);
        break;
      case MOVING_TO_STOW:
        intakeIO.setExtensionMotorPositionRad(Math.PI / 2);
        intakeIO.setSpinnerVoltage(-1.5);
    }
  }

  public void setState(WantedState state) {
    this.wantedState = state;
  }

  public boolean atWantedAngle(){
    if(wantedState == WantedState.EXTENDED_INTAKING || wantedState == WantedState.EXTENDED_PASSIVE){
      if(MathUtil.isNear(0.0, inputs.extensionPosRadians ,0.1)){
        return true;
      }
    }
    if(wantedState == WantedState.STOWED){
      if(MathUtil.isNear(Math.PI / 2, inputs.extensionPosRadians ,0.1)){
        return true;
      }
    }
    return false;
};

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    systemState = handleStateTransition();

    applyStates();
  }
}
