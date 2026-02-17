package frc.robot.Subsystems.Intake;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


// This code is Stupid and needs the be cleaned up.
public class Intake extends SubsystemBase {
  private final Timer timer = new Timer();

  private enum CurrentPos{
    STOWED,
    MOVING,
    EXTENDED
  }

  private enum EndGoal {
    EXTENDED,
    STOWED
  }

  private enum CurrentNeutralMode{
    BRAKE,
    COAST
  }


  public enum WantedIntakeState {
    EXTENDED_INTAKING,
    STOWED,
    PASSIVE_GRAVITY,
    IDLE
  }

  public enum SystemState {
    EXTENDED_INTAKING,
    MOVING_TO_EXTENSION,
    STOWED,
    MOVING_TO_STOW,
    PASSIVE_GRAVITY,
    IDLE
  }

  private SystemState systemState = SystemState.IDLE;
  private WantedIntakeState wantedState = WantedIntakeState.IDLE;
  private EndGoal endGoal = EndGoal.STOWED;
  private CurrentNeutralMode currentNeutralMode = CurrentNeutralMode.BRAKE;
  private CurrentPos currentPos = CurrentPos.STOWED;

  private final IntakeIO intakeIO;

  private IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  private SystemState handleStateTransition() {
    switch(wantedState){
      case IDLE: return SystemState.IDLE;
      case EXTENDED_INTAKING: 
        if(currentPos == CurrentPos.MOVING){
          return SystemState.MOVING_TO_EXTENSION;
        }
        else{return SystemState.EXTENDED_INTAKING;}
      case STOWED:
       if(currentPos == CurrentPos.MOVING){
          return SystemState.MOVING_TO_STOW;
        }
        else{return SystemState.STOWED;}
      
      case PASSIVE_GRAVITY:
        return SystemState.PASSIVE_GRAVITY;
      default: return SystemState.IDLE;
      }
    }

  private void applyStates() {
    switch (systemState) {
      case EXTENDED_INTAKING:
        intakeIO.setExtensionVoltage(0.0);
        intakeIO.setSpinnerVoltage(6.0);
        break;
      case MOVING_TO_EXTENSION:
        intakeIO.setExtensionVoltage(-3.0);
        intakeIO.setSpinnerVoltage(-1.5);
        break;
      case MOVING_TO_STOW:
      intakeIO.setExtensionVoltage(3.0);
      intakeIO.setSpinnerVoltage(1.5);
      break;
      default:
        intakeIO.setExtensionVoltage(0);
        intakeIO.setSpinnerVoltage(0);
        break;
    }
  }

  public void setWantedIntakeState(WantedIntakeState state) {
    switch(state){
      case STOWED:
              if(currentPos != CurrentPos.STOWED){
                currentPos = CurrentPos.MOVING;
                intakeIO.setExtensionNeutralMode(NeutralModeValue.Coast);
                currentNeutralMode = CurrentNeutralMode.COAST;
                endGoal = EndGoal.STOWED;
                timer.reset();
                timer.start();
              }
                else{intakeIO.setExtensionNeutralMode(NeutralModeValue.Brake);
                currentNeutralMode = CurrentNeutralMode.BRAKE;}
              break;
      case PASSIVE_GRAVITY:
            timer.reset();
            timer.start();
            break;
      case EXTENDED_INTAKING:
              if(currentPos != CurrentPos.EXTENDED){
                currentPos = CurrentPos.MOVING;
                intakeIO.setExtensionNeutralMode(NeutralModeValue.Coast);
                currentNeutralMode = CurrentNeutralMode.COAST;
                endGoal = EndGoal.EXTENDED;
                timer.reset();
                timer.start();
              }
                else{intakeIO.setExtensionNeutralMode(NeutralModeValue.Brake);
                currentNeutralMode = CurrentNeutralMode.BRAKE;}
              break;
        default: break;
    }
    
    this.wantedState = state;
  }

  @AutoLogOutput(key = "Subsystems/Intake")
  public boolean atWantedAngle() {
    if (wantedState == WantedIntakeState.EXTENDED_INTAKING) {
      if (MathUtil.isNear(0.0, inputs.extensionPosRadians, 0.01)) {
        return true;
      }
    }
    if (wantedState == WantedIntakeState.STOWED) {
      if (MathUtil.isNear(Math.PI / 2, inputs.extensionPosRadians, 0.01)) {
        return true;
      }
    }
    return false;
  };

  private void endTimerStateAndMoveOn(){
    if(systemState == SystemState.MOVING_TO_EXTENSION || systemState == SystemState.MOVING_TO_STOW){
      if(timer.hasElapsed(0.7)){
        timer.stop();
        timer.reset();
        setWantedIntakeState(WantedIntakeState.PASSIVE_GRAVITY);
      }
    }
    if(systemState == SystemState.PASSIVE_GRAVITY){
      if(timer.hasElapsed(0.15)){
        timer.stop();
        timer.reset();
        switch(endGoal){
          case EXTENDED:
            currentPos = CurrentPos.EXTENDED;
            setWantedIntakeState(WantedIntakeState.EXTENDED_INTAKING);
            break;
          case STOWED:
            currentPos = CurrentPos.STOWED;
            setWantedIntakeState(WantedIntakeState.STOWED);
            break;
        }
      }
    }
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Subsystems/Intake", inputs);

    Logger.recordOutput("Subsystems/Intake/SystemState", systemState);
    Logger.recordOutput("Subsystems/Intake/DesiredState", wantedState);
    endTimerStateAndMoveOn();
    systemState = handleStateTransition();
    applyStates();
  }


}
