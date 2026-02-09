package frc.robot.Subsystems.Climber;

import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Subsystems.Climber.ClimberIO;

public class Climber extends SubsystemBase {
  private ClimberIO climberIO;

  private ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();
  
  private final SysIdRoutine rotationToHeightConv;

  public enum ClimberWantedState {
    L1,
    L23,
    RETRACTED,
    LEVELHOLD
  }
  public enum ClimberSystemState {
    L1,
    L23,
    RETRACTED,
    LEVELHOLD
  }

  private boolean atGoal = true;
  private ClimberSystemState systemState = ClimberSystemState.RETRACTED;
  private ClimberWantedState wantedState = ClimberWantedState.RETRACTED;

  public Climber(ClimberIO io){
    climberIO = io;

    rotationToHeightConv = 
      new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
            climberIO::setVoltage,
            log -> {},
            this
        )
    );

  }

  @Override
  public void periodic(){
    ClimberIO.updateInputs(climberInputs);

    systemState = handleStateTransitions();
    applyStates();
  }
  /* L3 climb sequence:
      RETRACTED ->
      L1 ->
      RETRACTED ->
      L23 ->
      RETRACTED ->
      L23 ->
      RETRACTED

      Auto climb sequence
      RETRACTED ->
      L1 ->
      LEVELHOLD

      ...and decend sequence
      LEVELHOLD ->
      L1-> 
      Move away ->
      RETRACTED

      Make sure to wait until the climber actually reaches the target height before changing the state
   */
  public boolean isAtTarget() {
    return atGoal;
  }
  public void setState(ClimberWantedState state){
    wantedState = state;
  }
  public ClimberSystemState handleStateTransitions(){
    return switch(wantedState){
      case L1 -> ClimberSystemState.L1;
      case L23 -> ClimberSystemState.L23;
      case RETRACTED -> ClimberSystemState.RETRACTED;
      case LEVELHOLD -> ClimberSystemState.LEVELHOLD;
      default -> ClimberSystemState.RETRACTED;
    };
  }
  public void applyStates() {
    switch (systemState) {
      case L1:
        ClimberIO.setHeight(1);
        // replace 1 with L1 height
        atGoal = climberInputs.climberHeightMeters == 1 ? true : false;
        break;
      case L23:
        ClimberIO.setHeight(2);
        // replace 2 with L2/L3 height
        atGoal = climberInputs.climberHeightMeters == 2 ? true : false;
        break;
      case RETRACTED:
        ClimberIO.setHeight(0);
        // replace 0 with retracted height
        atGoal = climberInputs.climberHeightMeters == 0 ? true : false;
        break;
      case LEVELHOLD:
        ClimberIO.setHeight(0.1);
        // replace 0.1 with retracted height but a bit more so that the static claws dont grab the bar
        atGoal = climberInputs.climberHeightMeters == 0.1 ? true : false;
        break;
      default:
        break;
      //the atGoal checks are probably cursed
    };
  }
}
