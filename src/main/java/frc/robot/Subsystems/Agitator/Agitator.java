// package frc.robot.Subsystems.Agitator;

// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Agitator extends SubsystemBase {
//   private Timer timer;

//   public enum WantedAgitatorState {
//     IDLE,
//     SPINNING
//   }

//   public enum SystemState {
//     IDLE,
//     SPINNING,
//     BACK_SPIN
//   }

//   private SystemState systemState = SystemState.IDLE;
//   private WantedAgitatorState wantedState = WantedAgitatorState.IDLE;

//   private final AgitatorIO agitatorIO;

//   private AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();

//   public Agitator(AgitatorIO agitatorIO) {
//     this.agitatorIO = agitatorIO;
//     timer = new Timer();
//   }

//   @Override
//   public void periodic() {
//     agitatorIO.updateInputs(inputs);

//     systemState = handleSystemState();
//     applyWantedState();
//     moveOnByTimer();
//     System.out.println("Agitator State: " + systemState);
//   }

//   public void setWantedAgitatorState(WantedAgitatorState wantedAgitatorState) {
//     if (wantedState == WantedAgitatorState.SPINNING
//         && systemState != SystemState.SPINNING
//         && systemState != SystemState.BACK_SPIN) {
//       timer.reset();
//       timer.start();
//     }
//     wantedState = wantedAgitatorState;
//   }

//   private SystemState handleSystemState() {
//     switch (wantedState) {
//       case IDLE:
//         return SystemState.IDLE;
//       case SPINNING:
//         {
//           if (!timer.isRunning()) {
//             return SystemState.SPINNING;
//           }
//           return SystemState.BACK_SPIN;
//         }
//       default:
//         return SystemState.IDLE;
//     }
//   }

//   private void moveOnByTimer() {
//     if (timer.hasElapsed(0.3)) {
//       timer.stop();
//       timer.reset();
//       systemState = SystemState.SPINNING;
//     }
//   }

//   private void applyWantedState() {
//     switch (systemState) {
//       case IDLE:
//         agitatorIO.setVoltage(0.0);
//         break;
//       case SPINNING:
//         agitatorIO.setVoltage(5.0);
//         break;
//       case BACK_SPIN:
//         agitatorIO.setVoltage(-3);
//         break;
//     }
//   }

//   // public void setWantedState(WantedAgitatorState wantedState) {
//   //   this.wantedState = wantedState;
//   // }
// }

package frc.robot.Subsystems.Agitator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Agitator extends SubsystemBase {

  public enum WantedAgitatorState {
    IDLE,
    SPINNING
  }

  public enum SystemState {
    IDLE,
    BACK_SPIN,
    SPINNING
  }

  private final AgitatorIO agitatorIO;
  private final AgitatorIOInputsAutoLogged inputs = new AgitatorIOInputsAutoLogged();
  private final Timer timer = new Timer();

  private WantedAgitatorState wantedState = WantedAgitatorState.IDLE;
  private SystemState systemState = SystemState.IDLE;

  public Agitator(AgitatorIO agitatorIO) {
    this.agitatorIO = agitatorIO;
  }

  @Override
  public void periodic() {
    agitatorIO.updateInputs(inputs);

    systemState = handleSystemState();
    applyWantedState();

    System.out.println("Agitator State: " + systemState);
  }

  public void setWantedAgitatorState(WantedAgitatorState state) {
    this.wantedState = state;
  }

  private SystemState handleSystemState() {
    switch (wantedState) {
      case IDLE:
        timer.stop();
        timer.reset();
        return SystemState.IDLE;

      case SPINNING:
        switch (systemState) {
          case IDLE:
            // Entering spin for first time
            timer.restart();
            return SystemState.BACK_SPIN;

          case BACK_SPIN:
            if (timer.hasElapsed(0.3)) {
              timer.stop();
              return SystemState.SPINNING;
            }
            return SystemState.BACK_SPIN;

          case SPINNING:
            return SystemState.SPINNING;
        }
    }

    return SystemState.IDLE;
  }

  private void applyWantedState() {
    switch (systemState) {
      case IDLE:
        agitatorIO.setVoltage(0.0);
        break;

      case BACK_SPIN:
        agitatorIO.setVoltage(-1.0); // reverse for 0.3s
        break;

      case SPINNING:
        agitatorIO.setVoltage(3.0); // forward continuously
        break;
    }
  }
}
