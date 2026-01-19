package frc.robot.Util;

import edu.wpi.first.wpilibj.Threads;
import org.littletonrobotics.junction.LogDataReceiver;
import org.littletonrobotics.junction.LogTable;

/** Dummy log receiver used to adjust the thread priority of the log receiver thread. */
public class DummyLogReceiver implements LogDataReceiver {
  @Override
  public void start() {
    Threads.setCurrentThreadPriority(true, 1);
  }

  @Override
  public void putTable(LogTable table) throws InterruptedException {}
}
