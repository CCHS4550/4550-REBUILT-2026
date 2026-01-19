package frc.robot.Util;

import com.ctre.phoenix6.CANBus;

public class CanDeviceID {
  private final int deviceNumber;
  private final CANBus bus;

  public CanDeviceID(int deviceNumber, CANBus bus) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
  }

  // Use the default bus name "rio".
  public CanDeviceID(int deviceNumber) {
    this(deviceNumber, CANBus.roboRIO());
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBusName() {
    return bus.getName();
  }

  public String getBusString() {
    return bus.toString();
  }

  public CANBus getBus(){
    return bus;
  }

  public boolean equals(CanDeviceID other) {
    return other.deviceNumber == deviceNumber && other.bus.equals(bus);
  }

  @Override
  public String toString() {
    return "CanDeviceId(" + deviceNumber + ", " + bus + ")";
  }
}
