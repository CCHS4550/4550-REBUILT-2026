package frc.robot.Util;

public class CanDeviceID {
  private final int deviceNumber;
  private final String bus;

  public CanDeviceID(int deviceNumber, String bus) {
    this.deviceNumber = deviceNumber;
    this.bus = bus;
  }

  // Use the default bus name "rio".
  public CanDeviceID(int deviceNumber) {
    this(deviceNumber, "rio");
  }

  public int getDeviceNumber() {
    return deviceNumber;
  }

  public String getBus() {
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
