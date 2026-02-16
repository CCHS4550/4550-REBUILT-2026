package frc.robot.Config;

public class IntakeConfig {
    public double extensionkP;

    public double extensionkI;

    public double extensionkD;

    public double extensionkS;

    public double extensionkV;

    public double extensionkG;

    public IntakeConfig withExtensionkP(double extensionkP) {
    this.extensionkP = extensionkP;
    return this;
  }

  public IntakeConfig withExtensionkI(double extensionkI) {
    this.extensionkI = extensionkI;
    return this;
  }

  public IntakeConfig withExtensionkD(double extensionkD) {
    this.extensionkD = extensionkD;
    return this;
  }

  public IntakeConfig withExtensionkS(double extensionkS) {
    this.extensionkS = extensionkS;
    return this;
  }

  public IntakeConfig withExtensionkV(double extensionkV) {
    this.extensionkV = extensionkV;
    return this;
  }

  public IntakeConfig withExtensionkG(double extensionkG) {
    this.extensionkG = extensionkG;
    return this;
  }
}
