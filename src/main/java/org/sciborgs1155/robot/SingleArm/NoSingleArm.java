package org.sciborgs1155.robot.SingleArm;

public class NoSingleArm implements SingleArmIO {
  @Override
  public void setVoltage(double voltage) {}

  @Override
  public double getPosition() {
    return 0;
  }

  @Override
  public double getVelocity() {
    return 0;
  }

  @Override
  public double getAppliedVoltage() {
    return 0;
  }

  @Override
  public void close() throws Exception {}
}
