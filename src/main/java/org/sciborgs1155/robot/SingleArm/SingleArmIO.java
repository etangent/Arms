package org.sciborgs1155.robot.SingleArm;

public interface SingleArmIO extends AutoCloseable {
  public void setVoltage(double voltage);

  /*
   * radians
   */
  public double getPosition();

  /*
   * radians per second
   */
  public double getVelocity();

  public double getAppliedVoltage();
}
