package org.sciborgs1155.robot.SingleArm;

import monologue.Logged;

public interface SingleArmIO extends AutoCloseable, Logged {
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
