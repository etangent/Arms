package org.sciborgs1155.robot.SingleArm;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.GEARING;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.LENGTH;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.MASS;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.MIN_ANGLE;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.STARTING_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import monologue.Annotations.Log;

import org.sciborgs1155.robot.Constants;

public class SimSingleArm implements SingleArmIO {
  private final SingleJointedArmSim arm;
  @Log.NT private double appliedOutput = 0;

  public SimSingleArm() {
    arm =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            GEARING,
            SingleJointedArmSim.estimateMOI(LENGTH.in(Meters), MASS.in(Kilograms)),
            LENGTH.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            STARTING_ANGLE.in(Radians));
  }

  @Override
  public void setVoltage(double voltage) {
    appliedOutput = voltage;
    arm.setInputVoltage(voltage);
    arm.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return arm.getAngleRads();
  }

  @Override
  public double getVelocity() {
    return arm.getVelocityRadPerSec();
  }

  @Override
  public double getAppliedVoltage() {
    return appliedOutput;
  }

  @Override
  public void close() throws Exception {}
}
