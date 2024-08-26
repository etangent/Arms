package org.sciborgs1155.robot.SingleArm;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.LENGTH;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.MIN_ANGLE;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import monologue.Annotations.Log;
import monologue.Logged;

public class SingleArmVisualizer implements Logged {
  @Log.NT private final Mechanism2d mech;
  private final MechanismLigament2d arm;

  public SingleArmVisualizer(Color8Bit color) {
    mech = new Mechanism2d(50, 50);
    MechanismRoot2d chassis = mech.getRoot("chassis", 10, 5);
    arm =
        chassis.append(
            new MechanismLigament2d(
                "arm", LENGTH.in(Meters) * 10, MIN_ANGLE.in(Radians), 3, color));
  }

  public void setAngle(double radians) {
    arm.setAngle(radians * 180 / Math.PI);
  }
}
