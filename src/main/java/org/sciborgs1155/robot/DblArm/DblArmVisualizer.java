package org.sciborgs1155.robot.DblArm;

import static org.sciborgs1155.robot.DblArm.DblArmConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import monologue.Annotations.Log;
import monologue.Logged;

public class DblArmVisualizer implements Logged {
  @Log.NT private final Mechanism2d mech;
  private final MechanismLigament2d shoulder, elbow;

  public DblArmVisualizer(Color8Bit color) {
    mech = new Mechanism2d(50, 50);
    MechanismRoot2d chassis = mech.getRoot("chassis", 25, 25);

    shoulder =
        chassis.append(new MechanismLigament2d("shoulder", l1 * 7, startPos.get(0, 0) * 180 / Math.PI, 3, color));
    elbow = shoulder.append(new MechanismLigament2d("elbow", l2 * 7, startPos.get(1, 0) * 180 / Math.PI, 3, color));
  }

  public void setAngle(Matrix<N2, N1> pos) {
    shoulder.setAngle(pos.get(0, 0) * 180 / Math.PI);
    elbow.setAngle(pos.get(1, 0) * 180 / Math.PI);
  }
}
