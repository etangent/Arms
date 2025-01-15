package org.sciborgs1155.robot.DblArm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public class NoDblArm implements DblArmIO {

  @Override
  public void setVoltage(Matrix<N2, N1> voltage) {}

  @Override
  public Matrix<N2, N1> position() {
    return VecBuilder.fill(0, 0);
  }

  @Override
  public Matrix<N2, N1> velocity() {
    return VecBuilder.fill(0, 0);
  }
}
