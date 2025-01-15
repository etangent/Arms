package org.sciborgs1155.robot.DblArm;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;

public interface DblArmIO {
  /** should be called every loop */
  void setVoltage(Matrix<N2, N1> voltage);

  Matrix<N2, N1> position();

  Matrix<N2, N1> velocity();
}
