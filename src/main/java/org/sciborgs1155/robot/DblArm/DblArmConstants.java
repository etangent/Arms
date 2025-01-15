package org.sciborgs1155.robot.DblArm;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
// https://www.chiefdelphi.com/t/whitepaper-two-jointed-arm-dynamics/423060

public class DblArmConstants {
  public static final Measure<Time> SIM_PERIOD = Seconds.of(.01);

  // Constants
  public static final Matrix<N2, N1> startPos = VecBuilder.fill(-Math.PI, 0);
  public static final double m1 = 1.0; // Mass of link 1 (kg)
  public static final double m2 = 1.0; // Mass of link 2 (kg)
  public static final double l1 = 1.0; // Length of link 1 (m)
  public static final double l2 = 1.0; // Length of link 2 (m)
  // Assuming uniform rods
  public static final double r1 = l1 / 2; // Distance to center of mass of link 1 (m)
  public static final double r2 = l2 / 2; // Distance to center of mass of link 2 (m)

  public static final double I1 =
      SingleJointedArmSim.estimateMOI(l1, m1); // Moment of inertia of link 1 (kg*m^2)
  public static final double I2 =
      SingleJointedArmSim.estimateMOI(l2, m2); // Moment of inertia of link 2 (kg*m^2)
  public static final double g = 9.81; // Gravitational acceleration (m/s^2)

  public static class Dynamics {
    public static Matrix<N2, N2> M(Matrix<N2, N1> pos) {
      Matrix<N2, N2> M = new Matrix<>(N2.instance, N2.instance);

      M.set(
          0,
          0,
          I1
              + I2
              + m1 * r1 * r1
              + m2 * (l1 * l1 + r2 * r2 + 2 * l1 * r2 * Math.cos(pos.get(1, 0))));
      M.set(0, 1, I2 + m2 * (r2 * r2 + l1 * r2 * Math.cos(pos.get(1, 0))));
      M.set(1, 0, I2 + m2 * (r2 * r2 + l1 * r2 * Math.cos(pos.get(1, 0))));
      M.set(1, 1, I2 + m2 * r2 * r2);

      return M;
    }

    public static Matrix<N2, N2> C(Matrix<N2, N1> pos, Matrix<N2, N1> velocity) {
      Matrix<N2, N2> C = new Matrix<>(N2.instance, N2.instance);

      C.set(0, 0, -m2 * l1 * r2 * Math.sin(pos.get(1, 0)) * velocity.get(1, 0));
      C.set(
          0,
          1,
          -m2 * l1 * r2 * Math.sin(pos.get(1, 0)) * (velocity.get(0, 0) + velocity.get(1, 0)));
      C.set(1, 0, m2 * l1 * r2 * Math.sin(pos.get(1, 0)) * velocity.get(1, 0));
      C.set(1, 1, 0);

      return C;
    }

    public static Matrix<N2, N1> Tg(Matrix<N2, N1> pos) {
      Matrix<N2, N1> Tg = new Matrix<>(N2.instance, N1.instance);

      Tg.set(
          0,
          0,
          g
              * (m1 * r1 * Math.cos(pos.get(0, 0))
                  + m2
                      * (l1 * Math.cos(pos.get(0, 0))
                          + r2 * Math.cos(pos.get(0, 0) + pos.get(1, 0)))));
      Tg.set(1, 0, g * m2 * r2 * Math.cos(pos.get(0, 0) + pos.get(1, 0)));

      return Tg;
    }
  }
}
