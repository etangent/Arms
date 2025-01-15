package org.sciborgs1155.robot.DblArm;

import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.DblArm.DblArmConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.DblArm.DblArmConstants.Dynamics;

public class SimDblArm implements DblArmIO {
  Matrix<N2, N1> pos = startPos;
  Matrix<N2, N1> velocity = VecBuilder.fill(0, 0);
  Matrix<N2, N1> accel = VecBuilder.fill(0, 0);
  DCMotor shoulder = DCMotor.getNeoVortex(2);
  DCMotor elbow = DCMotor.getNeoVortex(1);

  @Override
  public void setVoltage(Matrix<N2, N1> voltage) {
    update(SIM_PERIOD.in(Seconds), voltage);
  }

  @Override
  public Matrix<N2, N1> position() {
    return pos;
  }

  @Override
  public Matrix<N2, N1> velocity() {
    return velocity;
  }

  /**
   * @param period time from last call in seconds
   */
  private void update(double period, Matrix<N2, N1> voltage) {
    // pos = pos.plus(velocity.times(period));
    // velocity = velocity.plus(accel.times(period));


    Matrix<N2, N1> torque =
        VecBuilder.fill(
            shoulder.getTorque(shoulder.getCurrent(velocity.get(0, 0), voltage.get(0, 0))),
            elbow.getTorque(elbow.getCurrent(velocity.get(1, 0), voltage.get(1, 0))));
    // accel =
    //     Dynamics.M(pos)
    //         .inv()
    //         .times(torque.minus(Dynamics.C(pos, velocity).times(velocity)).minus(Dynamics.Tg(pos)));

        // Current state
        Matrix<N2, N1> k1_pos = velocity;
        Matrix<N2, N1> k1_vel = Dynamics.M(pos).inv().times(
            torque.minus(Dynamics.C(pos, velocity).times(velocity)).minus(Dynamics.Tg(pos))
        );
    
        // Intermediate state (k2)
        Matrix<N2, N1> k2_pos = velocity.plus(k1_vel.times(period).times(0.5));
        Matrix<N2, N1> k2_vel = Dynamics.M(pos.plus(k1_pos.times(period).times(0.5))).inv().times(
            torque.minus(Dynamics.C(pos.plus(k1_pos.times(period).times(0.5)), velocity.plus(k1_vel.times(period).times(0.5))).times(velocity)).minus(Dynamics.Tg(pos.plus(k1_pos.times(period).times(0.5))))
        );
    
        // Intermediate state (k3)
        Matrix<N2, N1> k3_pos = velocity.plus(k2_vel.times(period).times(0.5));
        Matrix<N2, N1> k3_vel = Dynamics.M(pos.plus(k2_pos.times(period).times(0.5))).inv().times(
            torque.minus(Dynamics.C(pos.plus(k2_pos.times(period).times(0.5)), velocity.plus(k2_vel.times(period).times(0.5))).times(velocity)).minus(Dynamics.Tg(pos.plus(k2_pos.times(period).times(0.5))))
        );
    
        // Intermediate state (k4)
        Matrix<N2, N1> k4_pos = velocity.plus(k3_vel.times(period));
        Matrix<N2, N1> k4_vel = Dynamics.M(pos.plus(k3_pos.times(period))).inv().times(
            torque.minus(Dynamics.C(pos.plus(k3_pos.times(period)), velocity.plus(k3_vel.times(period))).times(velocity)).minus(Dynamics.Tg(pos.plus(k3_pos.times(period))))
        );
    
        // Update state
        pos = pos.plus(k1_pos.plus(k2_pos.times(2)).plus(k3_pos.times(2)).plus(k4_pos).times(period).times(1.0 / 6.0));
        velocity = velocity.plus(k1_vel.plus(k2_vel.times(2)).plus(k3_vel.times(2)).plus(k4_vel).times(period).times(1.0 / 6.0));
  }
}
