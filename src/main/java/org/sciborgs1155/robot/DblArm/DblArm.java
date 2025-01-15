package org.sciborgs1155.robot.DblArm;

import static edu.wpi.first.wpilibj2.command.button.RobotModeTriggers.teleop;
import static org.sciborgs1155.robot.DblArm.DblArmConstants.startPos;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import monologue.Logged;
import monologue.Annotations.Log;

public class DblArm extends SubsystemBase implements Logged {
  private final DblArmIO hardware;
  private final DblArmVisualizer setpointVisualizer =
      new DblArmVisualizer(new Color8Bit(0, 0, 255));
  private final DblArmVisualizer measurementVisualizer =
      new DblArmVisualizer(new Color8Bit(255, 0, 0));

  private final Matrix<N2, N1> setpoint = startPos;

  public static DblArm create() {
    return new DblArm(new SimDblArm());
  }

  public static DblArm none() {
    return new DblArm(new NoDblArm());
  }

  public DblArm(DblArmIO hardware) {
    this.hardware = hardware;
  }

  @Log.NT
  public Matrix<N2, N1> setpoint() {
    return this.setpoint;
  }

  public Matrix<N2, N1> position() {
    return hardware.position();
  }

  public Matrix<N2, N1> velocity() {
    return hardware.velocity();
  }

  @Log.NT
  public double shoulderPosition() {
    return position().get(0, 0);
  }

  @Log.NT
  public double shoulderVelocity() {
    return velocity().get(0, 0);
  }

  @Log.NT
  public double elbowPosition() {
    return position().get(1, 0);
  }

  @Log.NT
  public double elbowVelocity() {
    return velocity().get(1, 0);
  }

  @Log.NT
  public double test() {
    return 0;
  }

  @Override
  public void periodic() {
  }

  public void update() {
    if (teleop().getAsBoolean()) {
      hardware.setVoltage(VecBuilder.fill(0, 0));
      setpointVisualizer.setAngle(setpoint());
      measurementVisualizer.setAngle(position());
    }
  }
}
