package org.sciborgs1155.robot.SingleArm;

import static edu.wpi.first.units.Units.*;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import monologue.Annotations.Log;
import monologue.Logged;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.Robot;

public class SingleArm extends SubsystemBase implements Logged, AutoCloseable {
  public static SingleArm create() {
    return Robot.isReal() ? new SingleArm(new RealSingleArm()) : new SingleArm(new SimSingleArm());
  }

  public static SingleArm none() {
    return new SingleArm(new NoSingleArm());
  }

  private final SingleArmIO hardware;

  @Log.NT
  private final ProfiledPIDController pid =
      new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(MAX_SPEED, MAX_ACCEL));

  @Log.NT private final ArmFeedforward ff = new ArmFeedforward(kS, kG, kV, kA);

  private final SingleArmVisualizer setpointVisualizer =
      new SingleArmVisualizer(new Color8Bit(0, 0, 255));

  private final SingleArmVisualizer measurementVisualizer =
      new SingleArmVisualizer(new Color8Bit(255, 0, 0));

  private final SysIdRoutine routine;

  public SingleArm(SingleArmIO hardware) {
    this.hardware = hardware;

    routine =
        new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                (voltage) -> hardware.setVoltage(voltage.in(Volts)),
                (log) -> {
                  log.motor("pivot")
                      .voltage(Volts.of(hardware.getAppliedVoltage()))
                      .angularPosition(Radians.of(hardware.getPosition()))
                      .angularVelocity(RadiansPerSecond.of(hardware.getVelocity()));
                },
                this));

    SmartDashboard.putData("qf single-arm", routine.quasistatic(Direction.kForward));
    SmartDashboard.putData("qb single-arm", routine.quasistatic(Direction.kReverse));
    SmartDashboard.putData("df single-arm", routine.dynamic(Direction.kForward));
    SmartDashboard.putData("db single-arm", routine.dynamic(Direction.kReverse));
  }

  @Log.NT
  public boolean atGoal() {
    return pid.atGoal();
  }

  @Log.NT
  public double goal() {
    return pid.getGoal().position;
  }

  @Log.NT
  public double setpoint() {
    return pid.getSetpoint().position;
  }

  @Log.NT
  public double position() {
    return hardware.getPosition();
  }

  @Log.NT
  public double velocity() {
    return hardware.getVelocity();
  }

  public Command goTo(DoubleSupplier angle) {
    return run(
        () -> {
          double goal =
              MathUtil.clamp(angle.getAsDouble(), MIN_ANGLE.in(Radians), MAX_ANGLE.in(Radians));
          double prevVelocity = pid.getSetpoint().velocity;
          double feedback = pid.calculate(hardware.getPosition(), goal);
          double feedforward =
              ff.calculate(
                  pid.getSetpoint().position,
                  pid.getSetpoint().velocity,
                  (pid.getSetpoint().velocity - prevVelocity) / Constants.PERIOD.in(Seconds));
          hardware.setVoltage(feedback + feedforward);
        });
  }

  @Override
  public void periodic() {
    setpointVisualizer.setAngle(setpoint());
    measurementVisualizer.setAngle(hardware.getPosition());
    log("command", Optional.ofNullable(getCurrentCommand()).map(Command::getName).orElse("none"));
  }

  @Override
  public void close() throws Exception {
    hardware.close();
  }
}
