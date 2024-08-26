package org.sciborgs1155.robot.SingleArm;

import static edu.wpi.first.units.Units.Radians;
import static org.sciborgs1155.lib.FaultLogger.*;
import static org.sciborgs1155.lib.SparkUtils.*;
import static org.sciborgs1155.robot.Ports.SingleArm.*;
import static org.sciborgs1155.robot.SingleArm.SingleArmConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import java.util.Set;
import org.sciborgs1155.lib.SparkUtils;
import org.sciborgs1155.lib.SparkUtils.Sensor;

public class RealSingleArm implements SingleArmIO {
  private final CANSparkMax pivot;
  private final RelativeEncoder encoder;

  public RealSingleArm() {
    pivot = new CANSparkMax(PIVOT, MotorType.kBrushless);
    encoder = pivot.getAlternateEncoder(THROUGHBORE_CPR);

    pivot.restoreFactoryDefaults();
    SparkUtils.configureFrameStrategy(
        pivot, Set.of(Data.POSITION, Data.VELOCITY), Set.of(Sensor.QUADRATURE), false);
    pivot.setSmartCurrentLimit(CURRENT_LIMIT);
    pivot.setIdleMode(IdleMode.kBrake);
    pivot.setSoftLimit(SoftLimitDirection.kForward, (float) MIN_ANGLE.in(Radians));
    encoder.setInverted(false);
    encoder.setPositionConversionFactor(2 * Math.PI / THROUGHBORE_GEARING);
    encoder.setVelocityConversionFactor(2 * Math.PI / 60 / THROUGHBORE_GEARING);
    encoder.setPosition(MIN_ANGLE.in(Radians));
    pivot.burnFlash();
    register(pivot);
  }

  @Override
  public void setVoltage(double voltage) {
    pivot.setVoltage(voltage);
  }

  @Override
  public double getPosition() {
    return encoder.getPosition();
  }

  @Override
  public double getVelocity() {
    return encoder.getVelocity();
  }

  @Override
  public double getAppliedVoltage() {
    return pivot.getAppliedOutput();
  }

  @Override
  public void close() throws Exception {
    pivot.close();
  }
}
