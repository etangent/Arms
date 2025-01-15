package org.sciborgs1155.robot.SingleArm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class SingleArmConstants {
  public static final int THROUGHBORE_CPR = 8192;
  public static final int GEARING = 120;
  public static final int THROUGHBORE_GEARING = 1;
  public static final int CURRENT_LIMIT = 50;
  public static final Measure<Angle> MIN_ANGLE = Radians.of(-Math.PI / 2);
  public static final Measure<Angle> MAX_ANGLE = Radians.of(3 * Math.PI / 2);
  public static final Measure<Angle> STARTING_ANGLE = Radians.of(0);
  public static final Measure<Velocity<Angle>> MAX_SPEED = RadiansPerSecond.of(3);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCEL =
      RadiansPerSecond.per(Second).of(4);
  public static final Measure<Distance> LENGTH = Meters.of(.5);
  public static final Measure<Mass> MASS = Kilograms.of(8);

  public static final double kP = 10;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kG = .533;
  public static final double kV = 1;
  public static final double kA = 0;
}
