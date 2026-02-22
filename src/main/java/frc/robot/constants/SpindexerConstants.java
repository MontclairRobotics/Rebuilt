package frc.robot.constants;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;

public class SpindexerConstants {
  public static final int SPIN_ID = 20; //TODO: GET MOTOR IDS
  public static final int INDEX_ID = 21;

    public static final double SPIN_VOLTAGE = 12;
    public static final double INDEX_VOLTAGE = -12;

    public static final AngularVelocity MAX_SPIN_VELOCITY = RotationsPerSecond.zero();
    public static final AngularVelocity MAX_INDEX_VELOCITY = RotationsPerSecond.zero();
}
