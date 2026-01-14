package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

public interface HoodIO {

  public static class HoodIOInputs {
    public double appliedVoltage;
    public double current;
    public double tempCelsius;
    public double angle; // rotations
    public double angleSetpoint; // rotations
  }

  public void updateInputs(HoodIOInputs inputs);

  public void setVoltage(double voltage);

  /** Gets the angle in Rotations from the relative encoder */
  public double getAngle();

  public void setAngle(double goal);

  public void setAngle(DoubleSupplier goalSupplier);

  public void stop();

  public boolean atSetpoint();
}
