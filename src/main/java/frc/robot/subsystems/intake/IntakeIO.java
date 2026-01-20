package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double current = 0;
    public double velocity = 0;
    public double appliedVoltage = 0;
  }

  /**
   * Update the IOInputs structure
   *
   * @param inputs IO Inputs to update
   */
  public void updateInputs(IntakeIOInputs inputs);
  /**
   * Applies voltages to the intake motor
   *
   * @param voltage voltage to send to the intake motor
   */
  public void setVoltage(double voltage);

  /**
   * Applies motor output to intake motor, represented by a double from -1 to 1
   *
   * @param motor output to the intake motor
   */
  public void set(double motor);

  /** Stops the motor */
  public void stop();
}
