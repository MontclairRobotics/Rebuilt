package frc.robot.subsystems.intake;

import frc.robot.constants.IntakeConstants.IntakeConstants;

public class IntakeIOSim implements IntakeIO {
  private double appliedVoltage = 0.0;
  private double velocity = 0.0;
  private double current = 0.0;

  private static final double MAX_SPEED = 2000;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    velocity = (appliedVoltage * 12.0) * MAX_SPEED;
    current = Math.abs(appliedVoltage * 12.0) * IntakeConstants.MAX_CURRENT;

    // "population" of IOInputs
    inputs.appliedVoltage = appliedVoltage;
    inputs.velocity = velocity;
    inputs.current = current;
  }

  @Override
  public void setVoltage(double voltage) {}

  @Override
  public void set(double current) {}

  @Override
  public void stop() {
    motor.setVoltage(0);
  }
}
