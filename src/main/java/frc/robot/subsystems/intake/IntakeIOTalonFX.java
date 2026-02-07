package frc.robot.subsystems.intake;

import static frc.robot.constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX motor;
  TalonFXConfiguration config;

  public IntakeIOTalonFX() {
    // define motor
    motor = new TalonFX(MOTOR_ID);

    // define config
    config = new TalonFXConfiguration();

    // motor setup and inverted setting
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: needed?
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // get double (ints) as outputs
    inputs.current = motor.getStatorCurrent().getValueAsDouble();
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.temperature = motor.getDeviceTemp().getValueAsDouble();
    inputs.velocity = getVelocity();
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }
}
