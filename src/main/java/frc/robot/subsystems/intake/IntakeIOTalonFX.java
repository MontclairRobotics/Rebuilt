package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.constants.Constants.IntakeConstants;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX motor;
  TalonFXConfiguration config;

  public IntakeIOTalonFX() {
    // define motor
    motor = new TalonFX(IntakeConstants.MOTOR_ID);

    // define config
    config = new TalonFXConfiguration();

    // motor setup and inverted setting
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // get double (ints) as outputs
    inputs.current = motor.getStatorCurrent().getValueAsDouble();
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void set(double speed) {
    // set the speed
    motor.set(speed);
  }

  @Override
  public void stop() {
    // stop
    motor.stopMotor();
  }

  @Override
  public void setVoltage(double voltage) {
    // set voltage (same as speed?)
    motor.setVoltage(voltage);
  }
}
