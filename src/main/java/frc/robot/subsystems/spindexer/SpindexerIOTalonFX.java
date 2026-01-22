package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.constants.SpindexerConstants;

public class SpindexerIOTalonFX implements SpindexerIO {

  public static TalonFX motor;

  public SpindexerIOTalonFX() {
    motor = new TalonFX(SpindexerConstants.CAN_ID);
  }

  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble(); // celsius
    inputs.velocity = motor.getVelocity().getValueAsDouble(); // RPS
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public double getMotorVelocity() {
    return motor.getVelocity().getValueAsDouble() / SpindexerConstants.GEARING;
  }
}
