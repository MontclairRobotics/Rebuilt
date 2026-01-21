package frc.robot.subsystems.spindexer;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.SpindexerConstants;
import java.util.function.DoubleSupplier;

public class SpindexerIOTalonFX implements SpindexerIO {

  public static TalonFX motor;
  private PIDController pidController;
  private SimpleMotorFeedforward motorFeedForward;

  public SpindexerIOTalonFX() {
    motor = new TalonFX(SpindexerConstants.CAN_ID);
    pidController =
        new PIDController(
            SpindexerConstants.SLOT0_CONFIGS.kP,
            SpindexerConstants.SLOT0_CONFIGS.kI,
            SpindexerConstants.SLOT0_CONFIGS.kD);
    motorFeedForward =
        new SimpleMotorFeedforward(
            SpindexerConstants.SLOT0_CONFIGS.kS, SpindexerConstants.SLOT0_CONFIGS.kV);
  }

  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble(); // celsius
    inputs.velocity = motor.getVelocity().getValueAsDouble(); // RPS
    inputs.velocitySetpoint = pidController.getSetpoint();
  }

  public void stop() {
    motor.stopMotor();
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setVelocityRPS(double targetVelocity) {
    double pidOutput = pidController.calculate(getMotorVelocity(), targetVelocity);
    double ffVolts = motorFeedForward.calculate(targetVelocity);
    double totalOutput = pidOutput + ffVolts;
    setVoltage(MathUtil.clamp(totalOutput, 12.0, -12.0));
  }

  public boolean atSetPoint() {
    return pidController.atSetpoint();
  }

  public double getMotorVelocity() {
    return motor.getVelocity().getValueAsDouble() / SpindexerConstants.GEARING;
  }

  public void setVelocityRPS(DoubleSupplier targetVelocitySupplier) {
    setVelocityRPS(targetVelocitySupplier.getAsDouble());
  }
}
