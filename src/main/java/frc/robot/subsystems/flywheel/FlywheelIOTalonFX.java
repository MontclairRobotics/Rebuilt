package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {

  private TalonFX motor;
  private PIDController pidController;
  private SimpleMotorFeedforward motorFeedForward;

  public FlywheelIOTalonFX() {
    motor = new TalonFX(FlywheelConstants.CAN_ID);
    pidController =
        new PIDController(
            FlywheelConstants.SLOT0_CONFIGS.kP,
            FlywheelConstants.SLOT0_CONFIGS.kI,
            FlywheelConstants.SLOT0_CONFIGS.kD);
    motorFeedForward =
        new SimpleMotorFeedforward(
            FlywheelConstants.SLOT0_CONFIGS.kS, FlywheelConstants.SLOT0_CONFIGS.kV);
  }

  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.tempCelcius = motor.getDeviceTemp().getValueAsDouble(); //celsius
    inputs.velocity = motor.getVelocity().getValueAsDouble(); // RPS
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
    setVoltage(totalOutput);
  }
  public double getMotorVelocity() {
    return motor.getVelocity().getValueAsDouble();
  }
  
  public void setVelocityRPS(DoubleSupplier targetVelocitySupplier){
    setVelocityRPS(targetVelocitySupplier.getAsDouble());
  }
}
