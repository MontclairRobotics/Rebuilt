package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.constants.HoodConstants;
import java.util.function.DoubleSupplier;

public class HoodIOTalonFX implements HoodIO {

  public TalonFX motor;
  public ProfiledPIDController pidController;
  public ArmFeedforward feedforward;

  public HoodIOTalonFX() {
    motor = new TalonFX(0);

    pidController =
        new ProfiledPIDController(
            0,
            0,
            0,
            new Constraints(
                HoodConstants.MAX_VELOCITY.in(RotationsPerSecond),
                HoodConstants.MAX_ACCELERATION.in(RotationsPerSecondPerSecond)));

    feedforward = new ArmFeedforward(0, 0, 0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.current = motor.getStatorCurrent().getValueAsDouble();
    inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.angle = getAngle();
    inputs.angleSetpoint = pidController.getSetpoint().position;
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  @Override
  public double getAngle() {
    return motor.getPosition().getValueAsDouble();
  }

  @Override
  public void setAngle(double goal) {
    double pidVoltage = pidController.calculate(getAngle(), goal);
    double feedforwardVoltage = feedforward.calculate(Rotations.of(getAngle()).in(Radians), 0);
    setVoltage(pidVoltage + feedforwardVoltage);
  }

  @Override
  public void setAngle(DoubleSupplier goalSupplier) {
    double currentGoal = goalSupplier.getAsDouble();
    setAngle(currentGoal);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public boolean atSetpoint() {
    return pidController.atGoal();
  }
}
