package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class TurretIOTalonFX implements TurretIO {

  private TalonFX motor;
  private double fieldRelativeSetpoint;
  private ProfiledPIDController pidController;

  public TurretIOTalonFX() {
    fieldRelativeSetpoint = 0;

    motor = new TalonFX(CAN_ID);
    pidController =
        new ProfiledPIDController(
            TALON_KP,
            TALON_KI,
            TALON_KD,
            new TrapezoidProfile.Constraints(
                TURRET_CRUISE_VELOCITY.in(RotationsPerSecond),
                TURRET_ACCELERATION.in(RotationsPerSecondPerSecond)));
    pidController.disableContinuousInput();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.velocity = motor.getVelocity().getValueAsDouble(); // rps
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.robotRelativeAngle = getRobotRelativeAngle();
    inputs.robotRelativeAngleSetpoint = pidController.getSetpoint().position;
    inputs.fieldRelativeAngle = getFieldRelativeAngle();
    inputs.fieldRelativeAngleSetpoint = fieldRelativeSetpoint;
  }

  @Override
  public void setRobotRelativeAngle(double angle) {
    // converts to motor output shaft rotations and makes request to motion magic for a new
    // setpoint.
    pidController.setGoal(
        new TrapezoidProfile.State(
            wrapAngleSetpoint(angle),
            -RadiansPerSecond.of(RobotContainer.drivetrain.getState().Speeds.omegaRadiansPerSecond)
                .in(RotationsPerSecond)));
    setVoltage(pidController.calculate(getRobotRelativeAngle()));
  }

  @Override
  public double getRobotRelativeAngle() {
    return motor.getRotorPosition().getValue().in(Rotations) / GEAR_RATIO;
  }

  @Override
  public void zeroRelativeEncoder() {
    motor.setPosition(0);
  }

  @Override
  public void setRobotRelativeAngle(DoubleSupplier supplier) {
    setRobotRelativeAngle(supplier.getAsDouble());
  }

  @Override
  public void setVoltage(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void setFieldRelativeAngle(double angle) {
    fieldRelativeSetpoint = angle;
    setRobotRelativeAngle(angle - RobotContainer.drivetrain.odometryHeading.getRotations());
  }

  @Override
  public void setFieldRelativeAngle(DoubleSupplier supplier) {
    setFieldRelativeAngle(supplier.getAsDouble());
  }

  @Override
  public double getFieldRelativeAngle() {
    return getRobotRelativeAngle() + RobotContainer.drivetrain.odometryHeading.getRotations();
  }

  @Override
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  @Override
  public double wrapAngleSetpoint(double angle) {
    if (angle > MAX_ANGLE.in(Rotations)) {
      return angle - 1;
    } else if (angle < 0) {
      return angle + 1;
    } else {
      return angle;
    }
  }
}
