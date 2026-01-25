package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class TurretIOTalonFX implements TurretIO {

  private TalonFX motor;
  private MotionMagicVoltage mm_req;
  private double robotRelativeSetpoint;
  private double fieldRelativeSetpoint;
  private ProfiledPIDController pidController;
  private double voltageToUse;

  public TurretIOTalonFX() {

    robotRelativeSetpoint = 0;
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
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.velocity = motor.getVelocity().getValueAsDouble(); // rps
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
    inputs.robotRelativeAngle = getRobotRelativeAngle();
    inputs.robotRelativeAngleSetpoint = robotRelativeSetpoint;
    inputs.fieldRelativeAngle = getFieldRelativeAngle();
    inputs.fieldRelativeAngleSetpoint = fieldRelativeSetpoint;
  }

  @Override
  public void setRobotRelativeAngle(double angle) {
    robotRelativeSetpoint = angle;
    // converts to motor output shaft rotations and makes request to motion magic for a new
    // setpoint.
    pidController.setGoal(new TrapezoidProfile.State());
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
    double robotRelativeAngleSetpoint =
        angle - RobotContainer.drivetrain.getWrappedHeading().getRotations();
    setRobotRelativeAngle(robotRelativeAngleSetpoint);
  }

  @Override
  public void setFieldRelativeAngle(DoubleSupplier supplier) {
    setFieldRelativeAngle(supplier.getAsDouble());
  }

  @Override
  public double getFieldRelativeAngle() {
    return getRobotRelativeAngle() + RobotContainer.drivetrain.getWrappedHeading().getRotations();
  }

  @Override
  public boolean atSetpoint() {
    return Math.abs(
            getRobotRelativeAngle() - mm_req.getPositionMeasure().in(Rotations) / GEAR_RATIO)
        < ANGLE_TOLERANCE.in(Rotations);
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
