package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;

public class TurretIOTalonFX implements TurretIO {

  private TalonFX motor;
  private MotionMagicVoltage mm_req;
  private double robotRelativeSetpoint;
  private double fieldRelativeSetpoint;

  public TurretIOTalonFX() {

    robotRelativeSetpoint = 0;
    fieldRelativeSetpoint = 0;

    motor = new TalonFX(CAN_ID);
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = TALON_KS;
    slot0Configs.kV = TALON_KV;
    slot0Configs.kA = TALON_KA;
    slot0Configs.kP = TALON_KP;
    slot0Configs.kI = TALON_KI;
    slot0Configs.kD = TALON_KD;

    mm_req = new MotionMagicVoltage(0);

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity =
        TURRET_CRUISE_VELOCITY.in(RotationsPerSecond) * GEAR_RATIO;
    motionMagicConfigs.MotionMagicAcceleration =
        TURRET_ACCELERATION.in(RotationsPerSecondPerSecond) * GEAR_RATIO;
    motionMagicConfigs.MotionMagicJerk = TURRET_JERK * GEAR_RATIO;
    motor.getConfigurator().apply(talonFXConfigs);
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
    motor.setControl(
        mm_req
            .withPosition(wrapAngleSetpoint(angle) * GEAR_RATIO)
            .withSlot(0)
            .withEnableFOC(false));
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
        angle - RobotContainer.drivetrain.odometryHeading.getRotations();
    setRobotRelativeAngle(robotRelativeAngleSetpoint);
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
