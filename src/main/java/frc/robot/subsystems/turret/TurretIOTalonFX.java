package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;
import java.util.function.DoubleSupplier;

public class TurretIOTalonFX implements TurretIO {

  private TalonFX motor;
  private MotionMagicVoltage mm_req;

  public TurretIOTalonFX() {
    motor = new TalonFX(TurretConstants.CAN_ID);
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    Slot0Configs slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kS = 0.25; //TODO: UPDATE THESE AND MAKE CONSTANTS PROBABLY
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    mm_req = new MotionMagicVoltage(0);

    MotionMagicConfigs motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 4; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        16; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 160; // Target jerk of 1600 rps/s/s (0.1 seconds)
    motor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.velocity = motor.getVelocity().getValueAsDouble(); // rps
    inputs.appliedVoltage = motor.getMotorVoltage().getValueAsDouble();
  }

  @Override
  public void setRobotRelativeAngle(double angle) {
    double adjustedAngle;

    if (angle > 1) {
      angle -= 1;
    } else if (angle < 0) {
      angle += 1;
    }

    adjustedAngle = angle * TurretConstants.GEAR_RATIO; // converts to motor output shaft rotations
    motor.setControl(mm_req.withPosition(angle).withSlot(0).withEnableFOC(false));
  }

  @Override
  public double getRobotRelativeAngle() {
    return motor.getRotorPosition().getValueAsDouble() / TurretConstants.GEAR_RATIO;
  }

  @Override
  public void zeroRelativeEncoder() {
    motor.setPosition(0); // TODO: check if this is the right function
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
    double robotRelativeAngle = angle - RobotContainer.drivetrain.odometryHeading.getRotations();
    setRobotRelativeAngle(robotRelativeAngle);
  }

  @Override
  public void setFieldRelativeAngle(DoubleSupplier supplier) {
    setFieldRelativeAngle(supplier.getAsDouble());
  }
}
