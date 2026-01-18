package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;
import frc.robot.constants.TurretConstants;
import java.util.function.DoubleSupplier;

public class TurretIOSim implements TurretIO {

  TurretIOSim() {}

  DCMotor gearbox = DCMotor.getKrakenX60(0);
  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          gearbox,
          TurretConstants.GEAR_RATIO,
          TurretConstants.MOMENT_OF_INERTIA,
          TurretConstants.LENGTH,
          0,
          (TurretConstants.MAX_ANGLE / 180) * Math.PI,
          false,
          0,
          0);
  double appliedVoltage = 0;
  PIDController pid =
      new PIDController(TurretConstants.SIM_KP, TurretConstants.SIM_KI, TurretConstants.SIM_KD);
  double encoderOffset = 0;

  @Override
  public void updateInputs(TurretIOInputs input) {
    input.velocity = sim.getVelocityRadPerSec() / (2 * Math.PI);
    input.appliedVoltage = appliedVoltage;
  }

  @Override
  public void setRobotRelativeAngle(double angle) {
    pid.setSetpoint(angle);
    setVoltage(pid.calculate(getRobotRelativeAngle()));
  }

  @Override
  public double getRobotRelativeAngle() {
    return (sim.getAngleRads() / (2 * Math.PI)) + encoderOffset;
  }

  @Override
  public void zeroRelativeEncoder() {
    encoderOffset = -getRobotRelativeAngle();
  }

  @Override
  public void setRobotRelativeAngle(DoubleSupplier supplier) {
    setRobotRelativeAngle(supplier.getAsDouble());
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    sim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    setVoltage(0);
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
