package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import java.util.function.DoubleSupplier;

public class HoodIOSim implements HoodIO {

  public DCMotor motor;
  public ProfiledPIDController pidController;
  public ArmFeedforward feedforward;
  public SingleJointedArmSim sim;

  public HoodIOSim() {
    motor = DCMotor.getKrakenX44(1);
    // sim = new SingleJointedArmSim(
    // 	motor,
    // 	HoodConstants.GEAR_RATIO,
    // 	HoodConstants.MOMENT_OF_INERTIA,
    // 	getAngle(),
    // 	getAngle(),
    // 	getAngle(),
    // 	true,
    // 	getAngle(),
    // 	null)

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
    inputs.appliedVoltage = 0;
    // inputs.current = motor.getStatorCurrent().getValueAsDouble();
    // inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.angle = getAngle();
    inputs.angleSetpoint = pidController.getSetpoint().position;
  }

  @Override
  public void setVoltage(double voltage) {
    // motor.setVoltage(voltage);
  }

  // @Override
  // public double getAngle() {
  // 	// return motor.getPosition().getValueAsDouble();
  // }

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

  // @Override
  // public void stop() {
  // 	motor.stopMotor();
  // }

  @Override
  public boolean atSetpoint() {
    return pidController.atGoal();
  }

  @Override
  public double getAngle() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
  }

  @Override
  public void stop() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'stop'");
  }
}
