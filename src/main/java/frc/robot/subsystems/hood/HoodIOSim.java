package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import frc.robot.util.Tunable;
import java.util.function.DoubleSupplier;

public class HoodIOSim implements HoodIO {

  public DCMotor motor;
  public PIDController pidController;
  public ArmFeedforward feedforward;
  public SingleJointedArmSim sim;

  public DutyCycleEncoder realEncoder;
  public DutyCycleEncoderSim encoder;

  private double appliedVoltage;

  public HoodIOSim() {
    motor = DCMotor.getKrakenX44(1);

    Tunable kp = new Tunable("hood kP", kP, (value) -> pidController.setP(value));
    Tunable ki = new Tunable("hood kI", kI, (value) -> pidController.setI(value));
    Tunable kd = new Tunable("hood kD", kD, (value) -> pidController.setD(value));
    Tunable ks = new Tunable("hood ks", kS, (value) -> feedforward.setKs(value));
    Tunable kg = new Tunable("hood kg", kG, (value) -> feedforward.setKg(value));
    Tunable kv = new Tunable("hood kv", kV, (value) -> feedforward.setKv(value));

    pidController = new PIDController(HoodConstants.kP, HoodConstants.kI, HoodConstants.kD);
    feedforward = new ArmFeedforward(HoodConstants.kS, HoodConstants.kG, HoodConstants.kV);

    realEncoder =
        new DutyCycleEncoder(
            HoodConstants.ENCODER_PORT, 1, HoodConstants.HOOD_ENCODER_OFFSET.in(Rotations));
    encoder = new DutyCycleEncoderSim(realEncoder);
    encoder.setConnected(true);

    sim =
        new SingleJointedArmSim(
            motor,
            GEAR_RATIO,
            MOMENT_OF_INERTIA,
            HOOD_LENGTH.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            MIN_ANGLE.in(Radians),
            0.0,
            0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {

    sim.setInputVoltage(appliedVoltage);
    sim.update(0.02);

    inputs.appliedVoltage = appliedVoltage;
    inputs.current = sim.getCurrentDrawAmps();
    inputs.angle = getAngle();
    inputs.angleSetpoint = pidController.getSetpoint();

    // encoder.set(Radians.of(sim.getAngleRads()).in(Rotations));
  }

  @Override
  public void setVoltage(double voltage) {
    appliedVoltage = voltage;
    // encoder.set(Radians.of(sim.getAngleRads()).in(Rotations));
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
    setVoltage(0);
  }

  @Override
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }

  @Override
  public double getAngle() {
    return Radians.of(sim.getAngleRads()).in(Rotations);
  }
}
