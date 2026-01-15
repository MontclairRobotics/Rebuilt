package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.HoodConstants;
import frc.robot.subsystems.hood.HoodIO.HoodIOInputs;
import java.util.function.DoubleSupplier;

public class HoodIOSim implements HoodIO {

  public DCMotor motor;
  public ProfiledPIDController pidController;
  public ArmFeedforward feedforward;
  public SingleJointedArmSim sim;

  public DutyCycleEncoder realEncoder;
  public DutyCycleEncoderSim encoder;
  
  private double appliedVoltage;

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

    realEncoder = new DutyCycleEncoder(5, 1, HoodConstants.HOOD_ENCODER_OFFSET.getRotations());
    encoder = new DutyCycleEncoderSim(realEncoder);
    encoder.setConnected(true);

    sim =
      new SingleJointedArmSim(
        motor,
        HoodConstants.GEAR_RATIO,
        1,
        HoodConstants.HOOD_LENGTH,
        HoodConstants.MIN_ANGLE.in(Radians),
        HoodConstants.MAX_ANGLE.in(Radians),
        true,
        0,
        0.0,
        0.0);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.appliedVoltage = appliedVoltage;
    inputs.current = sim.getCurrentDrawAmps();
    // inputs.current = motor.getStatorCurrent().getValueAsDouble();
    // inputs.tempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.angle = getAngle();
    inputs.angleSetpoint = pidController.getSetpoint().position;

    encoder.set(sim.getAngleRads()); 
  }

  @Override
  public void setVoltage(double voltage) {
    
    sim.setInputVoltage(voltage);
    encoder.set(sim.getAngleRads());
  }

  // @Override
  // public double getAngle() {
  // 	// return motor.getPosition().getValueAsDouble();
  // }

  @Override
  public void setAngle(double goal) {
    goal = MathUtil.clamp(
      goal,
      HoodConstants.MIN_ANGLE.in(Rotations),
      HoodConstants.MAX_ANGLE.in(Rotations));

    double pidVoltage = pidController.calculate(getAngle(), goal);
    double feedforwardVoltage = feedforward.calculate(Rotations.of(getAngle()).in(Radians), 0);
    MathUtil.clamp(pidVoltage + feedforwardVoltage, -HoodConstants.VOLTAGE_LIMIT, HoodConstants.VOLTAGE_LIMIT);
    setVoltage(pidVoltage + feedforwardVoltage);
  }

  @Override
  public void setAngle(DoubleSupplier goalSupplier) {
    double currentGoal = goalSupplier.getAsDouble();
    setAngle(currentGoal);
  }

  @Override
  public void stop() {
    sim.setInputVoltage(0);
  }

  @Override
  public boolean atSetpoint() {
    return pidController.atGoal();
  }

  @Override
  public double getAngle() {
    
    throw new UnsupportedOperationException("Unimplemented method 'getAngle'");
  }

  //@Override
 // public void stop() {
    // TODO Auto-generated method stub
  //  throw new UnsupportedOperationException("Unimplemented method 'stop'");
  //}
}
