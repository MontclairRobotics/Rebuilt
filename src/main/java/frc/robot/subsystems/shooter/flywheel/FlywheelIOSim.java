package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.FlywheelConstants.*;
import static frc.robot.constants.FlywheelConstants.GEARING;

public class FlywheelIOSim implements FlywheelIO {

    private final FlywheelSim sim;
    private double appliedVoltage;

    private ProfiledPIDController pidController;
    private SimpleMotorFeedforward feedforward;

    public FlywheelIOSim() {
        sim = new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60Foc(2),
                MOMENT_OF_INERTIA,
                GEARING
            ),
            DCMotor.getKrakenX60Foc(2),
            0.0
        );

        pidController = new ProfiledPIDController(
            kP, 0, kD,
            new Constraints(0, 0)
        );

        pidController.setTolerance(VELOCITY_TOLERANCE.in(RotationsPerSecond));

        feedforward = new SimpleMotorFeedforward(kS, kV);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        sim.setInputVoltage(appliedVoltage);
        sim.update(0.02);

        inputs.leftMotorConnected = true;
        inputs.rightMotorConnected = true;

        // sim returns MECHANISM values, so we multiply by gearing to convert to MOTOR SHAFT values
        // this is because the VelocityTorqueCurrentFOC request tunes PID gains based on MOTOR SHAFT properties, velocities, and setpoints
        inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec()).times(GEARING);
        inputs.acceleration = RadiansPerSecondPerSecond.of(sim.getAngularAccelerationRadPerSecSq()).times(GEARING);
        inputs.setpointVelocity = RotationsPerSecond.of(pidController.getGoal().position).times(GEARING); // for a flywheel, 'position' = velocity setpoint
        inputs.setpointAcceleration = RotationsPerSecondPerSecond.of(pidController.getGoal().velocity).times(GEARING); // for a flywheel, 'velocity' = acceleration setpoint

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentDrawAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 0;
        inputs.atGoal = pidController.atGoal();
    }

    @Override
    public void setVelocity(AngularVelocity targetVelocity) {
        // again, we multiply the sim value by gearing to convert values to MOTOR SHAFT values
        double pidOutput = pidController.calculate(
            RadiansPerSecond.of(sim.getAngularVelocityRadPerSec()).times(GEARING).in(RotationsPerSecond),
            targetVelocity.in(RotationsPerSecond)
        );
		double ffOutput = feedforward.calculate(targetVelocity.in(RotationsPerSecond));
		double totalOutput = pidOutput + ffOutput;
        appliedVoltage = MathUtil.clamp(totalOutput, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    }

    @Override
    public void setVoltage(double voltage) {
        appliedVoltage = voltage;
    }

    @Override
    public void stop() {
        appliedVoltage = 0;
    }

    @Override
    public boolean isAtSetpoint() {
        return pidController.atGoal();
    }

    @Override
    public void setGains(double kP, double kD, double kS, double kV) {
        pidController.setP(kP);
        pidController.setD(kD);
        feedforward.setKs(kS);
        feedforward.setKv(kV);
    }

}
