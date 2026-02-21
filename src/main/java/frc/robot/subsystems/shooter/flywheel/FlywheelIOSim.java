package frc.robot.subsystems.shooter.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
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

    private PIDController pidController;
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

        pidController = new PIDController(
            kP, 0, kD
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

        inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
        inputs.acceleration = RadiansPerSecondPerSecond.of(sim.getAngularAccelerationRadPerSecSq());
        inputs.setpointVelocity = RotationsPerSecond.of(pidController.getSetpoint());
        inputs.setpointAcceleration = RotationsPerSecondPerSecond.zero(); // PIDController does not set 'velocity' setpoints

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentDrawAmps = sim.getCurrentDrawAmps();
        inputs.tempCelsius = 0;
        inputs.isAtSetpoint = isAtSetpoint();
    }

    @Override
    public void setVelocity(AngularVelocity targetVelocity) {
        double pidOutput = pidController.calculate(
            RadiansPerSecond.of(sim.getAngularVelocityRadPerSec()).in(RotationsPerSecond),
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
        return pidController.atSetpoint();
    }

    @Override
    public void setGains(double kP, double kD, double kS, double kV) {
        pidController.setP(kP);
        pidController.setD(kD);
        feedforward.setKs(kS);
        feedforward.setKv(kV);
    }

}
