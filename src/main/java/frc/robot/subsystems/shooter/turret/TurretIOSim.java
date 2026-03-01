package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {

    private final SingleJointedArmSim sim;
    private double appliedVoltage = 0;
    private PIDController pidController;
    private SimpleMotorFeedforward feedforward;

    public TurretIOSim() {
		sim = new SingleJointedArmSim(
			DCMotor.getKrakenX60(1),
			GEARING,
			MOMENT_OF_INERTIA,
			LENGTH.in(Meters),
			MIN_ANGLE.in(Radians),
			MAX_ANGLE.in(Radians),
			false,
			0,
			0,
			0
		);

        pidController = new PIDController(100, 0, 0);
        pidController.setTolerance(ANGLE_TOLERANCE.in(Rotations));
        feedforward = new SimpleMotorFeedforward(0, 0);
	}

    @Override
    public void updateInputs(TurretIOInputs inputs) {
       sim.setInputVoltage(appliedVoltage);
       sim.update(0.02);

       inputs.motorConnected = false;

       inputs.appliedVoltage = appliedVoltage;
       inputs.currentDrawAmps = sim.getCurrentDrawAmps();
       inputs.tempCelcius = 0; // motor temperature is not simulated

       inputs.velocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
       inputs.robotRelativeAngle = Radians.of(sim.getAngleRads());
       inputs.fieldRelativeAngle = Turret.toFieldRelativeAngle(inputs.robotRelativeAngle);
       inputs.robotRelativeAngleSetpoint = Rotations.of(pidController.getSetpoint());

       inputs.isAtSetpoint = isAtSetpoint();
    }

    @Override
    public void setRobotRelativeAngle(Angle angle) {
        pidController.setSetpoint(angle.in(Rotations));
        double pidOutput = pidController.calculate(Radians.of(sim.getAngleRads()).in(Rotations));
        double ffOutput = feedforward.calculate(0);
        appliedVoltage = MathUtil.clamp(pidOutput + ffOutput, -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
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
    public void setGains(double kP, double kD, double kS) {
        // pidController.setP(kP);
        // pidController.setD(kD);
        // feedforward.setKs(kS);
    }

    @Override
    public void setNeutralMode(NeutralModeValue value) {
       // does nothing
    }

}
