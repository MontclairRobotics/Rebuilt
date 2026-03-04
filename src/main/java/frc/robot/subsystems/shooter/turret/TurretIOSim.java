package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class TurretIOSim implements TurretIO {

    private final SingleJointedArmSim sim;
    private double appliedVoltage = 0;
    private ProfiledPIDController pidController;
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

        pidController = new ProfiledPIDController(
            50, 0, 2,
            new Constraints(
                MAX_VELOCITY.in(RotationsPerSecond),
                MAX_ACCELERATION.in(RotationsPerSecondPerSecond)
            )
        );

        pidController.setTolerance(ANGLE_TOLERANCE.in(Rotations), VELOCITY_TOLERANCE.in(RotationsPerSecond));
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
       inputs.robotRelativeAngleSetpoint = Rotations.of(pidController.getGoal().position);

       inputs.isAtSetpoint = isAtSetpoint();
    }

    @Override
    public void setRobotRelativeAngle(Angle angle, AngularVelocity velocity) {
        pidController.setGoal(new State(angle.in(Rotations), velocity.in(RotationsPerSecond)));
        double pidOutput = pidController.calculate(Radians.of(sim.getAngleRads()).in(Rotations));
        double ffOutput = feedforward.calculate(velocity.in(RotationsPerSecond));
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
    public void setMotionMagic(double velocity, double acceleration, double jerk) {
        // does nothing
    }

    @Override
    public void setNeutralMode(NeutralModeValue value) {
       // does nothing
    }

}
