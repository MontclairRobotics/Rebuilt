package frc.robot.subsystems.shooter.turret;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.GEARING;
import static frc.robot.constants.TurretConstants.LENGTH;
import static frc.robot.constants.TurretConstants.MAX_ANGLE;
import static frc.robot.constants.TurretConstants.MIN_ANGLE;
import static frc.robot.constants.TurretConstants.MOMENT_OF_INERTIA;
import static frc.robot.constants.TurretConstants.SLOT0_CONFIGS;

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

        pidController = new PIDController(SLOT0_CONFIGS.kP, SLOT0_CONFIGS.kI, SLOT0_CONFIGS.kD);
        feedforward = new SimpleMotorFeedforward(SLOT0_CONFIGS.kS, SLOT0_CONFIGS.kV);
	}

    @Override
    public void updateInputs(TurretIOInputs inputs) {
       sim.setInputVoltage(appliedVoltage);
       sim.update(0.02);

       inputs.motorConnected = false;

       inputs.appliedVoltage = appliedVoltage;
       inputs.currentDrawAmps = sim.getCurrentDrawAmps();
       inputs.tempCelcius = 0; // motor temperature is not simulated 

       // sim tracks everything relative to the MECHANISM, meaning we have to multiply each output
       // by the GEARING value to convert it to MOTOR SHAFT values
       inputs.motorVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec()).times(GEARING);
       inputs.motorPosition = Radians.of(sim.getAngleRads()).times(GEARING);

       inputs.robotRelativeAngle = Radians.of(sim.getAngleRads());
       inputs.fieldRelativeAngle = Turret.toFieldRelativeAngle(inputs.robotRelativeAngle);

       // the PIDController object's unit of reference is the ROTATIONS of the MOTOR SHAFT which means
       // that we need to divide by GEARING to convert back to the MECHANISMS frame of reference
       inputs.robotRelativeAngleSetpoint = Rotations.of(pidController.getSetpoint()).div(GEARING);
       inputs.motorPositionSetpoint = Rotations.of(pidController.getSetpoint());
    }

    @Override
    public void setRobotRelativeAngle(Angle angle) {
        // we constrain the angle between MIN_ANGLE and MAX_ANGLE, then multiply by gearing to convert
        // the setpoint to the MOTOR SHAFTS frame of reference
        double motorPositionSetpoint = Turret.constrainAngle(angle).times(GEARING).in(Rotations);
        pidController.setSetpoint(motorPositionSetpoint);

        // we need to multiply the sim anlge by GEARING here to convert MECHANISM rotations to MOTOR SHAFT rotations
        double pidOutput = pidController.calculate(Radians.of(sim.getAngleRads()).times(GEARING).in(Rotations));
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
        pidController.setP(kP);
        pidController.setD(kD);
        feedforward.setKs(kS);
    }
    
}
