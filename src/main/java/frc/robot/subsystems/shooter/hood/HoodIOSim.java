package frc.robot.subsystems.shooter.hood;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.HoodConstants.GEARING;
import static frc.robot.constants.HoodConstants.HOOD_LENGTH;
import static frc.robot.constants.HoodConstants.MAX_ANGLE;
import static frc.robot.constants.HoodConstants.MIN_ANGLE;
import static frc.robot.constants.HoodConstants.MOMENT_OF_INERTIA;
import static frc.robot.constants.HoodConstants.SLOT0_CONFIGS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {
    
    private SingleJointedArmSim sim;
    private double appliedVoltage;

    private PIDController pidController;
    private ArmFeedforward feedforward;

    public HoodIOSim() {
        sim = new SingleJointedArmSim(
			DCMotor.getKrakenX44Foc(1),
			GEARING,
			MOMENT_OF_INERTIA,
			HOOD_LENGTH.in(Meter),
			MIN_ANGLE.in(Radians),
			MAX_ANGLE.in(Radians),
			true,
			MIN_ANGLE.in(Radians),
			0.0,
			0.0
		);

        pidController = new PIDController(SLOT0_CONFIGS.kP, SLOT0_CONFIGS.kI, SLOT0_CONFIGS.kD); 
        feedforward = new ArmFeedforward(SLOT0_CONFIGS.kS, SLOT0_CONFIGS.kG, SLOT0_CONFIGS.kV);
        
    }

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        sim.setInputVoltage(appliedVoltage);
        sim.update(0.02);

        inputs.motorConnected = false;

        inputs.appliedVoltage = appliedVoltage;
        inputs.currentDrawAmps = sim.getCurrentDrawAmps();
        inputs.tempCelcius = 0; // motor temperature is not simulated

        // sim tracks everything relative to the MECHANISM, meaning we have to multiply each output
        // by the GEARING value to convert it to MOTOR SHAFT values
        inputs.motorPosition = Radians.of(sim.getAngleRads()).times(GEARING);
        inputs.motorPositionSetpoint = Rotations.of(pidController.getSetpoint());
        inputs.motorVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec()).times(GEARING);

        // the PIDController object's unit of reference is the ROTATIONS of the MOTOR SHAFT which means
        // that we need to divide by GEARING to convert back to the MECHANISMS frame of reference
        inputs.hoodAngle = Radians.of(sim.getAngleRads());
        inputs.hoodAngleSetpoint = Rotations.of(pidController.getSetpoint()).div(GEARING);
        inputs.hoodVelocity = RadiansPerSecond.of(sim.getVelocityRadPerSec());
    }

    @Override
    public void setAngle(Angle angle) {
        double motorPositionSetpoint = angle.times(GEARING).in(Rotations);
        pidController.setSetpoint(motorPositionSetpoint);

        // we need to multiply the sim angle by GEARING here to convert MECHANISM rotations to MOTOR SHAFT rotations
        double pidOutput = pidController.calculate(Radians.of(sim.getAngleRads()).times(GEARING).in(Rotations));
        double ffOutput = feedforward.calculate(sim.getAngleRads(), 0);
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
    public void setGains(double kP, double kD, double kS, double kG) {
        pidController.setP(kP);
        pidController.setD(kD);
        feedforward.setKs(kS);
        feedforward.setKg(kG);
    }
}
