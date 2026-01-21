package frc.robot.subsystems.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.RobotContainer;
import java.util.function.DoubleSupplier;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static frc.robot.constants.TurretConstants.*;
public class TurretIOSim implements TurretIO {
    private SingleJointedArmSim sim;
    PIDController pidController;
    double appliedVoltage = 0;
	double fieldRelativeSetpoint =0;

   public TurretIOSim() {
      sim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(0),
            GEAR_RATIO,
            MOMENT_OF_INERTIA,
            LENGTH,
            0,
            MAX_ANGLE.in(Radians),
            false,
            0,
            0);
  	pidController = new PIDController(SIM_KP, SIM_KI, SIM_KD);
      
   }

    @Override
    public void updateInputs(TurretIOInputs input) {
      input.velocity = sim.getVelocityRadPerSec() / (2 * Math.PI);
      input.appliedVoltage = appliedVoltage;
	  input.robotRelativeAngle = getRobotRelativeAngle();
	  input.robotRelativeAngleSetpoint = pidController.getSetpoint();
	  input.fieldRelativeAngle = getFieldRelativeAngle();
	  input.fieldRelativeAngleSetpoint = fieldRelativeSetpoint;
    }

    @Override
    public void setRobotRelativeAngle(double angle) {
      pidController.setSetpoint(wrapAngleSetpoint(angle));
      setVoltage(pidController.calculate(getRobotRelativeAngle()));
    }

    @Override
    public double getRobotRelativeAngle() {
      return Radians.of(sim.getAngleRads()).in(Rotations);
    }

	@Override
	public double getFieldRelativeAngle(){
		return getRobotRelativeAngle()+RobotContainer.drivetrain.odometryHeading.getRotations();
	}

    @Override
    public void zeroRelativeEncoder() {
		sim.setState(0,0);
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
        double robotRelativeAngleSetpoint = angle - RobotContainer.drivetrain.odometryHeading.getRotations();
        fieldRelativeSetpoint = angle;
		setRobotRelativeAngle(robotRelativeAngleSetpoint);
    }

    @Override
    public void setFieldRelativeAngle(DoubleSupplier supplier) {
        setFieldRelativeAngle(supplier.getAsDouble());
    }

	@Override
	public boolean atSetpoint() {
		return pidController.atSetpoint();
	}

	@Override
	public double wrapAngleSetpoint(double angle) {
		if(angle>MAX_ANGLE.in(Rotations)) {
			return angle-1;
		}else if(angle<0){
			return angle+1;
		}else {
			return angle;
		}
	}
}
