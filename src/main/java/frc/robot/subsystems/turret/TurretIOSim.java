package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.TurretConstants;

import java.util.function.DoubleSupplier;
public class TurretIOSim implements TurretIO{

    TurretIOSim() {

    }
    DCMotor gearbox = DCMotor.getKrakenX60(0);
    private SingleJointedArmSim sim = new SingleJointedArmSim(gearbox,0,0,0,0, 0, false, 0, 0);
    @Override
    public void updateInputs(TurretIOInputs input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setRobotRelativeAngle(double angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRobotRelativeAngle'");
    }

    @Override
    public double getRobotRelativeAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRobotRelativeAngle'");
    }

    @Override
    public void zeroRelativeEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroRelativeEncoder'");
    }

    @Override
    public void setRobotRelativeAngle(DoubleSupplier supplier) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setContinuousAngle'");
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }
	@Override
	public void setFieldRelativeAngle(double angle) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setFieldRelativeAngle'");
	}

	@Override
	public void setFieldRelativeAngle(DoubleSupplier supplier) {
		// TODO Auto-generated method stub
		throw new UnsupportedOperationException("Unimplemented method 'setFieldRelativeAngle'");
	}

}