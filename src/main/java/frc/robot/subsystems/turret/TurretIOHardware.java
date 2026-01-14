package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;

public class TurretIOHardware implements TurretIO{
    public TalonFX turretMotor;
    
    @Override
    public void updateInputs(TurretInputs input) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void setRobotRelativeAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRobotRelativeAngle'");
    }

    @Override
    public Rotation2d getRobotRelativeAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRobotRelativeAngle'");
    }

    @Override
    public void zeroRelativeEncoder() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'zeroRelativeEncoder'");
    }

    @Override
    public void setContinuousAngle(DoubleSupplier supplier) {
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

}