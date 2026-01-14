package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.TurretConstants;

public class TurretIOTalonFX implements TurretIO{
    public TalonFX turretMotor = new TalonFX(TurretConstants.TURRET_MOTOR_CAN_ID);
    @Override
    public void updateInputs(TurretInputs input) {
        input.currentVelocity = turretMotor.getVelocity().getValue();
        input.currentVoltage = turretMotor.getMotorVoltage().getValue();

    }

    @Override
    public void setRobotRelativeAngle(Rotation2d angle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setRobotRelativeAngle'");
    }

    @Override
    public Rotation2d getRobotRelativeAngle() {
        return Rotation2d.fromRotations(turretMotor.getRotorPosition().getValueAsDouble()*TurretConstants.ENCODER_RATIO);
    }

    @Override
    public void zeroRelativeEncoder() {
        turretMotor.setPosition(0); //TODO: check if this is the right function
    }

    @Override
    public void setContinuousAngle(DoubleSupplier supplier) {
        setRobotRelativeAngle(Rotation2d.fromRadians(supplier.getAsDouble()));
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