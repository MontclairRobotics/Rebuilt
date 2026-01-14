package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.FeedForwardConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FlywheelIOTalonFX implements FlywheelIO {

    private TalonFX talonFX;
    public PIDController pidController;
    public SimpleMotorFeedforward motorFeedForward; 

    public FlywheelIOTalonFX(){
        talonFX = new TalonFX(0);
        pidController = new PIDController(0, 0, 0);
        motorFeedForward = new SimpleMotorFeedforward(0, 0);
    }
    @Override
    public void updateInputs(FlywheelIOInputs inputs){
        inputs.appliedVoltageRPM = talonFX.getMotorVoltage().getValueAsDouble();
        inputs.tempCelcius = talonFX.getDeviceTemp().getValueAsDouble();
        inputs.currentVelocityRPM = talonFX.getVelocity().getValueAsDouble();
        inputs.isShooting = isShooting();
    }
    
    @Override
    public void stop(){
        talonFX.setVoltage(0);
    }
    
    @Override
    public void setVoltage(double appliedVoltageRPM){
        talonFX.setVoltage(appliedVoltageRPM);
    }
    public void shootVelocity(double currentVelocityRPM){
        motorFeedForward.calculate(currentVelocityRPM);
    }

    public void setPID(double kP, double kI, double kD){
        talonFX.s();
    }
    
    @Override
    public boolean isShooting(boolean isShooting){
         return isShooting;
    }
}
