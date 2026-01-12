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
        inputs.appliedVoltage = talonFX.getMotorVoltage().getValueAsDouble();
        inputs.tempC = talonFX.getDeviceTemp().getValueAsDouble();
        inputs.currentVelocity = talonFX.getVelocity().getValueAsDouble();
        inputs.isShooting = isShooting();
    }

    //@Override
    //public boolean isShooting() {}
    
    @Override
    public void stop(){
        talonFX.setVoltage(0);
    }
    
    @Override
    public void setVoltage(double appliedVoltage){
        talonFX.setVoltage(appliedVoltage);
    }
 
    public void shoot(){
         
    }
    
}
