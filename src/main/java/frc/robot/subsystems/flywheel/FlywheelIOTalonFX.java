package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.constants.FlywheelConstants;

public class FlywheelIOTalonFX implements FlywheelIO {

    private TalonFX talonFX;
    private PIDController pidController;
    private SimpleMotorFeedforward motorFeedForward; 

    public FlywheelIOTalonFX() {
        talonFX = new TalonFX(FlywheelConstants.FLYWHEEL_TALON_FX_PORT);
        pidController = new PIDController(FlywheelConstants.SLOT0_CONFIGS.kP, FlywheelConstants.SLOT0_CONFIGS.kI, FlywheelConstants.SLOT0_CONFIGS.kD);
        motorFeedForward = new SimpleMotorFeedforward(FlywheelConstants.SLOT0_CONFIGS.kS, FlywheelConstants.SLOT0_CONFIGS.kV);
    }

    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.appliedVoltage = talonFX.getMotorVoltage().getValueAsDouble();
        inputs.tempCelcius = talonFX.getDeviceTemp().getValueAsDouble();
        inputs.velocity = talonFX.getVelocity().getValueAsDouble(); //RPS
    }
    
    public void stop() {
        talonFX.stopMotor();
    }
    
    public void setVoltage(double voltage) {
        talonFX.setVoltage(voltage);
    }

    public void setVelocityRPM(double targetRPM){
        double pidOutput = pidController.calculate(getMotorVelocity(), targetRPM);
        double ffVolts = motorFeedForward.calculate(targetRPM);
        double totalOutput = pidOutput + ffVolts;
        setVoltage(totalOutput);
    }
    
    public double getMotorVelocity(){
        return talonFX.getVelocity().getValueAsDouble();    
    }
  
}