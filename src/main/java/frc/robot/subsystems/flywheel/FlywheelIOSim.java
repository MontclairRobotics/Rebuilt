package frc.robot.subsystems.flywheel;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.constants.FlywheelConstants;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class FlywheelIOSim implements FlywheelIO{
    private PIDController pidController;
    private SimpleMotorFeedforward motorFeedForward;
    
    private FlywheelSim sim = 
        new FlywheelSim(LinearSystemId.createFlywheelSystem(
            DCMotor.getFalcon500(1), 
            FlywheelConstants.MOMENT_OF_INERTIA, 
            FlywheelConstants.GEARING
            ),
            DCMotor.getFalcon500(1), 
            0.0
        );
    
    public FlywheelIOSim(){
        pidController = new PIDController(FlywheelConstants.SLOT0_CONFIGS.kP, FlywheelConstants.SLOT0_CONFIGS.kI, FlywheelConstants.SLOT0_CONFIGS.kD);
        motorFeedForward = new SimpleMotorFeedforward(FlywheelConstants.SLOT0_CONFIGS.kS, FlywheelConstants.SLOT0_CONFIGS.kV);

    }
    
    public void updateInputs (FlywheelIOInputs inputs){
        inputs.appliedVoltage = sim.getInputVoltage();
        inputs.velocity = sim.getAngularVelocityRadPerSec();
    }
    public void stop(){
        sim.setInputVoltage(0);
    }
    
}