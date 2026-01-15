package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Flywheel extends SubsystemBase {
  FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private double targetVelocity;
  public Flywheel(FlywheelIO io) {
    this.io = io;
  }
  

  @Override
  public void periodic() {
      io.updateInputs(inputs);
  }
  // public double getMotorRPM(){
  //     return inputs.currentMotorRPM;
  // }
  public Command shootCommand(){
    return Commands.run(() -> {io.setVelocityRPS(targetVelocity);}) ;
  }
  public Command ferryCommand(){
    return Commands.run(()-> {io.setVelocityRPS(targetVelocity);});
  } 
}