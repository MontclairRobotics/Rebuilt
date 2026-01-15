package frc.robot.subsystems.intake;

import java.lang.System.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    public IntakeIOInputsAutoLogged = new IntakeIOInputsAutoLogged(); 
    private final Debouncer isStalledDebouncer = new Debouncer(0.05, DebounceType.kRising);
    
    public Intake(IntakeIO io){
        this.io = io; 
    }

  public Debouncer isStalled() {
    return isStalledDebouncer.calculate(
        inputs.Current > RollerConstants.ROLLER_STALL_CURRENT);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Rollers", inputs);
  }

}



