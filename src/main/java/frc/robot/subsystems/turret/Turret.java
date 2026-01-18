package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and CANCoders to manage
 * turret movement and sensor readings.
 */
public class Turret extends SubsystemBase {

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }
  public Command stopCommand(){
    return Commands.runOnce(()->{io.stop();});
  }
  public Command setRobotRelativeAngleCommand(double target){
    return runOnce(()->{io.setRobotRelativeAngle(target);});
  }
  public Command setRobotRelativeAngleContinuousCommand(DoubleSupplier target){
    return runOnce(()->{io.setRobotRelativeAngle(target);});
  }

    public Command setFieldRelativeAngleCommand(double target){
    return runOnce(()->{io.setRobotRelativeAngle(target);});
  }
  public Command setFieldRelativeAngleContinuousCommand(DoubleSupplier target){
    return runOnce(()->{io.setRobotRelativeAngle(target);});
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }
}
