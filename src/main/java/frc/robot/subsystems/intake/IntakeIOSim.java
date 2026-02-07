package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.constants.IntakeConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  private FlywheelSim sim;

  public IntakeIOSim() {
    sim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(
                DCMotor.getKrakenX60(2), MOMENT_OF_INERTIA, GEARING),
            DCMotor.getKrakenX60(2),
            0.0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    // "population" of IOInputs
    inputs.appliedVoltage = sim.getInputVoltage();
    inputs.velocity = getVelocity();
    inputs.current = sim.getCurrentDrawAmps();
    inputs.temperature = 0;
  }

  @Override
  public void setVoltage(double voltage) {
    sim.setInputVoltage(voltage);
  }

  @Override
  public void stop() {
    setVoltage(0);
  }

  @Override
  public double getVelocity() {
    return sim.getAngularVelocity().in(RotationsPerSecond);
  }
}
