package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The TurretIOHardware class interfaces with the TalonFX motor controller and
 * CANCoders
 * to manage turret movement and sensor readings.
 */
public class Turret extends SubsystemBase {

    private final TurretIO io;
    private final TurretIO.TurretIOInputs inputs = new TurretInputs();

    public Turret(TurretIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        if (continuousSupplier != null) {
            io.setContinuousAngle(continuousSupplier);
        }
    }
}
