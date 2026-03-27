package frc.robot.subsystems.shooter.spindexer;

import static frc.robot.constants.SerializerConstants.SPIN_VELOCITY;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.shooter.spindexer.indexer.Indexer;
import frc.robot.subsystems.shooter.spindexer.serializer.Serializer;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Spindexer {

    private Serializer serializer;
    private Indexer indexer;

    public LoggedTunableNumber spinUpTime = new LoggedTunableNumber("Spindexer/Spin Up Time", 1);
    public LoggedTunableNumber spinDownTime = new LoggedTunableNumber("Spindexer/Spin Down Time", 0.1);

    public Spindexer(Serializer serializer, Indexer indexer) {
        this.serializer = serializer;
        this.indexer = indexer;
    }

    public void spinUp() {
        serializer.spinUp();
        indexer.spinUp();
    }

    public void spinDown() {
        serializer.spinDown();
        indexer.spinDown();
    }

    public Command setVoltageCommand(double voltage) {
        return Commands.run(() -> {
            serializer.setVoltage(6);
            indexer.setVoltage(voltage);
        }, serializer, indexer);
    }

    public Command spinUpCommand() {
        return Commands.run(() -> spinUp(), serializer, indexer);
    }

    public Command spinDownCommand() {
        return Commands.run(() -> spinDown(), serializer, indexer);
    }

    public Command jiggleCommand() {
        return Commands.parallel(
            indexer.spinUpCommand(),
            Commands.repeatingSequence(
                spinUpCommand().withTimeout(1),
                Commands.run(() -> {
                    indexer.setVelocity(SPIN_VELOCITY);
                    serializer.setVelocity(SPIN_VELOCITY.unaryMinus());
                }).withTimeout(1)
            )
        );
    }

    public Command jiggleFoReal() {
        return Commands.parallel(
            indexer.spinUpCommand(),
            Commands.repeatingSequence(
                Commands.startEnd(
                    () -> serializer.spinUp(), 
                    () -> serializer.stop(), 
                    serializer
                ).withTimeout(spinUpTime.getAsDouble()),
                Commands.startEnd(
                    () -> serializer.setVelocity(SPIN_VELOCITY.unaryMinus()), 
                    () -> serializer.stop(), 
                    serializer
                ).withTimeout(spinDownTime.getAsDouble())
            )
        );
    }

    public Command jiggleSerializerCommand() {
        return new SequentialCommandGroup(
            serializer.spinUpCommand().withTimeout(1),
            serializer.reverseCommand().withTimeout(1)
        ).repeatedly();
    }
}
