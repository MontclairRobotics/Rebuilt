package frc.robot.subsystems.intake;

import static frc.robot.constants.PivotConstants.*;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.constants.PivotConstants;
import frc.robot.subsystems.intake.pivot.Pivot;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.util.tunables.LoggedTunableNumber;

public class Intake {
    private Pivot pivot;
    private Rollers rollers;

    public LoggedTunableNumber pivotJiggleVoltage = new LoggedTunableNumber("Pivot/Pivot Jiggle Voltage", 4);
    public Intake(Pivot pivot, Rollers rollers){
        this.pivot = pivot;
        this.rollers = rollers;
    }

    public Command jostleCommand() {
        return Commands.repeatingSequence(
            pivot.goToAngleCommand(MAX_ANGLE.div(2)),
            pivot.goToAngleCommand(MIN_ANGLE)
        ).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
        .finallyDo(() -> pivot.stopCommand());
    }

    public Command stopCommand() { //unused
        return Commands.parallel(
            rollers.spinDownCommand(),
            pivot.stopCommand()
        );
    }

    public Command intakeCommand() {
        return Commands.parallel(
            pivot.goToAngleCommand(PivotConstants.MIN_ANGLE),
            rollers.spinUpCommand()
        );
    }

    public Command jiggleCommand() {
        return Commands.repeatingSequence(
            pivot.goToAngleCommand(PivotConstants.JIGGLE_ANGLE).withTimeout(1),
            Commands.waitSeconds(0.1),
            pivot.goToAngleCommand(PivotConstants.MIN_ANGLE).withTimeout(1),
            Commands.waitSeconds(0.1)
        );
    }

}
