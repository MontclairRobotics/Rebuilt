package frc.robot.util.tunables;

/**
 * Contains classes for defining control constants and tunable controllers.
 * <p>
 * Uses AdvantageKit's {@link LoggedTunableNumber} for runtime tuning of control.
 */
public class TunableControls {


    /**
     * Wrapper around WPILib's {@link PIDController} that updates the controller
     * with runtime-tunable parameters provided by a
     * {@link TunableControlConstants}.
     *
     * <p>
     * <i>Note: For motion profiling, use {@link TunableProfiledController}.</i>
     */

    /**
     * Wrapper around WPILib's {@link ProfiledPIDController} and that configures the
     * controller using runtime-tunable values provided by a
     * {@link TunableControlConstants}. It also includes feedforward control
     * (defined by kS, kV, kA, kG).
     *
     * <p>
     * <i>Note: For non-motion-profiled control, use
     * {@link TunablePIDController}.</i>
     */

}
