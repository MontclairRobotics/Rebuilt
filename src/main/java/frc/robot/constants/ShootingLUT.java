package frc.robot.constants;

import java.util.Map;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;


public class ShootingLUT {
    public static final InterpolatingTreeMap<Double,Double> ANGLE_MAP = new InterpolatingTreeMap<Double,Double>(InverseInterpolator.forDouble(), Interpolator.forDouble());
    public static final InterpolatingTreeMap<Double,Double> TIME_MAP = new InterpolatingTreeMap<Double,Double>(InverseInterpolator.forDouble(),Interpolator.forDouble());
    static {
        ANGLE_MAP.put(0.0, 0.0);
        TIME_MAP.put(0.0, 0.0);
    }
}
