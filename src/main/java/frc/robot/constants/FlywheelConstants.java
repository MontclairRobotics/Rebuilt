package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class FlywheelConstants {
    public static final int FLYWHEEL_TALON_FX_PORT = 0;
    public static final Slot0Configs SLOT0_CONFIGS = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withKG(0);


    
    public static final double MOMENT_OF_INERTIA = 0;
    public static final double GEARING = 1; 
}
