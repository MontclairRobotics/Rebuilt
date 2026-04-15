package frc.robot.leds;


import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

import com.lumynlabs.devices.ConnectorX;
import com.lumynlabs.devices.ConnectorXAnimate;


public class LEDs extends SubsystemBase {
    public static final int PORT = 2;
    public static final int LENGTH = 80;
    public static final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);
    public static final Distance kLedSpacing = Meters.of(1 / 100.0);
    public static final LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.2),
            kLedSpacing);
    //private static LEDPattern alliancePattern = alliancePattern();
    //private static LEDPattern disabledAlliancePattern = disabledAlliancePattern();
    //private static Alliance prevAlliance;
    // public static final LEDPattern m_progressbar = LEDs.progressBar(Color.kRed);

    // private LEDPattern altPattern;
    // private Timer altTimer = new Timer();
    // private double altTime = 0;
    static AddressableLED led;
    static AddressableLEDBuffer ledBuffer;
    static ConnectorXAnimate cXAnimate = new ConnectorXAnimate();
    public static ConnectorX cx = new ConnectorX();
    static ConfigBuilder builder = new ConfigBuilder();
		public static LumynDeviceConfig cfg = builder
			.forTeam("555")
			.setNetworkType(NetworkType.USB)
			.addChannel(1, "main-channel", 80)  // Channel 1, name, total LEDs
				.addStripZone("all", 80)
                // .addStripZone("right",40)
				// .addStripZone("left", 40)
				.endChannel()
			.build();
    public LEDs() {
        led = new AddressableLED(PORT);
        led.setLength(LENGTH);
        ledBuffer = new AddressableLEDBuffer(LENGTH);
        led.start();

    }
    // public static void gasFireFlames(){
    //     cXAnimate.leds.SetAnimation(Animation.Fire)
    //         .ForZone("all-climbers")
    //         .WithColor(new Color(1.0, 0, 0))
    //         .WithDelay(Milliseconds.of(0))
    //         .RunOnce(false);
    // }

    // Usually Returns Blinking Synched with RSL, Currently Not For Testing Purposes
    public static LEDPattern blink(Color color) {
        LEDPattern object = LEDPattern.solid(color);
        // LEDPattern blinkingObj =
        // object.synchronizedBlink(RobotController::getRSLState);
        LEDPattern blinkingObj = object.blink(Seconds.of(0.1));
        return blinkingObj;
    }

    public Command getDefaultCommand() {
        return Commands.run(() -> {
            if(RobotContainer.flywheel.isShooting()){
                gasFireFlames();
            } 
            else if(RobotContainer.vision.hasAcceptedPose()){
                // acceptedCameraBlinks();
            }
            else {
                cXAnimate.leds.SetAnimation(Animation.RainbowRoll)
                    .ForZone("all")
                    .WithColor(new Color(255,255,255))
                    .WithDelay(Milliseconds.of(40))
                    .RunOnce(false);
            }

        }, this).ignoringDisable(true);
    }


    public void periodic() {
        if (DriverStation.isDisabled()) {
            Alliance alliance = DriverStation.getAlliance().orElseGet(() -> Alliance.Blue);
            if (alliance != prevAlliance || prevAlliance == null) {
                System.out.println("Switching");
                prevAlliance = alliance;
                alliancePattern = alliancePattern();
                disabledAlliancePattern = disabledAlliancePattern();
            }
        }
        // for (int i = 0; i < ledBuffer.getLength(); i++) {
        //     int red = ledBuffer.getRed(i);
        //     int green = ledBuffer.getGreen(i);
        //     int blue = ledBuffer.getBlue(i);
        //     ledBuffer.setRGB(i, green, red, blue); //RGB -> GBR
        // }
        led.setData(ledBuffer);
    }
}
