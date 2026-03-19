
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    AddressableLED m_led  = new AddressableLED(1);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
    Distance LED_SPACING = Meters.of(16.0*12*0.0254 / 150);
    int brightness = 20;
    boolean lightson=false;
    String color = "off";
    // The below pattern should be periodically applied
    // To create a new pattern commands should just change the pattern
    LEDPattern ledPattern = LEDPattern.kOff;
    Boolean scroll = false;
    Boolean blueOn=false, greenOn=false, redOn=false, rainbowOn=false;
     // Reuse buffer
    // Length is expensive to set, so only set it once, then just update data
    
    public Lights() {
        m_led.setLength(m_ledBuffer.getLength());
        ledPattern.atBrightness(Percent.of(brightness));
        m_led.start();
        blue().execute();
    }

    /** Shared helper: applies a pattern or turns off if the same color is already active. */
    private void applyToggle(String name, LEDPattern pattern, boolean scrolling) {
        if (color.equals(name)) {
            // Same color requested again — turn off
            color = "off";
            lightson = false;
            ledPattern = LEDPattern.kOff;
            scroll = false;
            System.out.println(name + " toggled OFF");
        } else {
            color = name;
            lightson = true;
            ledPattern = pattern.atBrightness(Percent.of(brightness));
            scroll = scrolling;
            System.out.println(name + " toggled ON");
        }
        ledPattern.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }

    public Command red() {
        return runOnce(() -> applyToggle("red",
            LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen, Color.kBlack), false));
    }

     public Command red_on() {
        return runOnce(() -> {
            ledPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen, Color.kBlack).atBrightness(Percent.of(brightness));
            ledPattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        });
    }

    public Command off() {
        return runOnce(() -> {
            color = "off";
            lightson = false;
            ledPattern = LEDPattern.kOff;
            scroll = false;
            ledPattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
            System.out.println("Lights off");
        });
    }

    public Command blue() {
        return runOnce(() -> applyToggle("blue", LEDPattern.solid(Color.kBlue), false));
    }

    public Command green() {
        return runOnce(() -> applyToggle("green",
            LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlack), false));
            
    }

    public Command green_on() {
        return runOnce(() -> {
            ledPattern = LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed, Color.kBlack).atBrightness(Percent.of(brightness));
            ledPattern.applyTo(m_ledBuffer);
            m_led.setData(m_ledBuffer);
        });
    }

    public Command pink() {
        return runOnce(() -> applyToggle("pink", LEDPattern.solid(Color.kDarkCyan), false));
    }

    public Command rainbow() {
        return runOnce(() -> applyToggle("rainbow", LEDPattern.rainbow(255, 40), true));
    }

    public Command gradient() {
        return runOnce(() -> applyToggle("gradient",
            LEDPattern.gradient(GradientType.kDiscontinuous, Color.kCyan, Color.kBlack), true));
    }

    public Command pbar(int count){       
        return runOnce(
            () -> {
            // Create an LED pattern that displays a black-and-white mask that displays the current height of an elevator
            // mechanism. This can be combined with other patterns to change the displayed color to something other than white.
            ledPattern = LEDPattern.progressMaskLayer(() -> count / 10);
            ledPattern.atBrightness(Percent.of(brightness));
            scroll = false;
            });
        }

    /**
     * Toggle blue lights on/off.
     */
    public Command toggleBlue() {
        return runOnce(() -> {
            blueOn = !blueOn;
            if (blueOn) {
                blue().schedule();
            } else {
                off().schedule();
            }
            System.out.println("Lights: blue toggled " + (blueOn ? "ON" : "OFF"));
        });
    }

    /**
     * Toggle green lights on/off.
     */
    public Command toggleGreen() {
        return runOnce(() -> {
            greenOn = !greenOn;
            if (greenOn) {
                green().schedule();
            } else {
                off().schedule();
            }
            System.out.println("Lights: green toggled " + (greenOn ? "ON" : "OFF"));
        });
    }

    /**
     * Toggle rainbow on/off.
     */
    public Command toggleRainbow() {
        return runOnce(() -> {
            rainbowOn = !rainbowOn;
            if (rainbowOn) {
                rainbow().schedule();
            } else {
                off().schedule();
            }
            System.out.println("Lights: rainbow toggled " + (rainbowOn ? "ON" : "OFF"));
        });
    }

    /**
     * Cycle through colors in order: Blue → Red → Green → Rainbow → Off → Blue…
     */
    public Command cycleColor() {
        return runOnce(() -> {
            if (blueOn) {
                blueOn = false; redOn = true;
                red().schedule();
            } else if (redOn) {
                redOn = false; greenOn = true;
                green().schedule();
            } else if (greenOn) {
                greenOn = false; rainbowOn = true;
                rainbow().schedule();
            } else if (rainbowOn) {
                rainbowOn = false;
                off().schedule();
            } else {
                blueOn = true;
                blue().schedule();
            }
            System.out.println("Lights: cycled color");
        });
    }

    // ── Periodic ─────────────────────────────────────────────────────────────

    @Override
    public void periodic() {
        //LEDPattern m_rainbow = LEDPattern.rainbow(255, 10);
        // Scroll pattern 
        if (scroll){
            ledPattern = ledPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.003), LED_SPACING);
            ledPattern.applyTo(m_ledBuffer);
            // Set the LEDs
            m_led.setData(m_ledBuffer);
        }
    }
}

    
