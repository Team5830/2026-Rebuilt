
package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    private final AddressableLED       m_led       = new AddressableLED(1);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
    private final Distance             LED_SPACING = Meters.of(16.0 * 12 * 0.0254 / 150);

    private static final int BRIGHTNESS = 20;

    // Current active pattern and scroll flag
    private LEDPattern currentPattern = LEDPattern.solid(Color.kBlue);
    private boolean    scroll         = false;

    // Toggle state — tracks the last color set by each toggle command
    private boolean redOn     = false;
    private boolean blueOn    = true;  // default on at boot
    private boolean greenOn   = false;
    private boolean rainbowOn = false;

    public Lights() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();
    }

    // ── Internal helpers ──────────────────────────────────────────────────────

    private void setPattern(LEDPattern pattern, boolean scrolling) {
        currentPattern = pattern.atBrightness(Percent.of(BRIGHTNESS));
        scroll = scrolling;
    }

    // ── Individual color commands ─────────────────────────────────────────────

    public Command red() {
        return runOnce(() -> {
            setPattern(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack), true);
            System.out.println("Lights: red");
        });
    }

    public Command off() {
        return runOnce(() -> {
            currentPattern = LEDPattern.kOff;
            scroll = false;
            System.out.println("Lights: off");
        });
    }

    public Command blue() {
        return runOnce(() -> {
            setPattern(LEDPattern.solid(Color.kBlue), false);
            System.out.println("Lights: blue");
        });
    }

    public Command green() {
        return runOnce(() -> {
            setPattern(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlack), true);
            System.out.println("Lights: green");
        });
    }

    public Command pink() {
        return runOnce(() -> {
            setPattern(LEDPattern.solid(Color.kDarkCyan), false);
            System.out.println("Lights: pink");
        });
    }

    public Command rainbow() {
        return runOnce(() -> {
            setPattern(LEDPattern.rainbow(255, 40), true);
            System.out.println("Lights: rainbow");
        });
    }

    public Command gradient() {
        return runOnce(() -> {
            setPattern(LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCyan, Color.kBlack), true);
            System.out.println("Lights: gradient");
        });
    }

    public Command pbar(int count) {
        return runOnce(() -> {
            setPattern(LEDPattern.progressMaskLayer(() -> count / 10.0), false);
        });
    }

    // ── Toggle commands ───────────────────────────────────────────────────────

    /**
     * Toggle red lights on/off. Turns off (LEDPattern.kOff) when toggled off.
     */
    public Command toggleRed() {
        return runOnce(() -> {
            redOn = !redOn;
            if (redOn) {
                red().schedule();
            } else {
                off().schedule();
            }
            System.out.println("Lights: red toggled " + (redOn ? "ON" : "OFF"));
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
        LEDPattern patternToApply = scroll
            ? currentPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.003), LED_SPACING)
            : currentPattern;
        patternToApply.applyTo(m_ledBuffer);
        m_led.setData(m_ledBuffer);
    }
}

    
