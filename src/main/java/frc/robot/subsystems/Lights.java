
package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 

public class Lights extends SubsystemBase {
    AddressableLED m_led  = new AddressableLED(1);
    AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(150);
    Distance LED_SPACING = Meters.of(16.0*12*0.0254 / 150);
    int brightness = 20;
    public boolean lightsoff;
    // The below pattern should be periodically applied
    // To create a new pattern commands should just change the pattern
    LEDPattern ledPattern = LEDPattern.solid(Color.kBlue);//LEDPattern.kOff;
    Boolean scroll = false;
     // Reuse buffer
    // Length is expensive to set, so only set it once, then just update data
    
    public Lights() {
        m_led.setLength(m_ledBuffer.getLength());
    }

    public Command red(){       
        return runOnce(
            () -> {
            // Create an LED pattern that sets the entire strip to solid red
            ledPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kGreen, Color.kBlack);  //Red and Green switched 
            ledPattern.atBrightness(Percent.of(brightness));
            scroll = true;
            System.out.println("Red light command");
            });
        }
    
    public Command off(){       
        return runOnce(
            () -> {
            // turn Leds off
            ledPattern = LEDPattern.kOff; 
            scroll = false;
            System.out.println("Red light command");
            });
        }

    public Command blue(){       
        return runOnce(
            () -> {
            // Create an LED pattern that sets the entire strip to solid blue
            ledPattern = LEDPattern.solid(Color.kBlue);
            ledPattern.atBrightness(Percent.of(brightness));
            scroll = false;
            System.out.println("Blue light command");
            });
        }
    
    public Command green(){       
        return runOnce(
            () -> {
            // Create an LED pattern that sets the entire strip to solid green
            ledPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kRed, Color.kBlack);
            ledPattern.atBrightness(Percent.of(brightness));
            scroll = true;
            System.out.println("Green light command");
            });
        }
    public Command pink(){       
        return runOnce(
            () -> {
            // Create an LED pattern that sets the entire strip to solid green
            ledPattern = LEDPattern.solid(Color.kDarkCyan);
            ledPattern.atBrightness(Percent.of(brightness));
            scroll = false;
            System.out.println("Pink light command");
            });
        }

    public Command rainbow(){       
        return runOnce(
            () -> {
            // Create an LED pattern that sets the entire strip to solid green
            ledPattern = LEDPattern.rainbow(255, 40);
            scroll = true;
            System.out.println("Rainbow light command");
            });
        }

    public Command gradient(){
        return runOnce(
            () -> {
                // Create an LED pattern that displays a red-to-blue gradient.
                // The LED strip will be red at one end and blue at the other.
                ledPattern = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kCyan, Color.kBlack);
                ledPattern.atBrightness(Percent.of(brightness));
                System.out.println("Gradient");
                scroll = true;
            }
        );}

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


    @Override
    public void periodic() {
        //LEDPattern m_rainbow = LEDPattern.rainbow(255, 10);
        // Scroll pattern 
        if (scroll){
            ledPattern = ledPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.003), LED_SPACING);
        }
        ledPattern.applyTo(m_ledBuffer);
        // Set the LEDs
        m_led.setData(m_ledBuffer);
        m_led.start();
    }
 
}

    
