package frc.robot.subsystems;

import java.nio.Buffer;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(30);
    public boolean rainbowYes = true;
    //how many pixels before it goes full bright again
    int DarkPix = 6;

   
    //declare the red, green, and blue values, and declare the SubtractVal array.
    int red;
    int green;
    int blue;
    int[] SubtractVal = {0, 0, 0};


    public LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());
        m_led.start();

    }

    public void setColor(int[] Color){
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            //set all of the subtraction values to a mapped value
            SubtractVal[0] = map((i % DarkPix), 0, (DarkPix), 0, Color[0]);
            SubtractVal[1] = map((i % DarkPix), 0, (DarkPix), 0, Color[1]);
            SubtractVal[2] = map((i % DarkPix), 0, (DarkPix), 0, Color[2]);
            //set the modified red, green, and blue values
            red = Math.round(Color[0] - SubtractVal[0]);
            green = Math.round(Color[1] - SubtractVal[1]);
            blue = Math.round(Color[2] - SubtractVal[2]);
            // Sets the specified LED to the RGB values
            m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
    }


    public void rainbow(){
        int m_rainbowFirstPixelHue = 2;
        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Calculate the hue - hue is easier for rainbows because the color
            // shape is a circle so only one value needs to precess
            final var hue = (m_rainbowFirstPixelHue + (i * 180 / m_ledBuffer.getLength())) % 180;
            // Set the value
            m_ledBuffer.setHSV(i, hue, 255, 128);
          }
          // Increase by to make the rainbow "move"
          m_rainbowFirstPixelHue += 3;
          // Check bounds
          m_rainbowFirstPixelHue %= 180;
        m_led.setData(m_ledBuffer);
    }

    public void rainbowIsNo(){
        rainbowYes = false;
    }

    public void rainbowIsYes(){
        rainbowYes = true;
    }

    @Override
    public void periodic() {
        if(rainbowYes){
            rainbow();
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private int map(int x, int in_min, int in_max, int out_min, int out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
