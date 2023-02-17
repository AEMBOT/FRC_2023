package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(30);


    public LEDSubsystem() {

                
        m_led.setLength(m_ledBuffer.getLength());

        //color codes, color is specified after the list initiation. will add input support soon
        

        //how many pixels before it goes full bright again
        int DarkPix = 6;

        //color codes, color is specified after the array
        int[] color = {64,3,3}; //red
        //int[] color = {0,0,64}; //blue
        //int[] color = {64,32,0}; //yellow
        //int[] color = {64,0,64}; //purple

        //int[] color = {64,36,0}; //orange
        //int[] color = {0,64,0}; //green

        //last 2 so we have a rainbow if nessecary. IDK why we would need one though. Celebration?


        //declare the red, green, and blue values, and declare the SubtractVal array.
        int red;
        int green;
        int blue;
        int[] SubtractVal = {0,0,0};


        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            //set all of the subtraction values to a mapped value
            SubtractVal[0] = map((i%DarkPix),0,(DarkPix),0,color[0]);
            SubtractVal[1] = map((i%DarkPix),0,(DarkPix),0,color[1]);
            SubtractVal[2] = map((i%DarkPix),0,(DarkPix),0,color[2]);
            //set the modified red, green, and blue values
            red = Math.round(color[0]-SubtractVal[0]);
            green = Math.round(color[1]-SubtractVal[1]);
            blue = Math.round(color[2]-SubtractVal[2]);
            // Sets the specified LED to the RGB values
            m_ledBuffer.setRGB(i, red, green, blue);
        }

        m_led.setData(m_ledBuffer);
        m_led.start();

    }


    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    private int map(int x, int in_min, int in_max, int out_min, int out_max) {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
