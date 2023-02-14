package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final AddressableLED m_led = new AddressableLED(9);
    private final AddressableLEDBuffer m_ledBuffer = new AddressableLEDBuffer(30);


    public LEDSubsystem() {
        m_led.setLength(m_ledBuffer.getLength());

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            m_ledBuffer.setRGB(i, 127, 127, 127);
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
}
