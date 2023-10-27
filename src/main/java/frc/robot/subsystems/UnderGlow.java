
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UnderGlow extends SubsystemBase {
    private AddressableLED m_led;
    private AddressableLEDBuffer m_ledBuffer;

    public UnderGlow() {
        m_led = new AddressableLED(0);

        // Reuse buffer
        // Default to a length of 60, start empty output
        // Length is expensive to set, so only set it once, then just update data
        m_ledBuffer = new AddressableLEDBuffer(65);
        m_led.setLength(m_ledBuffer.getLength());
    
        // Set the data
        m_led.setData(m_ledBuffer);
        m_led.start();

        
    }

    private int m_rainbowFirstPixelHue = 3;
    
    public void rainbow() {
        for (var i=0;i<m_ledBuffer.getLength(); i++){
            final var hue = (m_rainbowFirstPixelHue + (i*180 / m_ledBuffer.getLength())) % 180;
            m_ledBuffer.setHSV(i,hue,255,128);
        }
        m_rainbowFirstPixelHue +=3;
        m_rainbowFirstPixelHue %=180;
        

        m_led.setData(m_ledBuffer);
        m_led.start();
    }
    
}
