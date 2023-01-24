package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// This is the subsystem that controls the clamp
public class ClampSubsystem extends SubsystemBase {
   
    // The solenoid that controls the clamp
    private Solenoid m_clampSolenoid;

    // Creates a new clamp subsystem
    public ClampSubsystem() {}
    
    // Contructs the clamp with the ports on the PCM
    // 
    // extendPort = port for extending the clamp
    public ClampSubsystem(int extendPort) {
        m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, extendPort);
    }
    
    // Extends the clamp
    public void extend() {
        m_clampSolenoid.set(true);
    }

    // Retracts the clamp
    public void retract() {
        m_clampSolenoid.set(false);
    }
}
