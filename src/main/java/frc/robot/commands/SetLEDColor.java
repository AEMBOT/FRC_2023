package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LEDSubsystem;

public class SetLEDColor extends CommandBase{
    private final LEDSubsystem m_LedSubsystem;
    private final int[] m_Color;
    
    public SetLEDColor(LEDSubsystem ledSubsystem, int[] Color){
        m_LedSubsystem = ledSubsystem;
        m_Color = Color;
    };

    @Override
    public void initialize(){
        m_LedSubsystem.rainbowIsNo();    
    }

   @Override
   public void execute(){
        m_LedSubsystem.setColor(m_Color);
   }

   @Override
   public void end(boolean interrupted){
        m_LedSubsystem.rainbowIsYes();
   }
}
