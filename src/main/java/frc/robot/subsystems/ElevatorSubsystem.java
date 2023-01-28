package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase{
    
    private CANSparkMax m_angleMotor = new CANSparkMax(15, MotorType.kBrushless);
    private CANSparkMax m_extendMotor = new CANSparkMax(14, MotorType.kBrushless);
    
    RelativeEncoder angleEncoder = m_angleMotor.getEncoder();
    RelativeEncoder extendEncoder = m_extendMotor.getEncoder();

    @Override
    public void periodic() {
        //periodic method (called every 1/60th of a second)
        SmartDashboard.putNumber("angleEncoder", angleEncoder.getPosition());
        SmartDashboard.putNumber("extendEncoder", extendEncoder.getPosition());
        SmartDashboard.putNumber("angleMotor", m_angleMotor.get());
        SmartDashboard.putNumber("extendMotor", m_extendMotor.get());
        SmartDashboard.putNumber("ExtendMotorCurrent", m_extendMotor.getOutputCurrent());
        SmartDashboard.putNumber("AngleMotorCurrent", m_angleMotor.getOutputCurrent());
    }

    public ElevatorSubsystem() {
        // Restore motors to factory defaults for settings to be consistent
        m_angleMotor.restoreFactoryDefaults();
        m_extendMotor.restoreFactoryDefaults();
        
        // Lift shouldn't drift, so set it to brake mode
        m_extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set max current the extend motor can draw
        m_extendMotor.setSmartCurrentLimit(25);
        m_angleMotor.setSmartCurrentLimit(25);
    }

    /* 
    public CommandBase angleToPosition(double targetPosition){
        //make this later, after testing so encoder values are available
        //use signum to find sign of difference from target and current position, use that to go up or down to target position
        // return runEnd(() -> Math.signum(diff) * 0.1, () -> stop motor).until(close to or past position);
    }
*/
    public void stopAngle(){
        m_angleMotor.set(0);
    }

    public void stopExtend(){
        m_extendMotor.set(0);
    }

    public void angleUp(){
        m_angleMotor.set(0.5);
    }

    public void angleDown(){
        m_angleMotor.set(-0.5);
    }
    
    public void extend(){
        m_extendMotor.set(-0.5);
    }

    public void retract(){
<<<<<<< HEAD
        m_extendMotor.set(-0.1);
=======
        m_extendMotor.set(0.5);
>>>>>>> 4ec7f694d104f4ae3e84549ddf0a35444cf66f4e
    }
}
