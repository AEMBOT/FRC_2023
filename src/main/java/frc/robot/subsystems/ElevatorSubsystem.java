package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase{
    private static final int deviceIDa = 1; //get IDs
    private static final int deviceIDb = 2; //get IDs
    
    private CANSparkMax m_angleMotor = new CANSparkMax(deviceIDa, MotorType.kBrushless);
    private CANSparkMax m_extendMotor = new CANSparkMax(deviceIDb, MotorType.kBrushless);
    
    RelativeEncoder angleEncoder = m_angleMotor.getEncoder();
    RelativeEncoder extendEncoder = m_extendMotor.getEncoder();

    @Override
    public void periodic() {
        //periodic method (called every 1/60th of a second)
        SmartDashboard.putNumber("angleEncoder", angleEncoder.getPosition());
        SmartDashboard.putNumber("extendEncoder", angleEncoder.getPosition());
    }

    /* 
    public CommandBase angleToPosition(double targetPosition){
        //make this later, after testing so encoder values are available
        //use signum to find sign of difference from target and current position, use that to go up or down to target position
        // return runEnd(() -> Math.signum(diff) * 0.1, () -> stop motor).until(close to or past position);
    }
*/
    public void angleUp(CANSparkMax m_angleMotor){
        m_angleMotor.set(0.1);
    }

    public void angleDown(CANSparkMax m_angleMotor){
        m_angleMotor.set(-0.1);
    }
    
    public void extend(CANSparkMax m_extendMotor){
        m_extendMotor.set(0.1);
    }

    public void retract(CANSparkMax m_extendMotor){
        m_extendMotor.set(-0.1);
    }
}
