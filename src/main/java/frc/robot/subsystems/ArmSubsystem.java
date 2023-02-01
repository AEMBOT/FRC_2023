package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
   
    // Elevator
    private CANSparkMax m_angleMotor = new CANSparkMax(Constants.ArmConstants.angleMotorCanID, MotorType.kBrushless);
    private CANSparkMax m_extendMotor = new CANSparkMax(Constants.ArmConstants.extendMotorCanID, MotorType.kBrushless);
    
    RelativeEncoder angleEncoder = m_angleMotor.getEncoder();
    RelativeEncoder extendEncoder = m_extendMotor.getEncoder();

    LinearFilter filter = LinearFilter.movingAverage(Constants.ArmConstants.movingAverage);

    @Override
    public void periodic() {
        //periodic method (called every 1/60th of a second)
        SmartDashboard.putNumber("angleEncoder", angleEncoder.getPosition());
        SmartDashboard.putNumber("extendEncoder", extendEncoder.getPosition());
        SmartDashboard.putNumber("angleMotor", m_angleMotor.get());
        SmartDashboard.putNumber("extendMotor", m_extendMotor.get());
        SmartDashboard.putNumber("ExtendMotorCurrent", m_extendMotor.getOutputCurrent());
        SmartDashboard.putNumber("AngleMotorCurrent", m_angleMotor.getOutputCurrent());
        SmartDashboard.putNumber("ExtendMotorOutput", m_extendMotor.getAppliedOutput());
        SmartDashboard.putNumber("AngleMotorOutput", m_angleMotor.getAppliedOutput());
    }

    public ArmSubsystem() {
        // Restore motors to factory defaults for settings to be consistent
        m_angleMotor.restoreFactoryDefaults();
        m_extendMotor.restoreFactoryDefaults();
        
        // Lift shouldn't drift, so set it to brake mode
        m_extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set max current the extend motor can draw
        m_extendMotor.setSmartCurrentLimit(Constants.ArmConstants.extendMotorCurrentLimit);
        m_angleMotor.setSmartCurrentLimit(Constants.ArmConstants.angleMotorCurrentLimit);
    }

    /* 
    public CommandBase angleToPosition(double targetPosition){
        //make this later, after testing so encoder values are available
        //use signum to find sign of difference from target and current position, use that to go up or down to target position
        // return runEnd(() -> Math.signum(diff) * 0.1, () -> stop motor).until(close to or past position);
    }
*/
    public void resetExtendEncoder(){
        extendEncoder.setPosition(0);
    }

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
    
    public void extendArm(){
        m_extendMotor.set(-0.5);
    }

    public void retractArm(){
        m_extendMotor.set(-0.1);
        m_extendMotor.set(0.5);
    }

    public boolean isCurrentLimited(){
        return filter.calculate(m_extendMotor.getOutputCurrent()) >= 25;
    }

    // Get arm to go to position using the extend and angle motors
    public boolean goToPosition(){
        return extendEncoder.getPosition() >= 0.25 && extendEncoder.getPosition() <0.5 && angleEncoder.getPosition() >= 0.25 && angleEncoder.getPosition() <= 0.5;
    }

    // Clamp
    // The solenoid that controls the clamp
    private Solenoid m_clampSolenoid;
    
    // Contructs the clamp with the ports on the PCM
    // 
    // extendPort = port for extending the clamp
    public ArmSubsystem(int extendPort) {
        m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, extendPort);
    }
    
    // Extends the clamp
    public void extendClamp() {
        m_clampSolenoid.set(true);
    }

    // Retracts the clamp
    public void retractClamp() {
        m_clampSolenoid.set(false);
    }
}
