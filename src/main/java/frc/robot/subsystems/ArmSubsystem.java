package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.SparkMaxAbsoluteEncoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.util.sim.SparkMaxEncoderWrapper;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase {
   
    // Elevator
    private CANSparkMax m_angleMotor = new CANSparkMax(angleMotorCanID, MotorType.kBrushless);
    private CANSparkMax m_extendMotor = new CANSparkMax(extendMotorCanID, MotorType.kBrushless);
    private Solenoid m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clampSolenoidID);
    
    public RelativeEncoder angleEncoder = m_angleMotor.getEncoder();
    public RelativeEncoder extendEncoder = m_extendMotor.getEncoder();
    public DutyCycleEncoder absoluteAngleEncoder = new DutyCycleEncoder(angleEncoderPort);
    private double rawAngle;

    LinearFilter filter = LinearFilter.movingAverage(movingAverage);

    PIDController pidExtend = new PIDController(582.62, 0, 10.198);

    @Override
    public void periodic() {
        //periodic method (called every 1/60th of a second)
        SmartDashboard.putNumber("extendEncoder", extendEncoder.getPosition());
        SmartDashboard.putNumber("angleMotor", m_angleMotor.get());
        SmartDashboard.putNumber("extendMotor", m_extendMotor.get());
        SmartDashboard.putNumber("ExtendMotorCurrent", m_extendMotor.getOutputCurrent());
        SmartDashboard.putNumber("AngleMotorCurrent", m_angleMotor.getOutputCurrent());
        SmartDashboard.putNumber("ExtendMotorOutput", m_extendMotor.getAppliedOutput());
        SmartDashboard.putNumber("AngleMotorOutput", m_angleMotor.getAppliedOutput());
        rawAngle = absoluteAngleEncoder.getAbsolutePosition() - angleEncoderOffset;
        SmartDashboard.putNumber("absoluteAngleEncoder", rawAngle);

        m_extendMotor.setVoltage(Math.max(pidExtend.calculate(extendEncoder.getPosition()), 3));

    }

    public ArmSubsystem() {
        // Restore motors to factory defaults for settings to be consistent
        m_angleMotor.restoreFactoryDefaults();
        m_extendMotor.restoreFactoryDefaults();
        
        // Lift shouldn't drift, so set it to brake mode
        m_extendMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_angleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Set max current the extend motor can draw
        m_extendMotor.setSmartCurrentLimit(extendMotorCurrentLimit);
        m_angleMotor.setSmartCurrentLimit(angleMotorCurrentLimit);

        extendEncoder.setPositionConversionFactor(Constants.ArmConstants.extendTickToMeter);
    }

    public void resetExtendEncoder(){
        extendEncoder.setPosition(0);
    }

    public void setExtendMeter(double positionMeters){
        pidExtend.setSetpoint(positionMeters);
    }

    public void stopAngle(){
        m_angleMotor.set(0);
    }

    public void stopExtend(){
        m_extendMotor.set(0);
    }

    public void angleUp(){
        m_angleMotor.set(0.7);
    }

    public double getAnglePosition(){
        return absoluteAngleEncoder.getAbsolutePosition();
    }

    public void angleDown(){
        m_angleMotor.set(-0.7);
    }
    
    public void extendArm(){
        m_extendMotor.set(-0.5);
    }

    public void retractArm(){
        m_extendMotor.set(0.5);
    }

    public double getExtendPosition(){
        return extendEncoder.getPosition();
    }

    public boolean isCurrentLimited(){
        return filter.calculate(m_extendMotor.getOutputCurrent()) >= 35;
    }

    // Extends the clamp
    public void extendClamp() {
        m_clampSolenoid.set(true);
    }

    // Retracts the clamp
    public void retractClamp() {
        m_clampSolenoid.set(false);
    }

    // Toggles the clamp
    public void toggleClamp() {
        m_clampSolenoid.set(!m_clampSolenoid.get());
    }

}
