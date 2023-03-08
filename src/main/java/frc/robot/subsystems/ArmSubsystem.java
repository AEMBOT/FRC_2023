package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.arm.GoToPosition;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;


public class ArmSubsystem extends SubsystemBase implements Loggable {

    // Elevator
    private final CANSparkMax m_angleMotor = new CANSparkMax(angleMotorCanID, MotorType.kBrushless);
    private final CANSparkMax m_extendMotor = new CANSparkMax(extendMotorCanID, MotorType.kBrushless);

    public RelativeEncoder angleEncoder = m_angleMotor.getEncoder();
    public RelativeEncoder extendEncoder = m_extendMotor.getEncoder();
    public DutyCycleEncoder absoluteAngleEncoder = new DutyCycleEncoder(angleEncoderPort);
    public Encoder relativeAngleEncoder = new Encoder(2, 1);
    @Log
    private boolean activateExtendPID = false; // Activates PID controller, false when zeroing
    @Log
    private boolean extendZeroed = false;


    LinearFilter filter = LinearFilter.movingAverage(movingAverage);

    @Log
    PIDController pidExtend = new PIDController(120, 0, 2); // p = 582.62, d = 10.198
    @Log
    PIDController pidTheta = new PIDController(150, 0, 2);

    ArmFeedforward thetaDown = new ArmFeedforward(0.5, 0.5, 50, 0);
    ArmFeedforward thetaUp = new ArmFeedforward(-0.5, 0.5, 40, 0);

//    SlewRateLimiter thetaVelocity = new SlewRateLimiter(10.0, -5.0, 0);


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
        SmartDashboard.putNumber("absoluteAngleEncoder", getAnglePosition());
        SmartDashboard.putNumber("ExtendMotorEncoder", extendEncoder.getPosition());
        SmartDashboard.putNumber("AngleMotorEncoder", angleEncoder.getPosition());
        SmartDashboard.putNumber("relativeAngleEncoderPosition", relativeAngleEncoder.getDistance());
        SmartDashboard.putNumber("relativeAngleEncoderVelocity", relativeAngleEncoder.getRate());


        double thetaDownFeedforward = thetaDown.calculate(pidTheta.getSetpoint(), 0);
        double thetaUpFeedforward = thetaUp.calculate(pidTheta.getSetpoint(), 0);
        double pidThetaValue = pidTheta.calculate(getAnglePosition());
        double pidExtendValue = pidExtend.calculate(extendEncoder.getPosition());

      /*  if(isGamePieceThere()){
            m_clampSolenoid.set(false);

        }

*/
        if (activateExtendPID) {
            setExtendMotorVoltage(pidExtendValue);
            setAngleMotorVoltage(
                    (pidTheta.getSetpoint() > getAnglePosition() ?
                            thetaDownFeedforward :
                            thetaUpFeedforward
                    ) +
                            pidThetaValue
            );
        }


        SmartDashboard.putNumber("thetaDownFeedForward", thetaDownFeedforward);
        SmartDashboard.putNumber("thetaUpFeedForward", thetaUpFeedforward);
        SmartDashboard.putNumber("pidThetaValue", pidThetaValue);
        SmartDashboard.putNumber("pidExtendValue", pidExtendValue);

        /*
        if (Math.abs(m_angleMotor.get()) < 0.001){
            extendRatchet();
        }
        else{
            retractRatchet();
        }*/
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

        m_extendMotor.setInverted(false);
        m_angleMotor.setInverted(false);
       

        extendEncoder.setPositionConversionFactor(extendMetersPerTick);
        absoluteAngleEncoder.setPositionOffset(0.049);
        relativeAngleEncoder.setDistancePerPulse(2 * Math.PI / 8192.0);

        pidExtend.setSetpoint(0);
        pidTheta.setSetpoint(0);

        pidExtend.setTolerance(0.01);
        pidTheta.setTolerance(0.01);
    }

    public void resetExtendEncoder() {
        extendEncoder.setPosition(0);
    }

    public void setExtendMeter(double positionMeters) {
        pidExtend.setSetpoint(positionMeters);
    }

    public void setTheta(double positionRadians) {
        pidTheta.setSetpoint(positionRadians);
    }

    public void setArmPosition(double r, double theta) {
        pidExtend.setSetpoint(r);
        pidTheta.setSetpoint(theta);
    }

    private void setAngleMotorVoltage(double voltage) {
//        voltage = thetaVelocity.calculate(voltage);

        if (getAnglePosition() > maxAngleHardStop) {
            m_angleMotor.setVoltage(MathUtil.clamp(voltage, -12, 0));
        } else if (getAnglePosition() < minAngleSoftStop) {
            m_angleMotor.setVoltage(MathUtil.clamp(voltage, 0, 12));
        } else {
            m_angleMotor.setVoltage(voltage);
        }
    }

    private void setExtendMotorVoltage(double voltage) {
        if (getExtendPosition() < minExtendHardStop && extendZeroed) {
            m_extendMotor.setVoltage(MathUtil.clamp(voltage, 0, 12));
        } else if (getExtendPosition() > maxExtendSoftStop && extendZeroed) {
            m_extendMotor.setVoltage(MathUtil.clamp(voltage, -12, 0));
        } else {
            m_extendMotor.setVoltage(voltage);
        }
    }

    public void stopAngle() {
        setAngleMotorVoltage(0);
    }

    public void stopExtend() {
        setExtendMotorVoltage(0);
    }

    public void angleUp() {
        setAngleMotorVoltage(4.0);
    }

    public double getAnglePosition() {
        return (absoluteAngleEncoder.getAbsolutePosition() - absoluteAngleEncoder.getPositionOffset()) * Math.PI * 2;
    }

    public void angleDown() {
        setAngleMotorVoltage(-4.0);
    }

    public void extendArm() {
        setExtendMotorVoltage(3.0);
    }

    public void extendArm(double power) {
        setExtendMotorVoltage(10.0 * power);
    }

    public void retractArm() {
        setExtendMotorVoltage(-3.0);
    }

    public void retractArm(double power) {
        setExtendMotorVoltage(-10.0 * power);
    }

    public double getExtendPosition() {
        return extendEncoder.getPosition();
    }

    public boolean isCurrentLimited() {
        return filter.calculate(m_extendMotor.getOutputCurrent()) >= 35;
    }

    public void setExtendPIDState(boolean ready) {
        activateExtendPID = ready;
    }

    public void setExtendZeroed(boolean zeroed) {
        extendZeroed = zeroed;
    }

    public Command getGoToPositionCommand(double targetExtend, double targetTheta) {
        return new GoToPosition(this, targetExtend, targetTheta);
    }

    public boolean getArmAtPosition() {
        return pidExtend.atSetpoint() && pidTheta.atSetpoint();
    }
    

}
