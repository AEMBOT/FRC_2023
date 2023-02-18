package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import static frc.robot.Constants.ArmConstants.*;

public class ArmSubsystem extends SubsystemBase implements Loggable {

    // Elevator
    private final CANSparkMax m_angleMotor = new CANSparkMax(angleMotorCanID, MotorType.kBrushless);
    private final CANSparkMax m_extendMotor = new CANSparkMax(extendMotorCanID, MotorType.kBrushless);
    private final Solenoid m_clampSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, clampSolenoidID);

    public RelativeEncoder angleEncoder = m_angleMotor.getEncoder();
    public RelativeEncoder extendEncoder = m_extendMotor.getEncoder();
    public DutyCycleEncoder absoluteAngleEncoder = new DutyCycleEncoder(angleEncoderPort);
    public Encoder relativeAngleEncoder = new Encoder(1, 2);
    private Ultrasonic objectSensor = new Ultrasonic(ultrasonicPingPort, ultrasonicEchoPort);
    private boolean activateExtendPID = false; // Activates PID controller, false when zeroing


    LinearFilter filter = LinearFilter.movingAverage(movingAverage);

    @Log
    PIDController pidExtend = new PIDController(582.62, 0, 10.198);
    @Log
    PIDController pidTheta = new PIDController(20, 0, 2);

    ArmFeedforward thetaDown = new ArmFeedforward(0.5, 0.5, 50, 0);
    ArmFeedforward thetaUp = new ArmFeedforward(-0.5, 0.5, 40, 0);


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

      /*  if(isGamePieceThere()){
            m_clampSolenoid.set(false);

        }

*/
        if (activateExtendPID) {
            m_extendMotor.setVoltage(Math.min(pidExtend.calculate(extendEncoder.getPosition()), 1));
            m_angleMotor.setVoltage(
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
//        SmartDashboard.putNumber("thetaGoal", pidTheta.getGoal().position);
//        SmartDashboard.putNumber("thetaSetpointPos", pidTheta.getSetpoint().position);
//        SmartDashboard.putNumber("thetaSetpointVel", pidTheta.getSetpoint().velocity);

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

        m_extendMotor.setInverted(true);
        m_angleMotor.setInverted(true);

        extendEncoder.setPositionConversionFactor(extendTickToMeter);
        absoluteAngleEncoder.setPositionOffset(0.346);
        relativeAngleEncoder.setDistancePerPulse(2 * Math.PI / 8192.0);

        pidExtend.setSetpoint(0);
        pidTheta.setSetpoint(0);
    }

    public boolean isGamePieceThere(){
        return objectSensor.getRangeInches() <=4;
    }


    public void resetExtendEncoder(){
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

    public void stopAngle() {
        m_angleMotor.set(0);
    }

    public void stopExtend() {
        m_extendMotor.set(0);
    }

    public void angleUp() {
        m_angleMotor.setVoltage(7.0);
    }

    public double getAnglePosition() {
        return (absoluteAngleEncoder.getAbsolutePosition() - absoluteAngleEncoder.getPositionOffset()) * Math.PI * 2;
    }

    public void angleDown() {
        m_angleMotor.setVoltage(-7.0);
    }

    public void extendArm() {
        m_extendMotor.set(0.5);
    }

    public void retractArm() {
        m_extendMotor.set(-0.5);
    }

    public double getExtendPosition() {
        return extendEncoder.getPosition();
    }

    public boolean isCurrentLimited() {
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

    public void setExtendPIDState(boolean ready) {
        activateExtendPID = ready;
    }

}
