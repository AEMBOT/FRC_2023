package frc.robot.commands.docking;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import io.github.oblarg.oblog.Loggable;

public class AutoPathDocking extends CommandBase implements Loggable {

    double pitchUpper = -65;
    double pitchLower = -75;

    private final Limelight m_limelight;

    private final boolean go = true;

    private final LinearFilter rollData = LinearFilter.movingAverage(20);
    private final LinearFilter vel = LinearFilter.movingAverage(20);
    private final AHRS navx = new AHRS(Port.kMXP);

    private final boolean stop = false;

    private final DrivebaseS m_drivebase;

    private boolean behindMiddle = false;
    private boolean frontMiddle = false;

    public AutoPathDocking(DrivebaseS drive, Limelight limelight) {
        m_drivebase = drive;
        m_limelight = limelight;
        addRequirements(m_drivebase);
    }

    public void brakeRobot() {
        m_drivebase.drive(0, 0, 0, false);
    }

    public static double tilt(double roll, double pitch) {
        double cy = (Math.cos(Units.degreesToRadians(pitch)));
        double sx = (Math.sin(Units.degreesToRadians(roll)));
        double sy = (Math.sin(Units.degreesToRadians(pitch)));

        double a = Units.radiansToDegrees(Math.asin(Math.sqrt(Math.pow(sy, 2) + Math.pow(sx, 2) * Math.pow(cy, 2))));
        return a;
    }

    //public double distanceTrakcer(double )

    @Override
    public void initialize() {
        // Make sure motors are running in brake mode to avoid overshooting
        brakeRobot();
    }

    public double decrementSpeed(double initSpeed, double distance, double difDistance) {
        //decrease given odometry distance
        //for increae in distance by 0.05 meters (5 cm), decreae speed by 1 m/s
        for (double i = difDistance; i <= distance; i += 0.05) {
            initSpeed = 1 / initSpeed;
        }
        return initSpeed;
    }

    public double omegaToDriveSpeed(double omega) {
        return (omega / 10);
    }

    public double distToMiddle(double distance) {
        double robotLimelightX = m_drivebase.getPose().getX();
        return Math.abs(3.8 - robotLimelightX);
    }

    //assuumes intial if(navx.getPitch() > pitchUpper || navx.getPitch() < pitchLower when called
    @Override
    public void execute() {
        //get position of robot
        //dock accordingly
        //double robotLimelightX = m_drivebase.getPose().getX();
        //-16
        //14
        double poseAvg = rollData.calculate(navx.getRoll());
        double velAvg = vel.calculate(navx.getRawGyroY());
        if (poseAvg < 15 && poseAvg > -4) {
            behindMiddle = true;
        }
        if (poseAvg < -8) {
            frontMiddle = true;
        }
        double currentRawAngle = navx.getRawGyroY();
        double angle = navx.getRoll();
        double kv = navx.getRawGyroY();
        if (poseAvg > 12.7) {
            m_drivebase.drive(new ChassisSpeeds(0, 0, 0));
        } else if (velAvg > 0) {
            m_drivebase.drive(new ChassisSpeeds(0, 0, 0));
        } else {
            m_drivebase.drive(new ChassisSpeeds(0.3, 0, 0));
        }
        /* 
        if (behindMiddle){
            if (poseAvg > 12.8){
                m_drivebase.drive(new ChassisSpeeds(0,0,0));
            }
            else if (velAvg < -8){
                m_drivebase.drive(new ChassisSpeeds(0,0,0));
            }
            else{
                SimpleMotorFeedforward angleFeedForward = new SimpleMotorFeedforward(0.5, 1, 0.6);
                double deltaDrive = angleFeedForward.calculate(kv, currentRawAngle);
                if (Math.abs(deltaDrive) > 0.3) {
                    deltaDrive = Math.copySign(0.3, deltaDrive);
                m_drivebase.drive(new ChassisSpeeds(0.3,0,0));
            }
        }
        if (frontMiddle){
            if (currentRawAngle > 10){
                m_drivebase.drive(new ChassisSpeeds(0,0,0));
            }
            if (angle < 3 && angle > -3){
                m_drivebase.drive(new ChassisSpeeds(0,0,0));
            }
            else{
                SimpleMotorFeedforward angleFeedForward = new SimpleMotorFeedforward(0.5, 1, 0.6);
                double deltaDrive = angleFeedForward.calculate(kv, currentRawAngle);
                if (Math.abs(deltaDrive) > 0.3) {
                    deltaDrive = Math.copySign(0.3, deltaDrive);
                }
                m_drivebase.drive(new ChassisSpeeds(-0.3,0,0));
            }
        }*/

    }


    @Override
    public void end(boolean _interrupted) {
        /* 
        if (Math.abs(navx.getPitch()) > pitchUpper){
            m_drivebase.arcadeDrive(0,0, false);
        }*/

    }
}


