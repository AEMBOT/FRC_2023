package frc.robot.commands.docking;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import io.github.oblarg.oblog.Loggable;

public class AutoPathDocking extends CommandBase implements Loggable {

    double pitchUpper = -65;
    double pitchLower = -75;

    private final Limelight m_limelight;

    private final boolean go = true;

    private final LinearFilter rollData = LinearFilter.movingAverage(10);
    private final LinearFilter vel = LinearFilter.movingAverage(10);
    private final AHRS navx = new AHRS(Port.kMXP);

    private boolean stop = false;

    private final DrivebaseS m_drivebase;

    private boolean behindMiddle = false;
    private boolean frontMiddle = false;

    private double appliedSpeed;

    private boolean finished = false;

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
        appliedSpeed = 0;
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

    public double distToMiddle(double distance){
        double robotLimelightX = m_drivebase.getPose().getX();
        return Math.abs(3.8 -robotLimelightX);
    }

    public double DockSpeed(double rollAvg, double rollBegin, double velAvg){
        if (rollBegin > rollAvg){
            return 0.15;
        }
        else if(tilt(navx.getRoll(), navx.getPitch()) > 13){
            return 0;
       }
        else if (velAvg > 6 || velAvg < -5){
            return 0;
        }
        else{
            return 0.25;
        }
    }
/* 
    public boolean isOvershoot(){
        double rollAvg = rollData.calculate(navx.getRoll());
        double velAvg = vel.calculate(navx.getRawGyroY());
        if (navx.getRoll() < 16 && navx.getRoll() > -4 ){
        if (tilt(rollAvg, navx.getPitch()) > 13){
            return true;
        }
        else{
            return false;
        }
    }*/

    @Override
    public void execute() {
       // double appliedSpeed  = 0.05* (navx.getRoll() + 12)  + 0.0025 * navx.getRawGyroY();
        double appliedSpeed  = 0.1* (navx.getRoll() + 12);
        if (Math.abs(appliedSpeed) > 0.5){
            appliedSpeed = 0.5 * Math.signum(appliedSpeed);   
        }
        if (navx.getRoll() < 12 && navx.getRoll() > -12){
            appliedSpeed = 0;
        }
        if (tilt(navx.getRoll(), navx.getPitch()) < 5){
            appliedSpeed =0;
        }
        m_drivebase.drive(new ChassisSpeeds(appliedSpeed,0,0));

        /* 
        //get position of robot
        //dock accordingly
        //double robotLimelightX = m_drivebase.getroll().getX();
        //-16
        //14
        double rollAvg = rollData.calculate(navx.getRoll());
        double velAvg = vel.calculate(navx.getRawGyroY());
        if (navx.getRoll() < 16 && navx.getRoll() > -4 ){
            behindMiddle = true;
        }
        if (navx.getRoll() < -8){
            frontMiddle = true;
        }
        double currentRawAngle = navx.getRawGyroY();
        double yaw = navx.getYaw();
        double angle = navx.getRoll();
        double kv = navx.getRawGyroY();
        double rollStart = 0;

        if (behindMiddle){
            
            if (yaw > -45 && yaw < 45){
                //facing forward
                rollStart = 13;
                m_drivebase.driveFieldRelative(new ChassisSpeeds(DockSpeed(navx.getRoll(), rollStart, velAvg),0,0));
            }
            if (yaw > -135 && yaw < 135){
                //facing backward
                rollStart = -14.3;
                m_drivebase.driveFieldRelative(new ChassisSpeeds(DockSpeed(navx.getRoll(),rollStart,velAvg),0,0));
            }
            if (yaw > -135 && yaw < -45){
                //facing left
                rollStart = 10;
                m_drivebase.drive(new ChassisSpeeds(DockSpeed(navx.getRoll(), rollStart, velAvg), 0,0));
            }
            if (yaw < 135 && yaw < 45){
                //facing right
                rollStart = 10;
                m_drivebase.drive(new ChassisSpeeds(DockSpeed(navx.getRoll(), rollStart, velAvg), 0,0));
            }
        }
         
        if (frontMiddle){  
            if (yaw > -135 && yaw < 135){
                //facing forward 
                rollStart = -16;
                m_drivebase.drive(new ChassisSpeeds(-DockSpeed(navx.getRoll(), rollStart, velAvg),0,0));
            }
            if (yaw > -45 && yaw < 45){
                //facing backward
                rollStart = 13;
                m_drivebase.drive(new ChassisSpeeds(-DockSpeed(navx.getRoll(), rollStart, velAvg),0,0));
            }
            if (yaw > -135 && yaw < -45){
                //facing left
                rollStart = 10;
                m_drivebase.drive(new ChassisSpeeds(-DockSpeed(navx.getRoll(), rollStart, velAvg), 0,0));
            }
            if (yaw < 135 && yaw < 45){
                //facing right
                rollStart = 10;
                m_drivebase.drive(new ChassisSpeeds(-DockSpeed(navx.getRoll(), rollStart, velAvg), 0,0));
            }
            }*/
        }
    /* 
    @Override
    public boolean isFinished() {
        // TODO Auto-generated method 
        
        frontMiddle = false;
        behindMiddle = false;
        double rollAvg = rollData.calculate(navx.getRoll());
        double velAvg = vel.calculate(navx.getRawGyroY());
        return (m_drivebase.getFieldRelativeLinearSpeedsMPS().getX() > -0.3 && m_drivebase.getFieldRelativeLinearSpeedsMPS().getX() < .3) 
         && (navx.getRoll() > -13 && navx.getRoll() < 8);
         //&& (tilt(rollAvg, velAvg) > -2);
    }*/
    @Override
    public void end(boolean _interrupted) {
        frontMiddle = false;
        behindMiddle = false;
        /* 
        if (Math.abs(navx.getPitch()) > pitchUpper){
            m_drivebase.arcadeDrive(0,0, false);
        }*/
    }
}