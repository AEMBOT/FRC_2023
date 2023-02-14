package frc.robot.commands.docking;

import java.util.concurrent.DelayQueue;
import java.util.concurrent.Delayed;

import com.fasterxml.jackson.databind.ser.std.CalendarSerializer;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.DrivebaseS;
import frc.robot.subsystems.Limelight;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.drivetrain.OperatorControlC;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionSubsystem;

public class Docking extends CommandBase implements Loggable{

    double pitchUpper = -65;
    double pitchLower = -75;

    private final Limelight m_limelight; 

    private boolean go = true;

    private final AHRS navx = new AHRS(Port.kMXP);

    private final DrivebaseS m_drivebase;

    public Docking(DrivebaseS drive, Limelight limelight){
        m_drivebase = drive;
        m_limelight = limelight;
        addRequirements(m_drivebase);
    }

    public void brakeRobot(){
        m_drivebase.drive(0,0, 0, false);
    }

    public static double tilt(double roll, double pitch){
        double cy = (Math.cos(Units.degreesToRadians(pitch)));
        double sx = (Math.sin(Units.degreesToRadians(roll)));
        double sy = (Math.sin(Units.degreesToRadians(pitch)));

        double a = Units.radiansToDegrees(Math.asin(Math.sqrt(Math.pow(sy,2) + Math.pow(sx,2) * Math.pow(cy,2))));
        return a;
    }

    //public double distanceTrakcer(double )

    @Override
  public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    brakeRobot();
  }

  public double decrementSpeed(double initSpeed, double distance, double difDistance){
    //decrease given odometry distance
    //for increae in distance by 0.05 meters (5 cm), decreae speed by 1 m/s
    for (double i = difDistance; i <= distance; i += 0.05){
        initSpeed = 1 / initSpeed; 
    }
    return initSpeed;
  }

  public double omegaToDriveSpeed(double omega){
    return (omega / 10);
  }

    //assuumes intial if(navx.getPitch() > pitchUpper || navx.getPitch() < pitchLower when called
    @Override
    public void execute(){
        /*
        if (navx.getPitch() < pitchUpper|| navx.getPitch() > pitchLower){
            //drive certian distance
            //drive until the pitch value is good
            //velocity control 
            //april tag control
            m_drivebase.drive(0.25,0,0, false);
        }
        //drive until the pitch value is good
        if (Math.abs(navx.getPitch()) < pitchUpper){
            m_drivebase.drive(0,0,0,false);
        }*/

        /* 
        Pose2d initPose = m_drivebase.getPose();
        double initPoseX = initPose.getX();
        double initPoseY = initPose.getY();
        double initDist = Math.sqrt(Math.pow(initPoseX,2) + Math.pow(initPoseY,2));
        double initSpeed = 0.6;
        if (tilt(navx.getRoll(), navx.getPitch()) < 3){
            m_drivebase.driveFieldRelative(new ChassisSpeeds(0.2, 0.2,0));
            //m_drivebase.drive(0.2, 0,0, false);
            while (tilt(navx.getRoll(), navx.getPitch()) > 3){
                Pose2d distPose = m_drivebase.getPose();
                double distX = distPose.getX();
                double distY =distPose.getY();
                double dist = Math.sqrt(Math.pow(distX, 2) + Math.pow(distY, 2));
                double distDif = dist - initDist;
                m_drivebase.driveFieldRelative(new ChassisSpeeds(decrementSpeed(initSpeed, initDist, distDif), decrementSpeed(initSpeed, initDist, distDif),0)); 
                 
                if (tilt(navx.getRoll(), navx.getPitch()) < 3){
                    m_drivebase.driveFieldRelative(new ChassisSpeeds(0,0,0));
                    break;
                }
        }*/

        //move robot when robot is farther from middle of the docking station
        //move at decrement speed
        //use tilt to make sure
        
        //Translation2d robotLimelightdist = m_drivebase.getPose().getTranslation()
        //.minus(VisionConstants.TAG_FIELD_LAYOUT.getTagPose(2).get().toPose2d().getTranslation());
        Pose3d robotLimelightdist = m_limelight.getPosition();
        double robotLimelightX =robotLimelightdist.getX();
        SmartDashboard.putNumber("AbiralLook", robotLimelightX);
        double robotLimelighytY = robotLimelightdist.getY();
        double robotLLDist = Math.sqrt(Math.pow(robotLimelightX,2) + Math.pow(robotLimelighytY,2));
        double targetLimelightDist = 2.9083;
        double middleDist = 0.9906; // dist from start of the ramp to middle of the ramp
        double initialLimelightDist = targetLimelightDist + middleDist;
        boolean stop = false;
        double prevTiltValue = tilt(navx.getRoll(), navx.getPitch());

        double AngularY = navx.getRawGyroY();
        double kv  = navx.getVelocityY();
        
        //if (AngularY > 10)
        if (AngularY < -8){
            m_drivebase.drive(new ChassisSpeeds(0,0,0));
            stop = true;
        }
        else{
            m_drivebase.drive(new ChassisSpeeds(0.4,0,0));
        }
        if (AngularY > 10){
            //derivative + feedback control angle
            m_drivebase.drive(new ChassisSpeeds(0,0,0));
            stop = true;
        }
        else{           
            SimpleMotorFeedforward angleFeedForward = new SimpleMotorFeedforward(0.5,1,0.6);
            double deltaDrive = angleFeedForward.calculate(kv, AngularY);
            if (Math.abs(deltaDrive) > 0.3){
                deltaDrive = 0.3;
            }
            m_drivebase.drive(new ChassisSpeeds(-deltaDrive,0,0));
        }
        if (stop){
            m_drivebase.drive(new ChassisSpeeds(0.0,0,0));
        }
        
        //double currTiltValue = tilt(navx.getRoll(), navx.getPitch());
        
       /*  if(robotLimelightX - targetLimelightDist < initialLimelightDist + middleDist){
            
            m_drivebase.drive(new ChassisSpeeds(
                decrementSpeed(0.6, initialLimelightDist + middleDist, robotLimelightX - initialLimelightDist),
                decrementSpeed(0, initialLimelightDist + middleDist, robotLimelightX - initialLimelightDist),0));
        
        
            
            m_drivebase.drive(new ChassisSpeeds(0.4,0,0));
            if (tilt(navx.getRoll(), navx.getPitch()) > 4){
                m_drivebase.drive(new ChassisSpeeds(0.3,0,0));
                if (Math.abs(robotLimelightX) > targetLimelightDist  && tilt(navx.getRoll(), navx.getPitch()) < 12){
                //if (robotLimelightX > -3){
                    m_drivebase.drive(new ChassisSpeeds(0,0,0));
                    stop = true;
                    if (tilt(navx.getRoll(), navx.getPitch()) > 3){
                        while (tilt(navx.getRoll(), navx.getPitch()) > 3 && robotLimelightX < targetLimelightDist ){
                            m_drivebase.drive(new ChassisSpeeds(-0.2,0,0));
                        }
                    }         
            }


        }
            //m_drivebase.drive(new ChassisSpeeds(decrementSpeed(0.2, initialLimelightDist + middleDist, robotLimelightX - initialLimelightDist),0,0));
            if(tilt(navx.getRoll(), navx.getPitch()) < 6){
                m_drivebase.drive(new ChassisSpeeds(0.4,0,0));
        } 
        else{
            m_drivebase.drive(new ChassisSpeeds(0,0,0));
        }
            
            if (go){
                m_drivebase.drive(new ChassisSpeeds(0.3,0,0));
                if (Math.abs(robotLimelightX) > targetLimelightDist - 0.5){
                    go = false;
                } 
            }
            
            //if (Math.abs(robotLimelightX) > targetLimelightDist && tilt(navx.getRoll(), navx.getPitch()) < 3){
                 
            else if(tilt(navx.getRoll(), navx.getPitch()) < 8){
            //if (robotLimelightX > -3){
                m_drivebase.drive(new ChassisSpeeds(0,0,0));
                stop = true;
                go = false;*/
                /* 
                if (tilt(navx.getRoll(), navx.getPitch()) > 3){
                    stop = true;
                    while (tilt(navx.getRoll(), navx.getPitch()) > 3 && robotLimelightX < targetLimelightDist){
                        m_drivebase.drive(new ChassisSpeeds(-0.2,0,0));
                        if (tilt(navx.getRoll(), navx.getPitch()) < 3 && robotLimelightX > targetLimelightDist){
                            stop = true;
                            break;
                        }
                    }
                } 
    }
            

            else{
                m_drivebase.drive(new ChassisSpeeds(0.4,0,0));
            }
            
            if (stop){
                m_drivebase.drive(new ChassisSpeeds(0,0,0));
            }

        

        }
        
       */ 
    }
    @Override 
    public void end(boolean _interrupted){
        /* 
        if (Math.abs(navx.getPitch()) > pitchUpper){
            m_drivebase.arcadeDrive(0,0, false);
        }*/

}
}

