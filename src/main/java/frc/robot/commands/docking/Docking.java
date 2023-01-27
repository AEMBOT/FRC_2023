package frc.robot.commands.docking;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;
import com.kauailabs.navx.frc.AHRS;
import frc.robot.subsystems.DrivebaseS;
import io.github.oblarg.oblog.Loggable;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.drivetrain.OperatorControlC;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Docking extends CommandBase implements Loggable{

    double pitchUpper = -65;
    double pitchLower = -75;

    private final AHRS navx = new AHRS(Port.kMXP);

    private final DrivebaseS m_drivebase;

    public Docking(DrivebaseS drive){
        m_drivebase = drive;
        addRequirements(m_drivebase);
    }

    public void brakeRobot(){
        m_drivebase.drive(0,0, 0, false);
    }

    public void startDistanceTracking(){

    }

    @Override
  public void initialize() {
    // Make sure motors are running in brake mode to avoid overshooting
    brakeRobot();
  }
  
    //assuumes intial if(navx.getPitch() > pitchUpper || navx.getPitch() < pitchLower when called
    @Override
    public void execute(){
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
        }
    }

    @Override 
    public void end(boolean _interrupted){
        /* 
        if (Math.abs(navx.getPitch()) > pitchUpper){
            m_drivebase.arcadeDrive(0,0, false);
        }*/

}
}