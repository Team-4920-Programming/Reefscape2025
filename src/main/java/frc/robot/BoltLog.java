package frc.robot;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;

public final class BoltLog {
    private int LogLevel=0;
    public static final int MedLog = 1;
    public  final int HighLog = 0;
    public static final int LowLog = 2;

    /* 
     * High Level - Logs to Log File and Logs message to SmartDashboard
     * Med Level - Logs to Log File Only
     * Low Level - Critical Message to Log File Only
     */

    public void Log(int Level, String Subsystem, String Command, String Function, String Message, String Msgvalue)
    {
        if(Level >= LogLevel)
        {
            DogLog.log(Subsystem+"/"+Command+"/"+Function+"/"+Message, Msgvalue);
        }


    }
    public void Log(int Level, String Subsystem, String Command, String Function, String Message, Boolean Msgvalue)
    {
        if(Level >= LogLevel)
        {
            DogLog.log(Subsystem+"/"+Command+"/"+Function+"/"+Message, Msgvalue);
        }


    }
    public void Log(int Level, String Subsystem, String Command, String Function, String Message, Double Msgvalue)
    {
        if(Level >= LogLevel)
        {
            DogLog.log(Subsystem+"/"+Command+"/"+Function+"/"+Message, Msgvalue);
        }


    }
    public void Log(int Level, String Subsystem, String Command, String Function, String Message, int Msgvalue)
    {
        if(Level >= LogLevel)
        {
            DogLog.log(Subsystem+"/"+Command+"/"+Function+"/"+Message, Msgvalue);
        }


    }
    public void Log(int Level, String Subsystem, String Command, String Function, String Message, Pose3d Msgvalue)
    {
        if(Level >= LogLevel)
        {
            DogLog.log(Subsystem+"/"+Command+"/"+Function+"/"+Message, Msgvalue);
        }


    }
    public void setLogLevelHigh(){
        LogLevel = 0;
    }
    public void setLogLevelMed(){
        LogLevel = 1;
    }
    public void setLogLevelLow(){
        LogLevel = 2;
    }

}


