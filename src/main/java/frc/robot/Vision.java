package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision {

    NetworkTable table;
    NetworkTableEntry tv;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry ledMode;
    NetworkTableEntry camMode;

    public Vision(){
        table = NetworkTableInstance.getDefault().getTable("limelight-swat");
        tv = table.getEntry("tv");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ledMode =  table.getEntry("ledMode");
        camMode = table.getEntry("camMode");
        
    }
    
    public double getTv(){
        // Return Target Valid Status
        return tv.getDouble(2);
    }

    public double getTx(){
        // Return Target X
        return tx.getDouble(0);
        //return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    }

    public double getTy(){
        // Return Target Y
        return ty.getDouble(0);
    }

    public double getTa(){
        // Return Target Area of Image
        return ta.getDouble(0);
    }

    public void ledDefault(){
        // Set LED to Pipeline Default
        ledMode.setDouble(0);
    }

    public void ledOn(){
        // Force LED to ON
        ledMode.setDouble(3);
    }

    public void ledOff(){
        // Force LED to OFF
        ledMode.setDouble(1);
    }

    public void ledStrobe(){
        // Force LED to Strobe
        ledMode.setDouble(2);
    }

    public void setDriverMode(){
        // Set Camera to Driver Mode
        camMode.setDouble(1);
    }

    public void setVisionMode(){
        // Set Camera to Vision Mode
        camMode.setDouble(0);
    }
        
}
