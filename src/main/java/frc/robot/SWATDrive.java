package frc.robot;

// import libraries
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SWATDrive {

    //slow variable
    private boolean slow = false;
    //low gear variable
    private static boolean lowGear = true;

    private double maxTurnSpeed;
    private double minTurnSpeed = 0.3;
    private double minStraightSpeed = 0.3;
    private double overrideMaxTurnSpeed = 1.0;
    private double maxStraightSpeed;
    private double overrideMaxSpeed = 1.0;
    private RobotMap robotMap;
    
    public SWATDrive(RobotMap roboMap) {
        robotMap = roboMap;
        maxStraightSpeed = 1;
        maxTurnSpeed = 0.8;
        SmartDashboard.putNumber("Max Turn Speed", maxTurnSpeed);
        SmartDashboard.putNumber("Max Speed", maxStraightSpeed);
    }
    public void gearShift() {
        lowGear = !lowGear;
        if(lowGear) {
            robotMap.getGearShift().set(DoubleSolenoid.Value.kReverse);
        }
        else {
            robotMap.getGearShift().set(DoubleSolenoid.Value.kForward);
        }
    }
    public void slow() {
        // slow = !slow;
        // if(slow) {
        //     robotMap.getDriveTrain().setMaxOutput(0.7);
        //     maxTurnSpeed = 0.7;
        //     maxStraightSpeed = 0.7;
        // }
        // else {
        //     robotMap.getDriveTrain().setMaxOutput(1);
        //     maxTurnSpeed = 0.8;
        //     maxStraightSpeed = 1;
        // }

    }
    public void updateFromShuffleData(){
        maxTurnSpeed = SmartDashboard.getNumber("Max Turn Speed", maxTurnSpeed);
        maxStraightSpeed = SmartDashboard.getNumber("Max Speed", maxStraightSpeed);
    }

    public double getMaxTurnSpeed() {
        return ((maxTurnSpeed * overrideMaxTurnSpeed) + minTurnSpeed) * (1.0 - minTurnSpeed);
    }
    public double getMaxStraightSpeed() {
        return ((maxStraightSpeed * overrideMaxSpeed) + minStraightSpeed) * (1.0 - minStraightSpeed);
    }

    public void arcadeDrive(double speed, double rotation) {
        robotMap.getDriveTrain().arcadeDrive(speed, rotation);
    }

    public void tankDrive(double lSpeed, double rSpeed) {
        robotMap.getDriveTrain().tankDrive(lSpeed, rSpeed);
    }
}
  