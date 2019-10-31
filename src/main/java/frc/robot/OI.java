package frc.robot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
class OI {

    //Driver Variables
    private XboxController driveStick = new XboxController(0);
    private String driverScheme = "Default";
    private double XSpeed;
    private double ZRotation;
    private boolean YDirection;
    private boolean XDirection;
    private boolean gearShift;
    private boolean slow;
    private double autoClimbPressed;
    private boolean autoClimb;
    private boolean manualControlSandstorm;
    private double manualClimbLift = 0;
    private boolean manualRearUp;
    private boolean manualFrontUp;
    private boolean manualDrive;
    private double manualRearDown;
    private boolean visionButton;

    //Gunner variables
    private Joystick gunnerStick = new Joystick(1);
    private String gunnerScheme = "Default";
    private boolean hatchOpen;
    private boolean hatchClosed;
    private boolean hatchPunchOut;
    private boolean hatchPunchIn;
    private boolean punchBall;
    private boolean secureBall;
    private boolean autoShoot;
    private boolean autoPunch;
    //Special Function variables
    boolean leftTriggerPressed = false;
    boolean rightTriggerPressed = false;
    
    public OI() {

    }
    //periodic function to update controller input
    public void checkInputs() {
        gearShift = driveStick.getXButtonReleased();
        slow = driveStick.getAButtonReleased();
        autoClimbPressed = driveStick.getTriggerAxis(Hand.kRight);
        manualClimbLift = driveStick.getTriggerAxis(Hand.kLeft);
        manualFrontUp = driveStick.getBumper(Hand.kLeft);
        manualRearUp = driveStick.getBumper(Hand.kRight);
        manualDrive = driveStick.getYButton();
        manualControlSandstorm = driveStick.getBButtonReleased();
        manualRearDown = driveStick.getTriggerAxis(Hand.kRight);
        visionButton = driveStick.getBackButtonReleased();

        //switch statement to detirmine controls for the driver
        switch (driverScheme) {
            case "Reverse Turning":
                YDirection = false;
                XDirection = true;
                XSpeed = -driveStick.getY(Hand.kLeft);
                ZRotation = driveStick.getX(Hand.kRight);
                break;    
            default:
                YDirection = true;
                XDirection = false;
                XSpeed = driveStick.getY(Hand.kLeft);
                ZRotation = -driveStick.getX(Hand.kRight);
                break;
        }
        
        //switch statement to detirmine controls for the gunner
        
        switch (gunnerScheme) {
            case "Reverse Hatch":
                hatchOpen = gunnerStick.getRawButtonReleased(6);
                hatchClosed = gunnerStick.getRawButtonReleased(5);
                autoPunch = gunnerStick.getRawButtonReleased(1);
                secureBall = gunnerStick.getRawButtonReleased(7);
                punchBall = gunnerStick.getRawButtonReleased(8);
                autoShoot = gunnerStick.getRawButtonReleased(8);
                break;
            case "Reverse Ball":
                hatchOpen = gunnerStick.getRawButtonReleased(5);
                hatchClosed = gunnerStick.getRawButtonReleased(6);
                autoPunch = gunnerStick.getRawButtonReleased(1);
                secureBall = gunnerStick.getRawButtonReleased(8);
                punchBall = gunnerStick.getRawButtonReleased(7);
                autoShoot = gunnerStick.getRawButtonReleased(7);
                break;
            case "Reverse All":
                hatchOpen = gunnerStick.getRawButtonReleased(6);
                hatchClosed = gunnerStick.getRawButtonReleased(5);
                autoPunch = gunnerStick.getRawButtonReleased(1);
                secureBall = gunnerStick.getRawButtonReleased(8);
                punchBall = gunnerStick.getRawButtonReleased(7);
                autoShoot = gunnerStick.getRawButtonReleased(7);
                break;    
            
            default:
                hatchOpen = gunnerStick.getRawButtonReleased(5);
                hatchClosed = gunnerStick.getRawButtonReleased(6);
                autoPunch = gunnerStick.getRawButtonReleased(1);
                secureBall = gunnerStick.getRawButtonReleased(7);
                punchBall = gunnerStick.getRawButtonReleased(8);
                autoShoot = gunnerStick.getRawButtonReleased(2);
                break;   
        }
        SmartDashboard.putBoolean("Hatch Open", hatchOpen);
        SmartDashboard.putBoolean("Hatch Closed", hatchClosed);
        SmartDashboard.putBoolean("Forward Direction", YDirection);
        SmartDashboard.putBoolean("Turn Direction", XDirection);
        //set button input of trigger for autoClimb
        autoClimb = buttonPressed(autoClimbPressed, "right");

    }
    public void updateFromShuffleData(){
        hatchOpen = SmartDashboard.getBoolean("Hatch Open", hatchOpen);
        YDirection = SmartDashboard.getBoolean("Forward Direction", YDirection);
        XDirection = SmartDashboard.getBoolean("Turn Direction", XDirection);
        hatchClosed = SmartDashboard.getBoolean("Hatch Closed", hatchClosed);
    }

    //Getter functions for controls
    public double getXSpeed() {
        return XSpeed;
    }
    public double getZRotation() {
        return ZRotation;
    }

    public boolean getGearShift() {
        return gearShift;
    }

    public boolean getSlow() {
        return slow;
    }

    public boolean getHatchOpen() {
        return hatchOpen;
    }

    public boolean getHatchClosed() {
        return hatchClosed;
    }

    public boolean getHatchPunchOut() {
		return hatchPunchOut;
    }
    
	public boolean getHatchPunchIn() {
		return hatchPunchIn;
    }
    public boolean getAutoPunch() {
        return autoPunch;
    }

    public boolean getAutoShoot() {
        return autoShoot;
    }

    public boolean getBallPunch() {
        return punchBall;
    }

    public boolean getBallSecure() {
        return secureBall;
    }

    public double elevatorSpeedDown() {
        return manualClimbLift;
    }

    public boolean elevatorFrontUp() {
        if(manualFrontUp) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean elevatorRearUp() {
        if(manualRearUp) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean elevatorDrive() {
        if(manualDrive) {
            return true;
        }
        else {
            return false;
        }
    }

    public boolean autoClimbPressed() {
        if(false) {
            return autoClimb;
        }
        else {
            return false;
        }
    }

    public double elevatorRearDown() {
        return -manualRearDown;
    }
    public boolean manualControlSandstorm() {
        return manualControlSandstorm;
    }

    public boolean getVisionButton() { return visionButton; }

    //Custom function to allow triggers to act as buttons
    private boolean buttonPressed(double triggerValue, String trigger) {
        if(trigger.equals("left")) {
            if(triggerValue < 0.2 && leftTriggerPressed) {
                leftTriggerPressed = false;
                return true;
            }

            else if(triggerValue > 0.2 && !leftTriggerPressed) {
                leftTriggerPressed = true;
                return false;
            }
        
            else{
                return false;
            }
        }

        else if(trigger.equals("right")) {
            if(triggerValue < 0.2 && rightTriggerPressed) {
                rightTriggerPressed = false;
                return true;
            }

            else if(triggerValue > 0.2 && rightTriggerPressed) {
                rightTriggerPressed = true;
                return false;
            }
        
            else{
                return false;
            }
        }

        else {
            return false;
        }
    }
}