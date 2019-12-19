package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.vision.CameraMode;
import frc.robot.vision.LimeLight;

public class Teleop {
    // Vairables for robot classes
    private SWATDrive driveTrain;
    private Intake intake;
    private Elevator elevator;
    private RobotMap robotMap;
    private OI joysticks;
    private Shooter shooter;
    private LimeLight limeLight;
    private boolean autoShoot = true;
    private boolean shooterDone = true;
    private boolean punchDone = true;

    public boolean PIDEnabled = false;
    public boolean aligning = false;
    private int onTargetCounter = 0;
    private PIDSubsystem visionAlignPID;
    private boolean visionActive = false;
    private double P = 0.05;
    private double I = 0.0026;
    private double D = 0.05;
    private double tolerance = 0.2;

    /*UltrasonicCode
    private Ultrasonics ultrasonics;
    */

    public Teleop() {
        //Initialize Classes
        robotMap = new RobotMap();
        joysticks = new OI();
        driveTrain = new SWATDrive(robotMap);
        intake = new Intake(robotMap.getHatchIntake(), robotMap.getHatchPunch());
        elevator = new Elevator(robotMap);
        shooter = new Shooter(robotMap);
        limeLight = new LimeLight();
    }
    public void teleopInit() {
        //intake.hatchOff();
        shooter.shooterInit();
        //Sets up PID
        visionAlignPID = new PIDSubsystem("AlignPID", P, I, D) {
            @Override
            protected double returnPIDInput() {return limeLight.getTx(); }
            @Override
            protected void usePIDOutput(double output) { drive(0, output, true);
                //DriverStation.reportWarning("Vision Running " + output, true);
            }
            @Override
            protected void initDefaultCommand() { }
            };
        
        visionAlignPID.setAbsoluteTolerance(tolerance);
        visionAlignPID.getPIDController().setContinuous(false);
        visionAlignPID.setOutputRange(-1,1);
        visionAlignPID.setInputRange(-27, 27);
        // Disable Vision Processing on Limelight
        limeLight.setCameraMode(CameraMode.Drive);
        SmartDashboard.putNumber("Tolerance", tolerance);
        SmartDashboard.putNumber("Setpoint", visionAlignPID.getSetpoint());
    }

    public void TeleopPeriodic() {
        joysticks.checkInputs();
        driveTrain.updateFromShuffleData();
        updateFromShuffleData();
        visionAlignPID.setAbsoluteTolerance(tolerance);

        //drive
        if(joysticks.autoClimbPressed()) {
            elevator.setAutoClimb();
        }

        if(!elevator.getClimbing()) {

            if(joysticks.getVisionButton()) {
                visionActive = !visionActive;
                if (visionActive) {
                    onTargetCounter = 0;
                }
            }
            //TODO: Make it drive forward once aligned to target

            // Vision Alignment
            if(visionActive) {

                // Disable Vision if Aligned
                if(PIDEnabled && visionAlignPID.onTarget()){
                    DriverStation.reportWarning("On target", false);
                    onTargetCounter++;
                    // Once has been on target for 10 counts: Disable PID, Reset Camera Settings
                    if (onTargetCounter > 10) {
                        stopAlignPID();
                        limeLight.setCameraMode(CameraMode.Drive);
                        visionActive = false;
                    }
                } else {
                    //onTargetCounter = 0;
                    DriverStation.reportWarning("Not on target", false);
                    // Start PID if not started already, vision is enabled, and not aligned
                    if(!PIDEnabled){
                        limeLight.setCameraMode(CameraMode.Vision);
                        startAlignPID();
                        System.out.println("Vision Runs");
                    }
                }
            // If Vision is disabled normal driving and control operations. (AKA Mainly not vision code)
            }else{
                // If PID is enabled but vision is disabled stop vision alignment PID and reset camera settings
                if(PIDEnabled){
                    stopAlignPID();
                    limeLight.setCameraMode(CameraMode.Drive);
                }

                // Normal Driving and Operation controls
                driveTrain.arcadeDrive(joysticks.getXSpeed() * driveTrain.getMaxStraightSpeed(), joysticks.getZRotation() * driveTrain.getMaxTurnSpeed());
                //speed limiters

                if(joysticks.getGearShift()) {
                    driveTrain.gearShift();
                }
                if (joysticks.getSlow()) {
                    driveTrain.slow();
                }
                else;


                //hatch control
                if (joysticks.getHatchOpen()) {
                    intake.HatchOpen();
                }
                else if (joysticks.getHatchClosed()) {
                    intake.HatchClose();
                }
                else;
                /*if (joysticks.getHatchPunchOut()) {
                    intake.punchOut();
                }
                else if (joysticks.getHatchPunchIn()) {
                    intake.punchIn();
                }*/
                if (joysticks.getAutoPunch() || !punchDone) {
                    punchDone = intake.autoPunch();
                }
                else;

                if(joysticks.getAutoShoot() || !shooterDone) {
                    shooterDone = shooter.autoShoot();
                }

                else if(joysticks.getBallSecure()) {
                        shooter.gate();
                }
                else if(joysticks.getBallPunch() && !autoShoot) {
                    shooter.punch();
                }
                else;

                elevator.elevatorDown(joysticks.elevatorSpeedDown());
                //Elevator up
                if(joysticks.elevatorFrontUp()) {
                    elevator.frontElevatorUp(0.5);
                }

                if(joysticks.elevatorRearDown() < -0.2) {
                    elevator.rearElevatorDown(0.5);
                }

                else if(joysticks.elevatorRearUp()) {
                    elevator.rearElevatorUp(0.5);
                }

                //Drives forward on back elevator wheels
                if(joysticks.elevatorDrive()) {
                    elevator.driveElevator();
                }
                else {
                    elevator.stopDrive();
                }
            }
            elevator.elevatorPeriodic();
        /* public getUltrasonicRange(int direction) {
            ultrasonic.getRange(direction);
        }
        */
        }  
    }
	public void drive(double speedA, double speedB, boolean arcade) {
        if(arcade) {
            driveTrain.arcadeDrive(speedA, speedB);
        }
        else {
            driveTrain.tankDrive(speedA, speedB);
        }
	}
	public double getInches() {
		return robotMap.getEncoders().getDistance();
    }
    public void hatch(boolean outake) {
        if(outake) {
            intake.HatchOpen();
        }
        else {
            intake.HatchClose();
        }
    }
	public void TestPeriodic() {
        joysticks.checkInputs();
    }

    // -- Vision: --
    // Start alignment PID
     private Double angleGoal;
    public void startAlignPID() {
        visionAlignPID.setSetpoint(0.0);
        visionAlignPID.enable();
        PIDEnabled = true;
    }

    // Stop Alignment PID
    public void stopAlignPID() {
        visionAlignPID.disable();
        PIDEnabled = false;
        
    }
    // Update tolerance for Vision PID from shuffleboard
    public void updateFromShuffleData(){
        tolerance = SmartDashboard.getNumber("Tolerance", tolerance);
    }

    // -- End Vision --

 
}
