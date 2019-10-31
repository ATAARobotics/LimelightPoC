package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Teleop {
    // Vairables for robot classes
    private SWATDrive driveTrain;
    private Intake intake;
    private Elevator elevator;
    private RobotMap robotMap;
    private OI joysticks;
    private Shooter shooter;
    private Vision vision;
    private boolean autoShoot = true;
    private boolean shooterDone = true;
    private boolean punchDone = true;

    public boolean PIDEnabled = false;
    public boolean aligning = false;
    private PIDSubsystem visionAlignPID;
    private boolean visionActive = false;
    private double P = 0.035;
    private double I = 0.002;
    private double D = 0.0;
    private double tolerance = 0.1;

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
        vision = new Vision();
    }
    public void teleopInit() {
        joysticks.checkInputs();
        //intake.hatchOff();
        shooter.shooterInit();
        //Sets up PID
        visionAlignPID = new PIDSubsystem("AlignPID", P, I, D) {
            @Override
            protected double returnPIDInput() {return vision.getTx(); }
            @Override
            protected void usePIDOutput(double output) { drive(0, output, true);
                DriverStation.reportError("Vision Running " + output, true);
            }
            @Override
            protected void initDefaultCommand() { }
            };
        
        visionAlignPID.setAbsoluteTolerance(tolerance);
        visionAlignPID.getPIDController().setContinuous(false);
        visionAlignPID.setOutputRange(-1,1);
        visionAlignPID.setInputRange(-27, 27);
        // Disable Vision Processing on Limelight
        vision.setCameraMode(CMode.Drive);
        SmartDashboard.putNumber("Tolerance", tolerance);
    }

    public void TeleopPeriodic() {
        driveTrain.updateFromShuffleData();
        joysticks.updateFromShuffleData();
        updateFromShuffleData();
        visionAlignPID.setAbsoluteTolerance(tolerance);

        //drive
        if(joysticks.autoClimbPressed()) {
            elevator.setAutoClimb();
        }

        if(!elevator.getClimbing()) {

            if(joysticks.getVisionButton()){
                visionActive = !visionActive;
            }
            if(visionActive){
                if(!PIDEnabled){
                    vision.setCameraMode(CMode.Vision);
                    startAlignPID();
                    System.out.println("Vision Runs");
                }

            }else{
                if(PIDEnabled){
                    stopAlignPID();;
                    vision.setCameraMode(CMode.Drive);
                }
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
     //Alignment PID Commands
     private Double angleGoal;
    public void startAlignPID() {
        angleGoal = 0.0;
        visionAlignPID.setSetpoint(angleGoal);
        visionAlignPID.enable();
        PIDEnabled = true;
    }

    public void stopAlignPID() {
        visionAlignPID.disable();
        PIDEnabled = false;
    }
    public boolean alignDrive(){
        if (vision.getTv() == 0) {
            DriverStation.reportError("Unable to locate more than 1 target", true);
        }
        if (visionAlignPID.onTarget()) {
            visionAlignPID.disable();
            visionAlignPID.free();
            aligning = true;
            return (true);
        } else {
            aligning = false;
            return (false);
        }
    }
    public void updateFromShuffleData(){
        tolerance = SmartDashboard.getNumber("Tolerance", tolerance);
    }

 
}
