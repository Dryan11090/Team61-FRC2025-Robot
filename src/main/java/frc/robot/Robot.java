
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.DriveSystem.SwerveModule;
import frc.robot.DriveSystem.SwerveSubsystem;
import frc.robot.Joysticks.DriverJoysticks;
import frc.robot.LimelightHelpers.LimelightResults;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 **/
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   **/
  public Robot() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
  }

  SwerveModule[] LowRide = {
    //Front Left
new SwerveModule(2, 1, 20),
     //Front Right
new SwerveModule(3, 4,21),
    //Back Right
new SwerveModule(6,5, 22),
  //Back Left
new SwerveModule(7,8,23) 
};  

Joystick CoralArmJoystick = new Joystick(1);
Joystick Balance = new Joystick(0);


TalonFX LeftLift = new TalonFX(9);
TalonFX RightLift = new TalonFX(10);
TalonFX CoralLift = new TalonFX(11);
TalonFX CoralStick = new TalonFX(12);
TalonFX AlgaeShoulderMotor = new TalonFX(13);
TalonFX AlgaeElboMotor = new TalonFX(14);
  SwerveSubsystem DriveBase = new SwerveSubsystem(LowRide, new Pigeon2(30));
  DriverJoysticks Driver = new DriverJoysticks();
  DigitalInput LimitLift = new DigitalInput(1);
  DigitalInput LimitCoral = new DigitalInput(0);
   DutyCycleEncoder EncoderCoral = new DutyCycleEncoder(2, 100, 0);
  boolean CoralHomeActive = false;
  double CoralStickRequest = 42;
     private ProfiledPIDController AnglePIDNEW = new ProfiledPIDController(0.3, 0,0,
    new TrapezoidProfile.Constraints(
         Math.PI/800, Math.PI*10)); //4/10*Math.PI Math.PI/4
  
         

  boolean CoralLiftActive = false;
  double EncoderValue = 0;

  boolean TurnAutoActive = false;
  int TurnAutoDeg = 0;

  DigitalInput CoralIntake = new DigitalInput(4);
  
  TalonSRX CoralIntakeMotor = new TalonSRX(25);

  int OperatorMode = 0;

  boolean strafeActiveLeft = false;
  boolean strafeActiveRight = false;
  boolean strafeActiveFoward = false;
 




  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   **/
  @Override
  public void robotPeriodic() {
//Brandon Spiller, best mechanical design officer and build team lead every. If this line of code is removed, the robot will kill itself
//"im so funny im brandon spiller" - Soumith Madadi (most mid officer ever, maybe less mid than Ben tho)
  }
 PIDController Ycamera = new PIDController(0.8,0,0);
 PIDController Xcamera = new PIDController(0.55,0,0);

PIDController ThetaCamera = new PIDController(0.01,0,0);
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   **/
  Timer autoClock;
  @Override
  public void autonomousInit() {
     m_autoSelected = m_chooser.getSelected();
     m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    AnglePIDNEW.enableContinuousInput(-180,180);
    autoClock = new Timer();
    autoClock.start();
    AutoState = 1;
    DriveBase.resetOdometry();

    

                //  UsbCamera camera = CameraServer.startAutomaticCapture();
              // Set the resolution
           //  camera.setResolution(640, 480);
   //   DriveBase.zeroHeading();
   //   DriveBase.HeadingPlusAngle(-90);
  }

  /** This function is called periodically during autonomous. **/

     // DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(DriveBase.Holo.getXController().calculate(DriveBase.odometer.getPoseMeters().getX(), 1.5)/4, 0, 0, DriveBase.getRotation2d())));
    // DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees()-60, 0)/16))); 
    // DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(new Rotation2d(0),0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees()-60, 0)/16));
    // DriveBase.setModuleState(Driver.getRobotRelativeChasisState(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees()-60, 0)/16));



  double YSpeed = 0;
  double XSpeed = 0;
  double TurningSpeed = 0;
  @Override
  public void autonomousPeriodic() {

        

    switch (AutoState) {
      case 1: // 
      
      XSpeedHelper = DriveBase.odometer.getPoseMeters().getX();
      XSpeedHelper = XSpeedHelper*1000;
      XSpeedHelper = Math.round(XSpeedHelper);
      XSpeedHelper = XSpeedHelper/1000;
      System.out.println("FowardSpeedHelper:  " + XSpeedHelper);
      XSpeed = DriveBase.Holo.getXController().calculate(XSpeedHelper, 1);

      YSpeedHelper = DriveBase.odometer.getPoseMeters().getY();
      YSpeedHelper = YSpeedHelper*1000;
      YSpeedHelper = Math.round(YSpeedHelper);
      YSpeedHelper = YSpeedHelper/1000;
      System.out.println("YSpeedHelper:  " + YSpeedHelper);
      YSpeed = DriveBase.Holo.getYController().calculate(YSpeedHelper, 0.0);

     // DriveBase.setModuleState(Driver.getRobotRelativeChasisState(XSpeed/2, YSpeed/2, 0)); 
      DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(new Rotation2d(0), XSpeed/2, 0, 0));
     
      SetRotateCoralMotor(67);

      if(SetRotateCoralMotorDone(67)) {
        CoralStick.set(0);
      }

      
      SetLiftCoralMotor(-210);

      if(SetRotateCoralMotorDone(-210)) {
        CoralLift.set(0);
      } 
    
      

   //   if(autoClock.get() >= 4) {
     //   AutoState = 2;
   //   }
        break;

      case 2: // 
    //  DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees()-45, 0)/16))); 
    //  XTagTarget("limelight-left");

      if(autoClock.get() >= 6) {
      //  DriveBase.resetOdometry();
        AutoState = 3;
      }
        break;

      case 3: //Strafe Left
    //  YTagTarget("limelight-left", -0.2);

        if(autoClock.get() >= 12) {
        //  DriveBase.resetOdometry();
          AutoState = 4;
        }
        break;

      case 4: // Drop

      
    //  XSpeedHelper = DriveBase.odometer.getPoseMeters().getX();
    //  XSpeedHelper = XSpeedHelper*1000;
    //  XSpeedHelper = Math.round(XSpeedHelper);
    //  XSpeedHelper = XSpeedHelper/1000;
    //  System.out.println("FowardSpeedHelper:  " + XSpeedHelper);
    //  XSpeed = DriveBase.Holo.getXController().calculate(XSpeedHelper, 0.7);

     // YSpeedHelper = DriveBase.odometer.getPoseMeters().getY();
     // YSpeedHelper = YSpeedHelper*1000;
     // YSpeedHelper = Math.round(YSpeedHelper);
     // YSpeedHelper = YSpeedHelper/1000;
     // System.out.println("YSpeedHelper:  " + YSpeedHelper);
     // YSpeed = DriveBase.Holo.getYController().calculate(YSpeedHelper, 0.0);

     // DriveBase.setModuleState(Driver.getRobotRelativeChasisState(XSpeed/2, YSpeed/2, 0)); 
     // DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(new Rotation2d(0), XSpeed/3, 0, 0));
       
      if(autoClock.get() >= 13.5) {
      //  DriveBase.resetOdometry();
        AutoState = 5;
      }
        break;

      case 5: //Back up //Lower Arm // Load Pos // Turn 180

    //  YSpeedHelper = DriveBase.odometer.getPoseMeters().getY();
    //  YSpeedHelper = YSpeedHelper*1000;
    //  YSpeedHelper = Math.round(YSpeedHelper);
    //  YSpeedHelper = YSpeedHelper/1000;
    //  System.out.println("YSpeedHelper:  " + YSpeedHelper);
    //  YSpeed = DriveBase.Holo.getYController().calculate(YSpeedHelper, 0.05);

        
        break; 

      case 6: //Locate Pick Up tag [1,2,12, or 13]
        
        break;

      case 7: // Intake with time delay and limit switch
        
        break;

      case 8: //Back up //Raise Arm // Turn toward previous face
        
        break;    

        case 9: // Slam in to Tag
        
        break;

        case 10: // Strafe
        
        break;
    }

    DriveBase.updateOdometer();
    SmartDashboard.putNumber("Robo Pos: X-axis", DriveBase.odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("Robo Pos: Y-axis", DriveBase.odometer.getPoseMeters().getY());



    
    switch (m_autoSelected) {
      case kCustomAuto:
    
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

int TESTcount = 0;
double ThetaSpeedAdder;
  /** This function is called once when teleop is enabled. **/
  @Override
  public void teleopInit() {
    AnglePIDNEW.enableContinuousInput(-Math.PI, Math.PI);
  }

  /** This function is called periodically during operator control. **/
  int AutoState = 1;
  @Override
  public void teleopPeriodic() {

    if(LimelightHelpers.getFiducialID("limelight-left") == 11) {
      System.out.println("YaxisNOOOWW: " + (LimelightHelpers.getBotPose("limelight-left")[1]+0.96));

    }



System.out.println("EEEEEEEEEECCCCODEER: " + CoralLift.getPosition());

    if(!(LimelightHelpers.getFiducialID("limelight-left") == -1))
    {

      SmartDashboard.putNumber("YSpeedHelper: ", LimelightHelpers.getBotPose("limelight-left")[1]+1.70);
      SmartDashboard.putNumber("XSpeedHelper: ", LimelightHelpers.getBotPose("limelight-left")[0]);
      
      
    }

    if(!LimitCoral.get()) {
      CoralLift.setPosition(0);
      }

    if(Driver.turnJoystick.getRawButton(11)) {
      DriveBase.resetOdometry();
    }
    


  //  SmartDashboard.putNumber("LimeLight X-Predict", LimelightHelpers.getBotPose("limelight-left")[0]);
  //  SmartDashboard.putNumber("LimeLight Y-Predict", LimelightHelpers.getBotPose("limelight-left")[1]);
  //  SmartDashboard.putNumber("LimeLight Another-Predict", LimelightHelpers.getBotPose("limelight-left")[2]);

    System.out.println("FFFFFFFFFFFFFFFFFF: " + (EncoderCoral.get()));
    SmartDashboard.putNumber("TAG ID???", LimelightHelpers.getFiducialID("limelight-left"));

    if(!strafeActiveLeft && !strafeActiveRight && !strafeActiveFoward) {
    if(Driver.driveJoystick.getTrigger()) {
    DriveBase.setModuleState(Driver.getRobotRelativeChasisState(0,0,0));   
    } else {
      if(Driver.driveJoystick.getRawButton(4)) {
        tagtarget = 1;
       // XTagTarget("limelight-left");
      } else {
    DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), 0, 0,ThetaSpeedAdder)); 
      }
    }
  } else {
    if (strafeActiveLeft) {

      YSpeedHelper = DriveBase.odometer.getPoseMeters().getY();
      YSpeedHelper = YSpeedHelper*1000;
      YSpeedHelper = Math.round(YSpeedHelper);
      YSpeedHelper = YSpeedHelper/1000;
      System.out.println("YSpeedHelper:  " + YSpeedHelper);
      YSpeed = DriveBase.Holo.getYController().calculate(YSpeedHelper, -0.05);

      DriveBase.setModuleState(Driver.getRobotRelativeChasisState(0, YSpeed, 0)); 
      if(Math.abs(YSpeed) <= 0.05) {
        strafeActiveLeft = false;
      }
    }
    if (strafeActiveRight) {

      YSpeedHelper = DriveBase.odometer.getPoseMeters().getY();
      YSpeedHelper = YSpeedHelper*1000;
      YSpeedHelper = Math.round(YSpeedHelper);
      YSpeedHelper = YSpeedHelper/1000;
      System.out.println("YSpeedHelper:  " + YSpeedHelper);
      YSpeed = DriveBase.Holo.getYController().calculate(YSpeedHelper, 0.05);

      DriveBase.setModuleState(Driver.getRobotRelativeChasisState(0, YSpeed, 0)); 
      if(Math.abs(YSpeed) <= 0.002) {
        strafeActiveRight = false;
      }
    }
    if(strafeActiveFoward) {

      XSpeedHelper = DriveBase.odometer.getPoseMeters().getX();
      XSpeedHelper = XSpeedHelper*1000;
      XSpeedHelper = Math.round(XSpeedHelper);
      XSpeedHelper = XSpeedHelper/1000;
      System.out.println("FowardSpeedHelper:  " + XSpeedHelper);
      XSpeed = DriveBase.Holo.getXController().calculate(XSpeedHelper, 0.2);

      YSpeedHelper = DriveBase.odometer.getPoseMeters().getY();
      YSpeedHelper = YSpeedHelper*1000;
      YSpeedHelper = Math.round(YSpeedHelper);
      YSpeedHelper = YSpeedHelper/1000;
      System.out.println("YSpeedHelper:  " + YSpeedHelper);
      YSpeed = DriveBase.Holo.getYController().calculate(YSpeedHelper, 0.0);

     // DriveBase.setModuleState(Driver.getRobotRelativeChasisState(XSpeed/2, YSpeed/2, 0)); 
      DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(new Rotation2d(0), XSpeed/2, YSpeed/2, 0));
      if(Math.abs(XSpeed) <= 0.03) {
        strafeActiveFoward = false;
      }
      
    }
  }


 if (Driver.turnJoystick.getRawButtonPressed(5)) {
    DriveBase.HeadingPlusAngle(-90);
 }
 if (Driver.turnJoystick.getRawButtonPressed(6)) {
    DriveBase.HeadingPlusAngle(90);
 }
 if (Driver.turnJoystick.getRawButtonPressed(3)) {
  DriveBase.zeroHeading();
}
if (Driver.turnJoystick.getRawButtonPressed(4)) {
  CoralLift.setPosition(0);
}


if(Driver.driveJoystick.getRawButtonPressed(6)) {
  strafeActiveLeft = !strafeActiveLeft; 
  strafeActiveRight = false;
  strafeActiveFoward = false;
  DriveBase.resetOdometry();
}
System.out.println("StrafeActiveLeft:  " + strafeActiveLeft);

if(Driver.driveJoystick.getRawButtonPressed(5)) {
  strafeActiveRight = !strafeActiveRight;
  strafeActiveLeft = false;
  strafeActiveFoward = false;
  DriveBase.resetOdometry();
}
System.out.println("StrafeActiveRight:  " + strafeActiveRight);

if(Driver.driveJoystick.getRawButtonPressed(12)) {
  strafeActiveFoward = !strafeActiveFoward;
  strafeActiveLeft = false;
  strafeActiveRight = false;
  DriveBase.resetOdometry();
}

   if(-Balance.getY()<-0.2 && LimitLift.get()) {
    double speed = Math.abs(Balance.getY());
    //Down
    LeftLift.set(speed);
    RightLift.set(-speed);
  } else
  { 
    if (-Balance.getY()>0.2) {
    double speed = Math.abs(Balance.getY());
    //Up
      LeftLift.set(-speed);
      RightLift.set(speed);
  }
  else {
    LeftLift.set(0);
    RightLift.set(0);
  }
}
    



  if(!LimitLift.get()) System.out.println("Lift Limit HIT");


    DriveBase.updateOdometer();
    SmartDashboard.putNumber("Robo Pos: X-axis", DriveBase.odometer.getPoseMeters().getX());
    SmartDashboard.putNumber("Robo Pos: Y-axis", DriveBase.odometer.getPoseMeters().getY());
    SmartDashboard.putNumber("Robo Pos: angle",  DriveBase.getRotation2d().getDegrees());



    if(Balance.getRawButton(11) && CoralIntake.get()) { // Coral Intake Action

        CoralIntakeMotor.set(ControlMode.PercentOutput, -0.6);
    } else {
      CoralIntakeMotor.set(ControlMode.PercentOutput, 0);
    }

    if(Balance.getRawButton(12)) { //Coral discharge action
      CoralIntakeMotor.set(ControlMode.PercentOutput, 1);
    }

    if(!CoralIntake.get() && !(Balance.getRawButton(12))) { //Coral disactive
      CoralIntakeMotor.set(ControlMode.PercentOutput, 0);
    }


    if(OperatorMode == 0 && CoralArmJoystick.getTriggerPressed()) {
      OperatorMode = 1; 
      CoralLift.set(0);
      CoralStick.set(0);
      AlgaeElboMotor.set(0);
    }

    if(OperatorMode == 1 && CoralArmJoystick.getTriggerPressed()) {
      OperatorMode = 0;
      CoralLift.set(0);
      CoralStick.set(0);
      AlgaeElboMotor.set(0);
    }
  
    SmartDashboard.putNumber("ThetaSpeed", ThetaSpeed);
    SmartDashboard.putNumber("ThetaSpeedHelper", DriveBase.getRotation2d().getDegrees());


  if(OperatorMode == 0) {
    JoystickOneToggle();
  }

  if(OperatorMode == 1) { //AUTO mode

    if (CoralArmJoystick.getRawButton(10)) { //Home 90
    //  CoralStickRequest = 90;
    }
    if (CoralArmJoystick.getRawButton(11) || CoralArmJoystick.getRawButton(9)) { //Load  87
      CoralStickRequest = 57;
    }
    if (CoralArmJoystick.getRawButton(12) || CoralArmJoystick.getRawButton(7) || CoralArmJoystick.getRawButton(8)) { //Discharge 98
      CoralStickRequest = 65;
    }

    if (CoralArmJoystick.getRawButton(9)) { //Home 0
      EncoderValue = -0.3;
    }
    if (CoralArmJoystick.getRawButton(7)) { //Level 2 || -68
      EncoderValue = -80;
    }
    if (CoralArmJoystick.getRawButton(8)) { //Top -210
      EncoderValue = -210;
    }
    
   
   
    //Coral Stick AUTO
     if(CoralArmJoystick.getRawButton(12) || CoralArmJoystick.getRawButton(11) || CoralArmJoystick.getRawButton(10) ||  CoralArmJoystick.getRawButton(7) || CoralArmJoystick.getRawButton(8) || CoralArmJoystick.getRawButton(9)  || CoralHomeActive) {
      SetRotateCoralMotor(CoralStickRequest);

      if(SetRotateCoralMotorDone(CoralStickRequest)) {
        CoralHomeActive = false;
      } else {
        CoralHomeActive = true;
      }
    } else {
      CoralStick.set(0);
    } 
    
   // SmartDashboard.putBoolean("Coral_Movement", CoralHomeActive);
   // SmartDashboard.putBoolean("Coral_Done", SetRotateCoralMotorDone(CoralStickRequest));
   // SmartDashboard.putNumber("Coral_Speed", CoralStick.get());
  
    
 //Coral Arm Lift AUTO

    if (CoralArmJoystick.getRawButton(7) || CoralArmJoystick.getRawButton(8)  || CoralArmJoystick.getRawButton(9) || CoralLiftActive) {
      SetLiftCoralMotor(EncoderValue);

      if(SetRotateCoralMotorDone(EncoderValue)) {
        CoralLiftActive = false;
      } else {
        CoralLiftActive = true;
      }
    }
 }    
}

  /** This function is called once when the robot is disabled. **/
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. **/
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. **/
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. **/
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. **/
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. **/
  @Override
  public void simulationPeriodic() {}


double YSpeedHelper = 0;
double XSpeedHelper = 0;
double ThetaSpeedHelper = 0;
double ThetaSpeed; 
double rotationGoal = 0;
int tagtarget = 1;
Timer StateTimer = new Timer();



double xOffset = 0;
double yOffset = 0;

  public void XTagTarget(String cameraName, double Xgoal) {
    switch (tagtarget) {
      case 1: //Step One // Line up with tag
      if(!(LimelightHelpers.getFiducialID(cameraName) == -1))
    {

      if(LimelightHelpers.getFiducialID(cameraName) == 6) {
        xOffset = -64;
        yOffset = -1.5;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 7) {
        xOffset = 0;
        yOffset = -57.4;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 8) {
        xOffset = 4.85;
        yOffset = 0.9;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 9) {
        xOffset = 3.74;
        yOffset = 0.96;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 10) {
        xOffset = 3.18;
        yOffset = 0;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 11) {
        xOffset = 3.1;
        yOffset = -1.70;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 17) {
        xOffset = -4.85;
        yOffset = -0.96;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 18) {
        xOffset = -5.4;
        yOffset = 0;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 19) {
        xOffset = -4.83;
        yOffset = -0.95;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 20) {
        xOffset = -3.7;
        yOffset = 0.97;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 21) {
        xOffset = -3.14;
        yOffset = 0.0;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 22) {
        xOffset = -3.73;
        yOffset = -0.99;
      }







      YSpeedHelper = LimelightHelpers.getBotPose(cameraName)[1];
      XSpeedHelper = LimelightHelpers.getBotPose(cameraName)[0];
      ThetaSpeedHelper = LimelightHelpers.getTX(cameraName);

      XSpeedHelper = (XSpeedHelper)*100;
      XSpeedHelper = Math.round(XSpeedHelper);
      XSpeedHelper = XSpeedHelper/100;

      YSpeedHelper = YSpeedHelper*10;
      YSpeedHelper = Math.round(YSpeedHelper);
      YSpeedHelper = YSpeedHelper/10;

      
   //   ThetaSpeedHelper = ThetaSpeedHelper*100;
   //   ThetaSpeedHelper = Math.round(ThetaSpeedHelper);
    //  ThetaSpeedHelper = ThetaSpeedHelper/100;

  
     // int ID = LimelightHelpers.getFiducialID("limelight-left");
  //  if(LimelightHelpers.getFiducialID("limelight-left") == 6) {
  //    rotationGoal = 60;
   // }
  //  if(LimelightHelpers.getFiducialID("limelight-left") == 7) {
  //    rotationGoal = 0;
 //   }



      YSpeed = Ycamera.calculate(YSpeedHelper-yOffset, 0);
      XSpeed = Xcamera.calculate(XSpeedHelper-xOffset, Xgoal);
 //     ThetaSpeed = ThetaCamera.calculate(ThetaSpeedHelper, 0); 
  //   ThetaSpeed = AnglePIDNEW.calculate(DriveBase.getRotation2d().getRadians(), Math.PI);

  //    SmartDashboard.putNumber("Heading: ", DriveBase.getHeading());
  //    SmartDashboard.putNumber("Goal: ", Math.PI/2);
 //     SmartDashboard.putNumber("Theta Speed ", ThetaSpeed);

      SmartDashboard.putNumber("XSpeedHelper__", XSpeedHelper);
      SmartDashboard.putNumber("YSpeedHelper__", YSpeedHelper-yOffset);

      SmartDashboard.putNumber("XSpeed_______", XSpeed);
      SmartDashboard.putNumber("YSpeed______", YSpeed);

      SmartDashboard.putNumber("EEEEEEEEEEE", YSpeedHelper-yOffset);
      SmartDashboard.putNumber("XXXXXXXXXX", XSpeedHelper-xOffset);
      System.out.println("YSpeedHelper: " + (YSpeedHelper-yOffset));
  
  DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(new Rotation2d(0), XSpeed/3, 0, 0));  
 // DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), XSpeed/4, YSpeed/4, 0));  
 //   DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), 0, 0, -ThetaSpeed/1));
        if ( (Math.abs(XSpeed) <= 0.08) && (Math.abs(YSpeed) <= 0.08) && (Math.abs(ThetaSpeed) <= 0.08) ) {
        //  tagtarget = 2;

          StateTimer.start();
          StateTimer.reset();

          DriveBase.resetOdometry();
        }
      } else {
        DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), 0, 0, 0));
      }
        break;

        case 2: 

      if(LimelightHelpers.getFiducialID(cameraName) == 7) {
        DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees(), 0)/16))); 

     //   if() {
     //     DriveBase.resetOdometry();
     //     tagtarget = 3;
     //   }
      }

      if(LimelightHelpers.getFiducialID(cameraName) == 11) {
        DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees()-60, 0)/16))); 

      }






       // XSpeedHelper = 0;
      //  XSpeed = DriveBase.Holo.getXController().calculate(DriveBase.odometer.getPoseMeters().getX(), 0); //Drive then set
//
     //   DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.,0,0)));
     //   if (StateTimer.get() >= 0.1) {
     //    tagtarget = 3; 
     //   }
     //   if(XSpeed <= 0.08) {
    //     tagtarget = 3; 
    //    }
        break;

        case 3: //Drive Foward
        
        break;

        case 4:
        
        break;

        case 5:
        
        break;
    
      default:
        break;
    }
  }

   public void YTagTarget(String cameraName, double Ygoal) {
    switch (tagtarget) {
      case 1: //Step One // Line up with tag
      if(!(LimelightHelpers.getFiducialID(cameraName) == -1))
    {

      if(LimelightHelpers.getFiducialID(cameraName) == 6) {
        xOffset = -64;
        yOffset = -1.5;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 7) {
        xOffset = 0;
        yOffset = -57.4;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 8) {
        xOffset = 4.85;
        yOffset = 0.9;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 9) {
        xOffset = 3.74;
        yOffset = 0.96;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 10) {
        xOffset = 3.18;
        yOffset = 0;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 11) {
        xOffset = 3.1;
        yOffset = -1.70;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 17) {
        xOffset = -4.85;
        yOffset = -0.96;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 18) {
        xOffset = -5.4;
        yOffset = 0;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 19) {
        xOffset = -4.83;
        yOffset = -0.95;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 20) {
        xOffset = -3.7;
        yOffset = 0.97;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 21) {
        xOffset = -3.14;
        yOffset = 0.0;
      }
      if(LimelightHelpers.getFiducialID(cameraName) == 22) {
        xOffset = -3.73;
        yOffset = -0.99;
      }







      YSpeedHelper = LimelightHelpers.getBotPose(cameraName)[1];
      XSpeedHelper = LimelightHelpers.getBotPose(cameraName)[0];
      ThetaSpeedHelper = LimelightHelpers.getTX(cameraName);

      XSpeedHelper = (XSpeedHelper)*100;
      XSpeedHelper = Math.round(XSpeedHelper);
      XSpeedHelper = XSpeedHelper/100;

      YSpeedHelper = YSpeedHelper*10;
      YSpeedHelper = Math.round(YSpeedHelper);
      YSpeedHelper = YSpeedHelper/10;

      
   //   ThetaSpeedHelper = ThetaSpeedHelper*100;
   //   ThetaSpeedHelper = Math.round(ThetaSpeedHelper);
    //  ThetaSpeedHelper = ThetaSpeedHelper/100;

  
     // int ID = LimelightHelpers.getFiducialID("limelight-left");
  //  if(LimelightHelpers.getFiducialID("limelight-left") == 6) {
  //    rotationGoal = 60;
   // }
  //  if(LimelightHelpers.getFiducialID("limelight-left") == 7) {
  //    rotationGoal = 0;
 //   }



      YSpeed = Ycamera.calculate(YSpeedHelper-yOffset, Ygoal);
      XSpeed = Xcamera.calculate(XSpeedHelper-xOffset, 0.1);
 //     ThetaSpeed = ThetaCamera.calculate(ThetaSpeedHelper, 0); 
  //   ThetaSpeed = AnglePIDNEW.calculate(DriveBase.getRotation2d().getRadians(), Math.PI);

  //    SmartDashboard.putNumber("Heading: ", DriveBase.getHeading());
  //    SmartDashboard.putNumber("Goal: ", Math.PI/2);
 //     SmartDashboard.putNumber("Theta Speed ", ThetaSpeed);

      SmartDashboard.putNumber("XSpeedHelper__", XSpeedHelper);
      SmartDashboard.putNumber("YSpeedHelper__", YSpeedHelper-yOffset);

      SmartDashboard.putNumber("XSpeed_______", XSpeed);
      SmartDashboard.putNumber("YSpeed______", YSpeed);

      SmartDashboard.putNumber("EEEEEEEEEEE", YSpeedHelper-yOffset);
      SmartDashboard.putNumber("XXXXXXXXXX", XSpeedHelper-xOffset);
      System.out.println("YSpeedHelper: " + (YSpeedHelper-yOffset));
  
  DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(new Rotation2d(0), XSpeed/4, YSpeed/4, 0));  
 // DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), XSpeed/4, YSpeed/4, 0));  
 //   DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), 0, 0, -ThetaSpeed/1));
        if ( (Math.abs(XSpeed) <= 0.08) && (Math.abs(YSpeed) <= 0.08) && (Math.abs(ThetaSpeed) <= 0.08) ) {
        //  tagtarget = 2;

          StateTimer.start();
          StateTimer.reset();

          DriveBase.resetOdometry();
        }
      } else {
        DriveBase.setModuleState(Driver.getFieldRelativeChasisSpeed(DriveBase.getRotation2d(), 0, 0, 0));
      }
        break;

        case 2: //Drive foward into the wall

      if(LimelightHelpers.getFiducialID(cameraName) == 7) {
        DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees(), 0)/16))); 

     //   if() {
     //     DriveBase.resetOdometry();
     //     tagtarget = 3;
     //   }
      }

      if(LimelightHelpers.getFiducialID(cameraName) == 11) {
        DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0,0, -AnglePIDNEW.calculate(DriveBase.getRotation2d().getDegrees()-60, 0)/16))); 

      }






       // XSpeedHelper = 0;
      //  XSpeed = DriveBase.Holo.getXController().calculate(DriveBase.odometer.getPoseMeters().getX(), 0); //Drive then set
//
     //   DriveBase.setModuleState(Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(0.,0,0)));
     //   if (StateTimer.get() >= 0.1) {
     //    tagtarget = 3; 
     //   }
     //   if(XSpeed <= 0.08) {
    //     tagtarget = 3; 
    //    }
        break;

        case 3: //Drive Foward
        
        break;

        case 4:
        
        break;

        case 5:
        
        break;
    
      default:
        break;
    }
  }
  

  double speed = 0;
  int mode = 0;
  public void JoystickOneToggle() {

     speed = -CoralArmJoystick.getY();
    System.out.println("Speed: " + speed);
    System.out.println("Coral LIMT: " + LimitCoral.get());
    if(Math.abs(CoralArmJoystick.getY())<=0.2) {
      speed = 0;
    }

    if(CoralArmJoystick.getRawButton(5)) { //Coral Lift
      mode = 1;
    }
    if(CoralArmJoystick.getRawButton(3)) { //Coral Stick
      mode = 2;
    }
    if(CoralArmJoystick.getRawButton(4)) { //Algae Sholder
      mode = 3;
    }
    if(CoralArmJoystick.getRawButton(6)) { //Disable
      mode = 4;
    }

    System.out.println("Joystick Mode: " + mode);
    if(Balance.getRawButton(7)) {
      AlgaeElboMotor.set(0.8);
    }
    if(Balance.getRawButton(8)) {
      AlgaeElboMotor.set(-0.8);
    }
    if(Balance.getRawButton(9)) {
      AlgaeElboMotor.set(0);
    }
    


    switch(mode) {
      case 0:
      CoralLift.set(0);
       CoralStick.set(0);
      AlgaeShoulderMotor.set(0);
      break;

      case 1: //Coral Lift
      
      CoralStick.set(0);
      AlgaeShoulderMotor.set(0);



      if(speed < 0 && LimitCoral.get()) {
        CoralLift.set(Math.abs(speed)); //Up - || Down +
      }


      if(!LimitCoral.get()) {
        CoralLift.set(0);
      }
            if(speed > 0) {
        CoralLift.set(-Math.abs(speed)); //Up - || Down +
      }
          if(speed == 0) {
        CoralLift.set(0);
      }


    break;

    case 2: //Coral Stick
      CoralStick.set(speed); // NEEDS TESTING

      CoralLift.set(0);
      AlgaeShoulderMotor.set(0);
    break;

    case 3: //Algae Sholder
    AlgaeShoulderMotor.set(-speed*0.5);

    CoralLift.set(0);
    CoralStick.set(0);

    break;

    case 4: //Lift LOCK
      if(LimitCoral.get()) {
        CoralLift.set(0.2);
      }
      if(!LimitCoral.get()) {
      CoralLift.set(0);
      }
    break;
    }
  }

  PIDController Coral = new PIDController(0.04/2, 0, 0);


   public void SetRotateCoralMotor(double ExpectedEncoder) {
    
    CoralStick.set(Coral.calculate(EncoderCoral.get(), ExpectedEncoder));
  }
  public boolean SetRotateCoralMotorDone(double ExpectedEncoder) {
    return 0.02 >= Math.abs(Coral.calculate(EncoderCoral.get(), ExpectedEncoder)); 
  }



  PIDController LiftCoral = new PIDController(0.015, 0, 0);


  public void SetLiftCoralMotor(double ExpectedEncoder) {
   
   CoralLift.set(LiftCoral.calculate(CoralLift.getPosition().getValueAsDouble(), ExpectedEncoder));
 }

 public boolean SetLiftCoralMotorDone(double ExpectedEncoder) {
   return 0.005 >= Math.abs(LiftCoral.calculate(CoralLift.getPosition().getValueAsDouble(), ExpectedEncoder)); 
 }
    
}

