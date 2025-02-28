package frc.robot.Joysticks;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.DriveSystem.SwerveSubsystem;

public class DriverJoysticks {
public Joystick turnJoystick = new Joystick(3);  
   public Joystick driveJoystick = new Joystick(2);

  SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);
  SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);
  SlewRateLimiter turningLimiter = new SlewRateLimiter(Constants.PhysicalMaxAcclerationUnitsPerSecond);

  //initialize output variables (unit: m/s)
  double xSpeed = 0;
  double ySpeed = 0;
  double turningSpeed = 0; 
  public double AddedTurningSpeed = 0;

  SwerveSubsystem DriveBase;

  public  SwerveModuleState[] getFieldRelativeChasisSpeed(Rotation2d RobotAngle, double XSpeedAdder, double YSpeedAdder, double ThetaSpeedAdder) {

        double attainableMaxSpeedMetersPerSecond = 5;
        SwerveModuleState[] initialStates = Constants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(getXSpeed()+XSpeedAdder, getYSpeed()+YSpeedAdder, getTurningSpeed() + ThetaSpeedAdder, RobotAngle));
        // Normilize script || Keep us from pushing pass our limits
       if(!(Math.abs(getXSpeed())==0) && !(Math.abs(getYSpeed())==0)) {
        double realMaxSpeed = 0;
        for(SwerveModuleState moduleState : initialStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : initialStates) {
              moduleState.speedMetersPerSecond =
                  moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
          }
       }
        return initialStates; 
    }


        public  SwerveModuleState[] AngleOverride_getFieldRelativeChasisSpeed(Rotation2d RobotAngle, double AngleOverideSpeed) {

        double attainableMaxSpeedMetersPerSecond = 5;
        SwerveModuleState[] initialStates = Constants.kDriveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(getXSpeed(), getYSpeed(), getTurningSpeed()+AngleOverideSpeed, RobotAngle));
        // Normilize script || Keep us from pushing pass our limits
       if(!(Math.abs(getXSpeed())==0) && !(Math.abs(getYSpeed())==0)) {
        double realMaxSpeed = 0;
        for(SwerveModuleState moduleState : initialStates) {
            realMaxSpeed = Math.max(realMaxSpeed, Math.abs(moduleState.speedMetersPerSecond));
        }
        if (realMaxSpeed > attainableMaxSpeedMetersPerSecond) {
            for (SwerveModuleState moduleState : initialStates) {
              moduleState.speedMetersPerSecond =
                  moduleState.speedMetersPerSecond / realMaxSpeed * attainableMaxSpeedMetersPerSecond;
            }
          }
       }
        return initialStates; 
    }


    public  SwerveModuleState[] getRobotRelativeChasisState(double XSpeedAdder, double YSpeedAdder, double ThetaSpeedAdder) {
        return  Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(-(getXSpeed()/3)+XSpeedAdder, -getYSpeed()+YSpeedAdder, getTurningSpeed()+ThetaSpeedAdder)); 
        }

    public  SwerveModuleState[] AngleOverride_getRobotRelativeChasisState(double AngleOverideSpeed) {
         return  Constants.kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(getXSpeed(), getYSpeed(), getTurningSpeed()+AngleOverideSpeed)); 
        }

        public double getXSpeed() {
            //if not in deadzone and controller is connected
                     if(Math.abs(driveJoystick.getY()) > 0.17 && driveJoystick.isConnected()) {
                     return xLimiter.calculate(driveJoystick.getY()*0.5); // Constants.topSpeedX);    
                    }
                    return 0;
                }
                public double getYSpeed() {
                    if(Math.abs(driveJoystick.getX()) > 0.17 && driveJoystick.isConnected()) {
                        return yLimiter.calculate(driveJoystick.getX()* 0.5); // Constants.topSpeedY);    
                    }
                    return 0;
                }
            
                public double getTurningSpeed() {
                    if(Math.abs(turnJoystick.getX()) > 0.17 && turnJoystick.isConnected()) {
                     return turningLimiter.calculate(turnJoystick.getX()*Constants.topSpeedTurning);    
                    }
                    return 0;
                }
}
