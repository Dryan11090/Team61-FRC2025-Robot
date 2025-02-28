package frc.robot.DriveSystem;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;

public class SwerveSubsystem {
    public final SwerveModule frontLeft, frontRight, backRight, backLeft;
    public double xPos = 0, yPos = 0;
    
    private PIDController DrivingXPID = new PIDController(1.5,0,0);
    private PIDController DrivingYPID = new PIDController(1.5,0,0);
    public PIDController AngleCONTROLERZ = new PIDController(0.2, 0, 0);
    private ProfiledPIDController AnglePID = new ProfiledPIDController(1.5, 0,0,
    new TrapezoidProfile.Constraints(
         Math.PI/800, Math.PI*8)); //4/10*Math.PI Math.PI/4
    
    public HolonomicDriveController Holo = new HolonomicDriveController(DrivingXPID, DrivingYPID, AnglePID);

    public Pigeon2 InertiaMeasureUnit;
    public final SwerveDriveOdometry odometer; 

    public SwerveSubsystem(SwerveModule[] modules, Pigeon2 pigeon)
    {
        this.frontLeft = modules[0];
        this.frontRight = modules[1];        
        this.backLeft = modules[2];
        this.backRight = modules[3];
        this.InertiaMeasureUnit = pigeon;
        this.odometer =
            new SwerveDriveOdometry(Constants.kDriveKinematics, new Rotation2d(0), new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backRight.getPosition(),
            backLeft.getPosition()} ,
            new Pose2d());
        AnglePID.enableContinuousInput(-Math.PI, Math.PI);
        AngleCONTROLERZ.enableContinuousInput(-Math.PI, Math.PI);
    }    
    public void intialize() {}

    public void updateOdometer() {
        odometer.update(getRotation2d(), new SwerveModulePosition[] {
            frontLeft.getPosition(), frontRight.getPosition(),
      backRight.getPosition(), backLeft.getPosition()}
        );
    }

    public void zeroHeading() 
    {
        InertiaMeasureUnit.setYaw(0);
    }

    public void zeroPos() {
        odometer.resetPose(new Pose2d(0,0, new Rotation2d(0)));
    }

    public void HeadingPlusAngle(double addAngle) 
    {
        InertiaMeasureUnit.setYaw(InertiaMeasureUnit.getYaw().getValueAsDouble() + addAngle);
    }

    public double getHeading() {
     return Math.IEEEremainder((InertiaMeasureUnit.getYaw().getValueAsDouble()) * Math.PI/180, 2*Math.PI); // Get rid of neg for low Add neg for high
    }

    public Rotation2d getRotation2d() 
    {
        return Rotation2d.fromRadians(this.getHeading());
    }

    public void resetOdometry() {
        odometer.resetPose(new Pose2d(0,0, new Rotation2d(0)));
    }

    public void t_driveZero() {
        frontLeft.t_driveZero(); frontRight.t_driveZero(); backRight.t_driveZero(); backLeft.t_driveZero();
    }

   // public void t_driveFowardOne() {
    //    Holo.calculate();
    // }

    public Pose2d CurrentPos() {
        return odometer.getPoseMeters();
    }

public void setModuleState(SwerveModuleState[] requestedState) {
    frontLeft.setState(requestedState[0]);
   frontRight.setState(requestedState[2]);
    backLeft.setState(requestedState[3]);
    backRight.setState(requestedState[1]);
 }
}
