package frc.robot.DriveSystem;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    public TalonFX driveMotor, turnMotor;
    public CANcoder absEncoder;
    
    public PIDController turningPID;

    private final NeutralOut m_brake = new NeutralOut();

    TalonFXConfiguration Configs;

    private final VelocityVoltage speedToVolt = new VelocityVoltage(0).withSlot(0);
    // private final VelocityTorqueCurrentFOC SpeedToCurrent = new VelocityTorqueCurrentFOC(0).withSlot(0);

    public SwerveModule(int driveMotorID, int turnMotorID, int absEncoderID)
    {
        this.absEncoder = new CANcoder(absEncoderID);
        this.driveMotor = new TalonFX(driveMotorID);
        this.turnMotor = new TalonFX(turnMotorID);
        

        turningPID = new PIDController(0.5, 0,0);
        turningPID.enableContinuousInput(Math.PI, -Math.PI);
      

        this.Configs = new TalonFXConfiguration();
        Configs.Slot0.kS = 0.0;
        Configs.Slot0.kV = 0.1;
        Configs.Slot0.kI = 0;
        Configs.Slot0.kD = 0;

        Configs.Voltage.PeakForwardVoltage = 6;
        Configs.Voltage.PeakReverseVoltage = -6;
        Configs.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        Configs.TorqueCurrent.PeakReverseTorqueCurrent = -80;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = driveMotor.getConfigurator().apply(Configs);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
         }
    }
    public double getAbsEncoderValue() {
        double absEncoderReading = this.absEncoder.getAbsolutePosition().getValueAsDouble();
        absEncoderReading *= 2*Math.PI; //convert rotation to Rad.
        return absEncoderReading;
    }
    public SwerveModuleState getState() 
    {
        return new SwerveModuleState(driveMotor.getVelocity().getValueAsDouble(), new Rotation2d(this.getAbsEncoderValue()));
    }

    public SwerveModulePosition getPosition() 
    {
     return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble()*Constants.kWheelDiameterMeters *Math.PI/Constants.kDriveMotorGearRatio,  new Rotation2d(getAbsEncoderValue())); 
    }
    
    public void setState(SwerveModuleState state) 
    {
    if(Math.abs(state.speedMetersPerSecond) < 0.05)
        {
            Stop();
            return;
        }
    state.optimize(new Rotation2d(getAbsEncoderValue()));
    driveMotor.setControl(speedToVolt.withVelocity(state.speedMetersPerSecond* 80));
    turnMotor.set(turningPID.calculate(getAbsEncoderValue(), state.angle.getRadians()));
    }

    public void Stop() 
    {
        driveMotor.set(0);
        turnMotor.set(0);
    }
    
    public void halt() {
        driveMotor.setControl(m_brake);
        turnMotor.setControl(m_brake);
    }

    public void t_driveZero() {
        driveMotor.set(0.1);
        turnMotor.set(turningPID.calculate(getAbsEncoderValue(), 0));
    }
}
