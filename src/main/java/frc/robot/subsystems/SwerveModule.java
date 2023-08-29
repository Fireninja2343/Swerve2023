package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;


public class SwerveModule {
    //Motors
    private final WPI_TalonFX DriveMotor;
    private final WPI_TalonFX TurningMotor;

    private final PIDController turningPidController;

    //Absolute Encoder
    private final AnalogInput absoluteEncoder;
    private final Boolean AbsoluteEncoderReversed;
    private final double absoluteEncoderOffset;

    public SwerveModule(int DriveMotorID, int TurningMotorID, boolean DriveMotorReversed,
     Boolean TurningMotorReversed, int absoluteEncoderID, boolean AbsoluteEncoderReversed, double absoluteEncoderOffset){

        this.absoluteEncoderOffset = absoluteEncoderOffset;
        this.AbsoluteEncoderReversed = AbsoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderID);

        DriveMotor = new WPI_TalonFX(DriveMotorID);
        TurningMotor = new WPI_TalonFX(TurningMotorID);

        DriveMotor.setInverted(DriveMotorReversed);
        TurningMotor.setInverted(TurningMotorReversed);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition(){
        return DriveMotor.getSelectedSensorPosition();
    }
    public double getTurningPosition(){
        return TurningMotor.getSelectedSensorPosition();
    }
     public double getDriveVelocity(){
        return DriveMotor.getSelectedSensorVelocity();
    }
     public double getTurningVelocity(){
        return TurningMotor.getSelectedSensorVelocity();
    }
     public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2 *Math.PI;
        angle -= absoluteEncoderOffset;
        return angle * (AbsoluteEncoderReversed ? -1.0 : 1.0);
    }

    // reset the encoder and set the turning into absolute encoder
    public void resetEncoders(){
        DriveMotor.setSelectedSensorPosition(0);
        TurningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setState(SwerveModuleState state){
        if (Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state,getState().angle);
        DriveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        TurningMotor.set(turningPidController.calculate(getTurningPosition(),state.angle.getRadians()));
        SmartDashboard.putString("Swerve["+ absoluteEncoder.getChannel() + "] state", state.toString());
    }
    public void stop(){
        DriveMotor.set(0);
        TurningMotor.set(0);
    }
    
}
