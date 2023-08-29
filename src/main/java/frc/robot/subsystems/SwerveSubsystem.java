package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {
    
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad
    );

    private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad
    );

    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad
    );

    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad
    );

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    
    public SwerveSubsystem() {
        new Thread(() -> {
            try{
                Thread.sleep(1000);
                gyroReset();
            }catch (Exception e){}
        }).start();
    }
    //reset the gyro
    public void gyroReset(){
        gyro.reset();
    }
    //get the robot's heading (direction)
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }
    public Rotation2d getRotation2d(){
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }
    public void stopModules(){
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }
}
