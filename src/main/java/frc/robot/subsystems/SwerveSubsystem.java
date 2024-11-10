package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveModule leftFront;
    private final SwerveModule rightFront;
    private final SwerveModule leftBack;
    private final SwerveModule rightBack;
    
    private final Pigeon2 gyro;
    private final Pigeon2Configuration gyroConfig;

    private final SwerveDriveOdometry odometry;

    private final Field2d field;
    /**
     * 
     */
    public SwerveSubsystem() {
        leftFront = new SwerveModule(
            SwerveConstants.leftFrontTurningMotorID,
            SwerveConstants.leftFrontDriveMotorID, 
            SwerveConstants.leftFrontAbsoluteEncoderID, 
            SwerveConstants.leftFrontOffset,
            SwerveConstants.leftFrontturningMotorInversion,
            SwerveConstants.leftFrontDriveMotorInversion);
        rightFront = new SwerveModule(
            SwerveConstants.rightFrontTurningMotorID, 
            SwerveConstants.rightFrontDriveMotorID, 
            SwerveConstants.rightFrontAbsoluteEncoderID, 
            SwerveConstants.rightFrontOffset,
            SwerveConstants.rightFrontturningMotorInversion,
            SwerveConstants.rightFrontDriveMotorInversion);
        leftBack = new SwerveModule(
            SwerveConstants.leftBackTurningMotorID, 
            SwerveConstants.leftBackDriveMotorID, 
            SwerveConstants.leftBackAbsoluteEncoderID, 
            SwerveConstants.leftBackOffset,
            SwerveConstants.leftBackturningMotorInversion,
            SwerveConstants.leftBackDriveMotorInversion);
        rightBack = new SwerveModule(
            SwerveConstants.rightBackTurningMotorID, 
            SwerveConstants.rightBackDriveMotorID, 
            SwerveConstants.rightBackAbsoluteEncoderID, 
            SwerveConstants.rightBackOffset,
            SwerveConstants.rightBackturningMotorInversion,
            SwerveConstants.rightBackDriveMotorInversion);
        field = new Field2d();
        gyro = new Pigeon2(SwerveConstants.pigeon2ID);
        gyroConfig = new Pigeon2Configuration();
        gyroConfig.MountPose.MountPoseYaw = -10;
        gyro.getConfigurator().apply(gyroConfig);
        resetGyro();
        odometry = new SwerveDriveOdometry(SwerveConstants.swervKinematics, gyro.getRotation2d(), getModulePosition());
        AutoBuilder.configureHolonomic(
            this::getPose, 
            this::setPose, 
            this::getSpeeds, 
            this::driveAuto,
            new HolonomicPathFollowerConfig(
                new PIDConstants(SwerveConstants.pathingMoving_Kp, SwerveConstants.pathingMoving_Ki, SwerveConstants.pathingMoving_Kd), // Translation constants 
                new PIDConstants(SwerveConstants.pathingtheta_Kp, SwerveConstants.pathingtheta_Ki, SwerveConstants.pathingtheta_Kd), // Rotation constants 
                SwerveConstants.maxVelocityMetersPersecond, 
                SwerveConstants.kDriveBaseRadius, // Drive base radius (distance from center to furthest module) 
                new ReplanningConfig(false, false)
            ),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                return alliance.get() == DriverStation.Alliance.Red;
                return false;
            },
            this
            );

            PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
            SmartDashboard.putData("Field", field);

    }
    @Override
    public void periodic() {
        odometry.update(gyro.getRotation2d(), getModulePosition());
        field.setRobotPose(odometry.getPoseMeters());
        // SmartDashboard.putNumber("leftFrontDrivePosition", leftFront.getDrivePosition());
        // SmartDashboard.putNumber("leftFrontturningPosition", leftFront.getTurningPosition());
        // SmartDashboard.putNumber("leftFrontVelocity", leftFront.getDriveVelocity());
        // SmartDashboard.putNumber("leftBackDrivePosition", leftBack.getDrivePosition());
        // SmartDashboard.putNumber("leftBackturningPosition", leftBack.getTurningPosition());
        // SmartDashboard.putNumber("leftBackVelocity", leftBack.getDriveVelocity());
        // SmartDashboard.putNumber("rightFrontDrivePosition", rightFront.getDrivePosition());
        // SmartDashboard.putNumber("rightFrontturningPosition", rightFront.getTurningPosition());
        SmartDashboard.putNumber("Swerve/rightFrontVelocity", rightFront.getDriveVelocity());
        SmartDashboard.putNumber("rightBackDrivePosition", rightBack.getDrivePosition());
        // SmartDashboard.putNumber("rightBackturningPosition", rightBack.getTurningPosition());
        SmartDashboard.putNumber("rightBackVelocity", rightBack.getDriveVelocity());
    }
    
    public ChassisSpeeds getSpeeds() {
        return SwerveConstants.swervKinematics.toChassisSpeeds(getModuleSates());
    }

    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
        SwerveModuleState[] states = null;
        if(fieldOrient) {
            states = SwerveConstants.swervKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, gyro.getRotation2d()));
        }else {
            states = SwerveConstants.swervKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        setModuleState(states);
    }

    public void setModuleStatesAuto(SwerveModuleState[] autoDesiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(autoDesiredStates, SwerveConstants.maxDriveMotorSpeed);
        leftFront.setDesiredState_Auto(autoDesiredStates[0]);
        rightFront.setDesiredState_Auto(autoDesiredStates[1]);
        leftBack.setDesiredState_Auto(autoDesiredStates[2]);
        rightBack.setDesiredState_Auto(autoDesiredStates[3]);
    }
    // Auto Drive
    public void driveAuto(ChassisSpeeds RobotSpeeds){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(RobotSpeeds, 0.01);
        SwerveModuleState[] states = SwerveConstants.swervKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStatesAuto(states);
    }
    public void auto_Drive(ChassisSpeeds speed){
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(speed, 0.02);
        SwerveModuleState[] states = SwerveConstants.swervKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStatesAuto(states);
    }

    public SwerveModuleState[] getModuleSates() {
        return new SwerveModuleState[] {
            leftFront.getstate(),
            rightFront.getstate(),
            leftBack.getstate(),
            rightBack.getstate()
        };
    }    

    public SwerveModulePosition[] getModulePosition() {
        return new SwerveModulePosition[] {
            leftFront.getPosition(),
            rightFront.getPosition(),
            leftBack.getPosition(),
            rightBack.getPosition()
        };
    }
    public void setModuleState(SwerveModuleState[] desiredState) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredState,1);
        leftFront.setState(desiredState[0]);
        rightFront.setState(desiredState[1]);
        leftBack.setState(desiredState[2]);
        rightBack.setState(desiredState[3]);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }

    public void resetGyro() {
        gyro.reset();
    }

    
}
