package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule extends SubsystemBase{
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveMotorEncoder;

    private final CANcoder turningAbsoluteEncoder;
    private final CANcoderConfiguration CANcoderConfig;

    private final PIDController turningPidController;
    
    public SwerveModule(int turningMotorID, int driveMotorID, int absolutedEncoderID, double absolutedEncoderOffset, boolean turningMotorInversion, boolean driveMotorInversion) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotorEncoder = driveMotor.getEncoder();

        turningAbsoluteEncoder = new CANcoder(absolutedEncoderID);
        CANcoderConfig = new CANcoderConfiguration();
        CANcoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        CANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CANcoderConfig.MagnetSensor.MagnetOffset = absolutedEncoderOffset;
        turningAbsoluteEncoder.getConfigurator().apply(CANcoderConfig);

        turningPidController = new PIDController(SwerveConstants.turningPidController_Kp, SwerveConstants.turningPidController_Ki, SwerveConstants.turningPidController_Kd);
        turningPidController.enableContinuousInput(SwerveConstants.pidRangeMin, SwerveConstants.pidRangeMax);

        driveMotor.restoreFactoryDefaults();
        turningMotor.restoreFactoryDefaults();

        turningMotor.setInverted(turningMotorInversion);
        driveMotor.setInverted(driveMotorInversion);

        driveMotor.setIdleMode(IdleMode.kBrake);
        turningMotor.setIdleMode(IdleMode.kBrake);

        driveMotor.burnFlash();
        turningMotor.burnFlash();

        resetEncoder();
    }

    public SwerveModuleState getstate() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningPosition()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningPosition()));
    }

    public void resetEncoder() {
        driveMotorEncoder.setPosition(0);
    }

    public double getDrivePosition() {
        return driveMotorEncoder.getPosition()*SwerveConstants.driveEncoderRot2Meter;//*SwerveModuleConstants.driveEncoderRot2Meter
    }

    public double getTurningPosition() {
        return turningAbsoluteEncoder.getAbsolutePosition().getValueAsDouble()*360;//要*360
    }

    public double getDriveVelocity() {
        return driveMotorEncoder.getVelocity()*SwerveConstants.driveEncoderRPM2MeterPerSec;//*SwerveConstants.driveEncoderRPM2MeterPerSec
    }

    public void setState(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state,getstate().angle);
        double turningMotorOutput = turningPidController.calculate(getstate().angle.getDegrees(), optimizedState.angle.getDegrees());
        turningMotor.set(turningMotorOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond);
    }

    public void setDesiredState_Auto(SwerveModuleState state){
        state = SwerveModuleState.optimize(state, getstate().angle);
        // Drive Motor pid
        // double drivePidOutput = driveingPIDController.calculate(getState().speedMetersPerSecond, state.speedMetersPerSecond);
        driveMotor.set(state.speedMetersPerSecond/SwerveConstants.maxDriveMotorSpeed);
        // Angle Motor pid
        double anglePidOutput = turningPidController.calculate(getstate().angle.getDegrees(), state.angle.getDegrees());
        anglePidOutput = Math.abs(turningPidController.getPositionError())<1 ? 0 : anglePidOutput;
        turningMotor.set(anglePidOutput);
    }

    public void stopMotor() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void setState_Auto(SwerveModuleState state) {
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getstate().angle);
        double turningMotorOutput = turningPidController.calculate(getstate().angle.getDegrees(), optimizedState.angle.getDegrees());
        turningMotor.set(turningMotorOutput);
        driveMotor.set(optimizedState.speedMetersPerSecond/SwerveConstants.maxDriveMotorSpeed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    
}
