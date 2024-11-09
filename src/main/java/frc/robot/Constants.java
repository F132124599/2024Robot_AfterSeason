// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
  }

  public final class AMPBarConstants {
    public static final int arm_ID = 32;

    public static final double armPid_Kp = 1;
    public static final double armPid_Ki = 0;
    public static final double armPid_Kd = 0;

    public static final double armFeedforward_Ks = 0;
    public static final double armFeedforward_Kg = 0;
    public static final double armFeedforward_Kv = 0;

    public static final double armMotorGearRatio = 0;

    public static final double outAngle = 100;
    public static final double backAngle = 0;

    public static final double maxOutPut = 0.3;

    public static final boolean armMotorInversion = false;

    public static final double waitTime = 0;

  }

  public final class LimeLightConstants {
    public static final double trackNotePid_Kp = 0.1;
    public static final double trackNotePid_Ki = 0;
    public static final double trackNotePid_Kd = 0;

    public static final double trackNotePidMaxOutPut = 0.4;
    
  }

  public final class PhotonvisionConstants {
    public static final double xPid_Kp = 0;
    public static final double xPid_Ki = 0;
    public static final double xPid_Kd = 0;

    public static final double yPid_Kp = 0;
    public static final double yPid_Ki = 0;
    public static final double yPid_Kd = 0;

    public static final double zPid_Kp = 0.01;
    public static final double zPid_Ki = 0;
    public static final double zPid_Kd = 0;

    public static final int blueSpeakerCenterID = 7;
    public static final int blueSpeakerLeftID = 8;
    public static final int blueAmpID = 9;
    public static final int bluesourceRightID = 1;
    public static final int blueSourceLeftID = 2;
    public static final int blueTrapLeftID = 15;
    public static final int blueTrapRightID = 16;
    public static final int blueTrapCenterID = 14; 

    public static final int redSpeakerCenterID = 4;
    public static final int redSpeakerRightID = 3;
    public static final int redAmpID = 5;
    public static final int redSourceRightID = 9;
    public static final int redSourceLeftID = 10;
    public static final int redTrapLeftID = 11;
    public static final int redTrapRightID = 12;
    public static final int redTrapCenterID = 13;
    
  }

  public final class LEDConstants {
    public static final int LedNum = 19;
    public static final int candle_ID = 46;

    public static boolean LEDFlag = false;
    public static boolean hasNote = false;
    public static boolean intaking = false;
    public static boolean trackingNote = false;
    public static boolean hasNoteInSight = false;
    public static boolean prepSPEAKER = false;
    public static boolean prepAMP = false;
    public static boolean speedReadySPEAKER = false;
    public static boolean speedReadyAMP = false;
    public static boolean aimingAMP = true;
    public static boolean aimReadyAMP = false;
    public static boolean haveApriltag = true ;
    public static boolean playing = false;
    public static boolean prepPassNote = false;
    public static boolean speedReadyPassNote = false;
    
    
  }
  public final class IntakeConstants {
    public static final double intakeArmPID_Kp = 0.0047;
    public static final double intakeArmPID_Ki = 0;
    public static final double intakeArmPID_Kd = 0.00039;

    public static final double intakeArmFeedforward_Ks1 = 0;
    public static final double intakeArmFeedforward_Kg1 = 0;
    public static final double intakeArmFeedforward_Kv1 = 0;

    public static final double intakeArmFeedforward_Ks2 = 0.1;
    public static final double intakeArmFeedforward_Kg2 = 1;
    public static final double intakeArmFeedforward_Kv2 = 0;

    public static final double intakeArmFeedforward_Ks3 = -0.01;
    public static final double intakeArmFeedforward_Kg3 = -0.3;
    public static final double intakeArmFeedforward_Kv3 = 0;

    public static final double intakeArmFeedforward_Ks4 = 0.1;
    public static final double intakeArmFeedforward_Kg4 = 0.7;
    public static final double intakeArmFeedforward_Kv4 = 0;

    public static final double intakeArmFeedforward_Ks5 = 0.001;
    public static final double intakeArmFeedforward_Kg5 = 0.35;
    public static final double intakeArmFeedforward_Kv5 = 0;

    public static final double intakeArmFeedforward_Ks6 = 0;
    public static final double intakeArmFeedforward_Kg6 = -0.6;
    public static final double intakeArmFeedforward_Kv6 = 0;

    public static final double intakeCancoderOffset = 0.428;
    public static final double intakewheelVoltage = 7.2;
    public static final double intakeWheelEjectVoltage = -3;

    public static final double intakeArmArriveAngle = 0;
    public static final double intakeArmMaxOutPut = 0.3;
    public static final double arriveDownAngle = -58;
    public static final double arriveUpAngle = 60;

    public static final double intakeArmGearRatio = 0;

    public static final int intakeWheel_ID = 1;
    public static final int intakeArm_ID = 27;
    public static final int absoluteArmEncoderID = 45;
  }

  public final class ShooterConstants {
    public static final int shooterMotor_ID = 2;

    public static final double shootAMPVoltage = 4;
    public static final double shootSpeakerVoltage = 9.6;
    public static final double passNoteVoltage = 11.5;

    public static final double speedAMP = 1800;
    public static final double speedSpeaker = 4800;
    public static final double speedPassNote = 5700;
  }

  public final class ClimberConstants {
    public static final int leftClimberMotor_ID = 8;
    public static final int rightClimberMotor_ID = 31;

    public static final int rightRopeFinal_ID = 1;
    public static final int leftRopeFinal_ID = 2;

    public static final double climbUpVoltage = 0;
    public static final double climbDownVoltage = 0;
    public static final double rightMaxClimbposition = 130;
    public static final double leftmaxClimbPosition = 130;
  }
  
  public final class IndexerConstants {
    public static final int indexerMotor_ID = 10;
    public static final int irSensor_ID = 3;

    public static final double indexerVoltage = 9;
  }

  public final class SwerveConstants {
    public static final int leftFrontDriveMotorID = 29;
    public static final int rightFrontDriveMotorID = 19;
    public static final int leftBackDriveMotorID = 15;
    public static final int rightBackDriveMotorID = 16;

    public static final int leftFrontTurningMotorID = 21;
    public static final int rightFrontTurningMotorID = 17;
    public static final int leftBackTurningMotorID = 22;
    public static final int rightBackTurningMotorID = 26;

    public static final int leftFrontAbsoluteEncoderID = 43;
    public static final int rightFrontAbsoluteEncoderID = 42;
    public static final int leftBackAbsoluteEncoderID = 44;
    public static final int rightBackAbsoluteEncoderID = 41;

    public static final double leftFrontOffset = 0.4052734375;
    public static final double rightFrontOffset = 0.141845703125;
    public static final double leftBackOffset = -0.11767578125;
    public static final double rightBackOffset = 0.174072265625;

    public static final double xSpeedMaxOutPut = 0.6;
    public static final double ySpeedMaxOutPut = 0.6;
    public static final double zSpeedMaxOutPut = 0.6;
 
    public static final int pigeon2ID = 33;

    public static final double turningPidController_Kp = 0.012;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0;

    public static final int pidRangeMin = -180;
    public static final int pidRangeMax = 180;

    public static final double wheelDiameterMeters = Units.inchesToMeters(4);

    public static final double driveGearRatio = 1/6.75;
    public static final double turningGearRatio = 1.0/(150/7);

    public static final double maxVelocityMetersPersecond = 5;
    public static final double maxAccelerationMeterPersecond = 10;

    public static final boolean leftFrontturningMotorInversion = true;
    public static final boolean leftFrontDriveMotorInversion = false; 
    public static final boolean leftBackturningMotorInversion = true;
    public static final boolean leftBackDriveMotorInversion = false; 
    public static final boolean rightFrontturningMotorInversion = true;
    public static final boolean rightFrontDriveMotorInversion = false; 
    public static final boolean rightBackturningMotorInversion = true;
    public static final boolean rightBackDriveMotorInversion = false; 



    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio/60)*wheelDiameterMeters*Math.PI;

    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;

    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;
    public static final double turningEncoderRot2Rad = turningGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;

    public static final double kModuleDistance = 21*0.0254;

    public static SwerveDriveKinematics swervKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );

    public static final double pathingMoving_Kp = 12;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0.056;

    public static final double pathingtheta_Kp = 3;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0.035;

    public static final double maxOutput = 0;

    public static final double maxDriveMotorSpeed = 5;
    public static final double kDriveBaseRadius = 14.85 * 0.0254;

  }

  public final class RobotContainerConstants {
    public static final int operatorXboxController_ID = 0;
    public static final int driverXboxController_ID = 1;
  }

  public static double setMaxOutPut(double outPut, double maxOutPut){
    return Math.min(maxOutPut, Math.max(-maxOutPut, outPut));//值不用*12
  }
}
