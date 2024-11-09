// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.RobotContainerConstants;
import frc.robot.commands.VerticalMovement;
import frc.robot.commands.AprilTag;
import frc.robot.commands.IndexerReverse;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.NoteIntake;
import frc.robot.commands.PassNote;
import frc.robot.commands.ShootAMP;
import frc.robot.commands.ShootFinalSpeaker_Auto;
import frc.robot.commands.ShootPrepSpeaker_Auto;
import frc.robot.commands.ShootSpeaker;
import frc.robot.commands.ShootSpeaker_Auto;
import frc.robot.commands.ShooterReserve;
import frc.robot.commands.StopIntake;
import frc.robot.commands.ThrowNoteAway;
import frc.robot.commands.TrackNote_LimeLight;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final LimeLightSubsystem m_LimeLightSubsystem = new LimeLightSubsystem();
  // private final AMPBarSubsystem m_AMPBarSubsystem = new AMPBarSubsystem();
  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();
  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  private final PhotonVisionSubsystem m_PhotonVisionSubsystem = new PhotonVisionSubsystem();

  private final CommandXboxController operatorController = new CommandXboxController(RobotContainerConstants.operatorXboxController_ID);
  private final CommandXboxController driverController = new CommandXboxController(RobotContainerConstants.driverXboxController_ID);
  private final SendableChooser<Command> autoChooser;



  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    NamedCommands.registerCommand("NoteIntake", new NoteIntake(m_intakeSubsystem, m_indexerSubsystem).withTimeout(10));
    NamedCommands.registerCommand("ShootPrepSpeaker", new ShootPrepSpeaker_Auto(m_shooterSubsystem).withTimeout(0.02));
    NamedCommands.registerCommand("ShootSpeaker", new ShootSpeaker_Auto(m_shooterSubsystem, m_indexerSubsystem).withTimeout(0.5));
    NamedCommands.registerCommand("ShootFinalSpeaker", new ShootFinalSpeaker_Auto(m_shooterSubsystem, m_indexerSubsystem));

    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    DoubleSupplier rightClimbSpeed = ()-> operatorController.getRawAxis(1);
    DoubleSupplier leftClimbSpeed = ()-> operatorController.getRawAxis(5);

    BooleanSupplier ifFeed = ()-> operatorController.getHID().getRightBumper();
    BooleanSupplier climberInsurance = ()-> operatorController.getHID().getLeftBumper();
    BooleanSupplier isSlow = ()-> driverController.getHID().getLeftTriggerAxis()>0.4;
    DoubleSupplier xSpeed = ()-> -driverController.getRawAxis(1);
    DoubleSupplier ySpeed = ()-> -driverController.getRawAxis(0);
    DoubleSupplier zSpeed = ()-> -driverController.getRawAxis(4);

    // driverController.x().whileTrue(new TrackNote_LimeLight(m_swerveSubsystem, m_LimeLightSubsystem, m_indexerSubsystem));
    driverController.leftBumper().and(operatorController.y()).whileTrue(new TrackNote_LimeLight(m_swerveSubsystem, m_LimeLightSubsystem, m_indexerSubsystem));
    driverController.x().whileTrue(new AprilTag(m_PhotonVisionSubsystem, m_swerveSubsystem));
    driverController.b().whileTrue(
      Commands.runOnce(()-> {
        m_swerveSubsystem.resetGyro();
      })
    );
    operatorController.y().whileTrue(new ShooterReserve(m_shooterSubsystem));
    operatorController.x().whileTrue(new NoteIntake(m_intakeSubsystem, m_indexerSubsystem));
    operatorController.a().whileTrue(new ThrowNoteAway(m_intakeSubsystem, m_indexerSubsystem));
    operatorController.b().whileTrue(new PassNote(m_shooterSubsystem, m_indexerSubsystem, ifFeed));
    // operatorController.y().onTrue(new AMPBar(m_AMPBarSubsystem));
    operatorController.rightTrigger().whileTrue(new ShootSpeaker(m_shooterSubsystem, m_indexerSubsystem, ifFeed));
    operatorController.leftTrigger().whileTrue(new ShootAMP(m_shooterSubsystem, m_indexerSubsystem, ifFeed));
    operatorController.pov(0).or(operatorController.pov(315)).or(operatorController.pov(45)).whileTrue(new IndexerReverse(m_indexerSubsystem));

    m_climberSubsystem.setDefaultCommand(new VerticalMovement(m_climberSubsystem, leftClimbSpeed, rightClimbSpeed, climberInsurance));
    m_swerveSubsystem.setDefaultCommand(new ManualDrive(m_swerveSubsystem, xSpeed, ySpeed, zSpeed, isSlow));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return autoChooser.getSelected();
    return autoChooser.getSelected();
  }
}
