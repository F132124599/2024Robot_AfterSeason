// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.LimeLightConstants;

public class LimeLightSubsystem extends SubsystemBase {
  /** Creates a new LimLightSubsystem. */
  private final NetworkTable m_table = NetworkTableInstance.getDefault().getTable("limelight");

  private final PIDController pid;

  private double noteAngle;

  private double pidOutPut;
  
  public LimeLightSubsystem() {

    pid = new PIDController(LimeLightConstants.trackNotePid_Kp, LimeLightConstants.trackNotePid_Ki, LimeLightConstants.trackNotePid_Kd);
  }

  public double getNoteX() {
    return m_table.getEntry("tx").getDouble(0);
  }

  public boolean hasNote() {
    return !(m_table.getEntry("tx").getDouble(0)==0.0);
  }

  public double getPidOutPut() {
    noteAngle = getNoteX();
    pidOutPut = pid.calculate(noteAngle, 0);
    pidOutPut = Constants.setMaxOutPut(pidOutPut, LimeLightConstants.trackNotePidMaxOutPut);
    return pidOutPut;
  }

  public double getPidPositionError() {
    return pid.getPositionError();
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LimeLight/targetX", getNoteX());
    SmartDashboard.putBoolean("LimeLight/hasTarget", hasNote());

  }
}
