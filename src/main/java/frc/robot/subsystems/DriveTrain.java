/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  public static CANSparkMax front_Left = new CANSparkMax(RobotMap.front_Left_ID, MotorType.kBrushless);
  public static CANSparkMax back_Left = new CANSparkMax(RobotMap.back_Left_ID, MotorType.kBrushless);
  public static CANSparkMax front_Right = new CANSparkMax(RobotMap.front_Right_ID, MotorType.kBrushless);
  public static CANSparkMax back_Right = new CANSparkMax(RobotMap.back_Right_ID, MotorType.kBrushless);

  static SpeedControllerGroup m_Right = new SpeedControllerGroup(back_Right, front_Right);
  static SpeedControllerGroup m_Left = new SpeedControllerGroup(back_Left, front_Left);

  public static DifferentialDrive m_drive = new DifferentialDrive(m_Left,m_Right);
  //private PIDController pid = new PIDController(0.8, 0.2, 0.1);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand();

  }
}
