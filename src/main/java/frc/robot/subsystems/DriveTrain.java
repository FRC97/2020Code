/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  public static CANSparkMax FL = new CANSparkMax(RobotMap.FL,MotorType.kBrushless);
  public static CANSparkMax BL = new CANSparkMax(RobotMap.BL,MotorType.kBrushless);
  public static CANSparkMax FR = new CANSparkMax(RobotMap.FR,MotorType.kBrushless);
  public static CANSparkMax BR = new CANSparkMax(RobotMap.BR,MotorType.kBrushless);
  static SpeedControllerGroup MR = new SpeedControllerGroup(BR, FR);
  static SpeedControllerGroup ML = new SpeedControllerGroup(BL, FL);
  public static DifferentialDrive m_drive = new DifferentialDrive(ML,MR);
  //private PIDController pid = new PIDController(0.8, 0.2, 0.1);
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    //setDefaultCommand();

  }
  public void arcadeDrive(double speed, double rotation){
    m_drive.arcadeDrive(speed, rotation);
  }
}
