/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  public static WPI_TalonSRX front_Left = new WPI_TalonSRX(RobotMap.front_Left_ID);
  public static WPI_TalonSRX back_Left = new WPI_TalonSRX(RobotMap.back_Left_ID);
  public static WPI_TalonSRX front_Right = new WPI_TalonSRX(RobotMap.front_Right_ID);
  public static WPI_TalonSRX back_Right = new WPI_TalonSRX(RobotMap.back_Right_ID);
  static SpeedControllerGroup m_Right = new SpeedControllerGroup(back_Right, front_Right);
  static SpeedControllerGroup m_Left = new SpeedControllerGroup(back_Left, front_Left);
  public static DifferentialDrive m_drive = new DifferentialDrive(m_Left,m_Right);
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
