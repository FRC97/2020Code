/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.robot.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterController extends SubsystemBase {
  private Ultrasonic ultrasonic = new Ultrasonic(null, null);
  //Will change upon futher shooterAngle and shooterHeight
  private final double shooterAngle = 55.0;
  private final double shooterHeight = 0.75;
  private WPI_TalonSRX topShooterMotor;
  private WPI_TalonSRX bottomShooterMotor;

  public ShooterController(){
    topShooterMotor = new WPI_TalonSRX(RobotMap.TS);
    bottomShooterMotor = new WPI_TalonSRX(RobotMap.BS);

    topShooterMotor.set(0);
    bottomShooterMotor.set(0);
  }
    /**
   * Calculate the angle of the shot to be made based on the distance from the wall
   *
   * Height is constant: (0.249m - the height at the center of the shooter)
   * Distance is non-constant: UltraSonic return values in mm
   * @param angle
   */
  private double[] Calculate() {
    double topSpeed = 0.0;
    double bottomSpeed = 0.0;
    double distance = ultrasonic.getRangeMM()*0.001;
    //Angle in Degrees for the Outer Port
    double targetAngle = Math.toDegrees(Math.tan(0.249/distance));
    double angleIncrement = 0;

    Math.copySign(angleIncrement, targetAngle);
    if (angleIncrement <= 0.5 || angleIncrement >= -0.5){
      topSpeed = 1.0;
      bottomSpeed = topSpeed;
    }
    else if (angleIncrement > 0.5){
      //bottomSpeed > topSpeed
    }
    else if (angleIncrement < -0.5){
      //bottomSpeed < topSpeed
    }


    double[] setShooterMotors = {topSpeed, bottomSpeed};
    return setShooterMotors;
  }
  
  /**
   * This Method sets the top and bottom motors to their adjusted speeds
   * From commands Shooter.java will send in to Params:
   * @param topAmount
   * @param bottomAmount
   */
  public void shoot(boolean pressed){
    if (pressed){
      topShooterMotor.set(ControlMode.Velocity, Calculate()[0]);
      bottomShooterMotor.set(ControlMode.Velocity, Calculate()[1]);
    }
  }
}
