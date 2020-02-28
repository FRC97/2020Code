/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class ShooterController extends SubsystemBase {
  private static final CANSparkMax topShooterMotor = new CANSparkMax(RobotMap.TS, MotorType.kBrushless);
  private static final CANSparkMax bottomShooterMotor =  new CANSparkMax(RobotMap.BS, MotorType.kBrushless);
  // Will change upon futher shooterAngle and shooterHeight
  private static final double shooterAngle = 55.0;
  private static final double shooterHeight = 0.75;
  private static final double shooterAngleRads = Math.toRadians(shooterAngle);

  /**
   * Calculate the angle of the shot to be made based on the distance from the
   * wall
   *
   * Height is constant: (2.49m - the height at the center of the shooter)
   * Distance is non-constant: UltraSonic return values in mm
   * 105 inches is ideal distance to the inner goal
   */
  private static double Calculate(Double distance) {
    double speed = 0.0; //Rad/s
    double speedRPM = 0.0; //RPM
    //Angle in Degrees for the Outer Port

    double numerator = -4.905 * Math.pow(distance, 2);
    double denominator = ((2.49 -shooterHeight) - (distance * Math.tan(shooterAngleRads))) * Math.pow(Math.cos(shooterAngleRads), 2);

    speed = (10 * Math.sqrt(numerator / denominator)); // Rads/Sec for the wheel
    speedRPM = 3 * speed * (60/(2*Math.PI)); //RPM for the motor

    return speedRPM;
  }

  /**
   * This Method sets the top and bottom motors to their adjusted speeds
   * From commands Shooter.java will send in to Params:
   * @param topAmount
   * @param bottomAmount
   */
  public static void shoot(boolean pressed, double d){
    if (pressed){
      topShooterMotor.set(Calculate(d));
      bottomShooterMotor.set(-Calculate(d));
    }
    else {
      topShooterMotor.set(0);
      bottomShooterMotor.set(0);
    }
  }
}
