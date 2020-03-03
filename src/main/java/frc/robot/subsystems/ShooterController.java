/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterController extends SubsystemBase {
  // Will change upon futher shooterAngle and shooterHeight
  private static final double shooterWheelRadius = 0.051;
  private static final double shooterAngle = 65;
  private static final double shooterHeight = 0.475;
  private static final double shooterAngleRads = Math.toRadians(shooterAngle);

  /**
   * Calculate the angle of the shot to be made based on the distance from the
   * wall
   *
   * Height is constant: (2.49m - the height at the center of the shooter)
   * Distance is non-constant: UltraSonic return values in mm, must be in meters to calculate
   *
   * 105 inches is ideal distance to the inner goal (2.667m)
   */
  public static double Calculate(double distance) {
      double speed = 0.0; //Rad/s
      double speedRPM = 0.0; //RPM
      //Angle in Degrees for the Outer Port

      double numerator = -(9.81/2) * Math.pow(distance, 2);
      double denominator = ((2.49 - shooterHeight) - (distance * Math.tan(shooterAngleRads))) * Math.pow(Math.cos(shooterAngleRads), 2);

      speed = ((1/shooterWheelRadius) * Math.sqrt(numerator / denominator)); // Rads/Sec for the wheel
      speedRPM = speed * (30/(Math.PI)); //RPM Conversion
      //System.out.printf("Wheel Speed: %s RPM\nWheel Tangential Velocity is: %s m/s\nMotor Speed: %s RPM\nMotor Tangential Velocity: %s m/s\n", speedRPM, speed*shooterWheelRadius, speedRPM * 3, speed * 3 * shooterWheelRadius);
      return speedRPM*3/8500;
  }
}
