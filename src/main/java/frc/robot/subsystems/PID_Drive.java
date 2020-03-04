/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.controller.PIDController;

import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.interfaces.Gyro;
//import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;;
/**
 * Add your docs here.
 */
public class PID_Drive extends PIDSubsystem {

  //public final PIDController turnController;
  /**
   * Add your docs here.
   */
    static double speed;
    static double P, I, D = 1;
    static double integral, previous_error, setpoint = 0;
    static Gyro gyro;
  static double derivative;
  static double error;
  DifferentialDrive robotDrive;
  static double rcw;

  public PID_Drive(double P, double I, double D, Gyro gyro) {

    super("SubsystemName", P, I, D);
    PID_Drive.P = P;
    PID_Drive.I = I;
    PID_Drive.D = D;
    PID_Drive.gyro = gyro;
  }

  public static void setpoint(double setpoint) {
    PID_Drive.setpoint = setpoint;
  }
  // public double linear_Error()

  public static void PID_Angle() {
    error = setpoint - gyro.getAngle(); // Error = Target - Actual
    if (error > setpoint * 0.005) {
      PID_Drive.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
                                           // // IterativeRobot)
      derivative = (error - PID_Drive.previous_error) / .02;
      PID_Drive.rcw = P * error + I * PID_Drive.integral + D * derivative;
    }
    previous_error = error;
  }

  public static void PID_drive(double ultra) {
    error = setpoint - ultra; // Error = Target - Actual
    if (error > setpoint * 0.005) {
      PID_Drive.integral += (error * .02); // Integral is increased by the error*time (which is .02 seconds using normal
                                           // // IterativeRobot)
      derivative = (error - PID_Drive.previous_error) / .02;
      PID_Drive.rcw = P * error + I * PID_Drive.integral + D * derivative;
    }
    previous_error = error;
  }

  public static void executeTurn(double speed, double angle) {
      setpoint(angle);
      PID_Angle();
      DriveTrain.m_drive.arcadeDrive(speed, constrain(rcw * Math.PI / 180));
  }

  public static void turnInPlace(double speed) {

    DriveTrain.m_drive.tankDrive(speed, speed);

  }
  public static boolean vision() {

    return false;

  }

  public static void drive(double speed, double distance, double ultra) {

    setpoint(distance);
    PID_drive(ultra);
    DriveTrain.m_drive.arcadeDrive(speed, 0);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  protected double returnPIDInput() {
    // Return your input value for the PID loop
    // e.g. a sensor, like a potentiometer:
    // yourPot.getAverageVoltage() / kYourMaxVoltage;
    return 0.0;
  }

  @Override
  protected void usePIDOutput(double output) {
    // Use output to drive your system, like a motor
    // e.g. yourMotor.set(output);
  }

  public static double constrain(double num) {
    if (num > 1) {
      num = 1; 
    }
    else if (num < -1) {
      num = -1;
    }
    return num;
  }

}
