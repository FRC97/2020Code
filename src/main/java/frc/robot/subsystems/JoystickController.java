/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class JoystickController extends SubsystemBase {
  /**
   * Creates a new JoystickController.
   */
  private Joystick joy = new Joystick(0);
  private double Yval = joy.getRawAxis(1);
  private  double Xval = joy.getRawAxis(0);
  private  double Zval = joy.getRawAxis(2);
  private  double slider = joy.getRawAxis(3);

  private  boolean triggerP = joy.getRawButtonPressed(1);
  private  boolean button2P = joy.getRawButtonPressed(2);
  private  boolean button3P = joy.getRawButtonPressed(3);
  private  boolean button4P = joy.getRawButtonPressed(4);
  private  boolean button5P = joy.getRawButtonPressed(5);
  private  boolean button6P = joy.getRawButtonPressed(6);
  private  boolean button7P = joy.getRawButtonPressed(7);
  private  boolean button8P = joy.getRawButtonPressed(8);
  private  boolean button9P = joy.getRawButtonPressed(9);
  private  boolean button10P = joy.getRawButtonPressed(10);



  public JoystickController() {
/**
 * Add your docs here.
 */

  }

 public double getX(){
   return Xval;
 }
 public double getY(){
  return Yval;
}
public double getZ(){
  return Zval;
}
public double getSlider(){
  return slider;
}
public boolean getTrig(){
  return triggerP;
}
public boolean getB2(){
  return button2P;
}
public boolean getB3(){
  return button3P;
}
public boolean getB4(){
  return button4P;
}
public boolean getB5(){
  return button5P;
}
public boolean getB6(){
  return button6P;
}
public boolean getB7(){
  return button7P;
}
public boolean getB8(){
  return button8P;
}
public boolean getB9(){
  return button9P;
}
public boolean getB10(){
  return button10P;
}

}