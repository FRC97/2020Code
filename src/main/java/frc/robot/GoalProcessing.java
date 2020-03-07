
package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.opencv.core.KeyPoint;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GripBallContourPipeline;
import frc.robot.subsystems.GripBallPipeline;
import frc.robot.subsystems.GripBallTwoPipeline;
import frc.robot.subsystems.GripPipeline;

public class GoalProcessing{
  NetworkTable visionTable;
  //double[] areas;
  //double[] defaultValue = new double[0];
  private static final int IMG_WIDTH = 320;
  private static final int IMG_HEIGHT = 240;
  VisionThread visionThread;
  private final Object imgLock = new Object();
  SmartDashboard board;
  CameraServer cserver;
  public double centerX;
  public double centerY;
  String aa;
  NetworkTableEntry bX;
  /*NetworkTableEntry bY;
  public final double STRIP = 19.625; 
  public final double HEIGHT = 17.0;
  public final double TOP_WIDTH = 39.25;
  public final double BOT_WIDTH = TOP_WIDTH - STRIP*Math.cos(Math.toRadians(60));
  public double[][] worldarr = new double[][]{{-TOP_WIDTH/2, BOT_WIDTH,0.0}, {-TOP_WIDTH/2, -BOT_WIDTH,0.0},{TOP_WIDTH/2, -BOT_WIDTH,0.0},{TOP_WIDTH/2, BOT_WIDTH,0.0}};
*/
public void init(){
    visionTable = NetworkTableInstance.getDefault().getTable("GRIP/mycontoursReport");
    bX = visionTable.getEntry("target X");
    bX.setDouble(centerY);
    SmartDashboard.putNumber("bX", centerX);
    UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT);

    visionThread = new VisionThread(camera, new GripPipeline(), pipeline -> {
        if (!pipeline.filterContoursOutput().isEmpty()) {
           Rect r = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
           synchronized (imgLock) {
            centerX = r.x + (r.width / 2);
            bX.setDouble(centerX);
            SmartDashboard.putNumber("bX", centerX);
          }
        }
    });
    visionThread.start();
}
}
