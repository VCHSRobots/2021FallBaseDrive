// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.vision.VisionServer;

public class visionController {
  DigitalOutput VisionCamera = new DigitalOutput(0);
  private VisionServer mVisionServer = VisionServer.getInstance();
  private double[] mostRecentTargetYZ = {0, 1};
  private PIDController m_visionPIDomega = new PIDController(20, 0, 700, 0.02);
  private PIDController m_visionPIDvx = new PIDController(7, 0, 240, 0.02);

  private ShuffleboardTab stRobot = Shuffleboard.getTab("Robot");
  private NetworkTableEntry ntVisionY = stRobot.add("VisionY",0).getEntry();
  private NetworkTableEntry ntVisionZ = stRobot.add("VisionZ",0).getEntry();
  private NetworkTableEntry ntVisionValid = stRobot.add("Vision Valid", false).getEntry();

  private double kMaxVxSpeed = 2.5;
  private double kMaxRotationSpeed = 3*Math.PI;

  private double targetX_offset = 0.0;
  private double targetRot_offset = 0.0;

  /** Creates a new visionController. */
  public visionController() {
        // The PIDController used by the subsystem
        
  }

  public void robotInit() {
    
    //vision
    m_visionPIDomega.setSetpoint(0);
    m_visionPIDomega.setTolerance(0.03);
    m_visionPIDvx.setTolerance(0.015);
  }

  public void robotPeriodic() {
    double[] tmpTarget = mVisionServer.getFirstTargetYZ();
    if (tmpTarget == null) {
      // no valid target
      ntVisionValid.setBoolean(false);
    }
    else {
      mostRecentTargetYZ = tmpTarget;
      // calculate pid on Z, horizontal error
      ntVisionValid.setBoolean(true);
      ntVisionY.setNumber(mostRecentTargetYZ[0]);
      ntVisionZ.setNumber(mostRecentTargetYZ[1]);
    }
  }
  

  // returns target pose based on rotation
  // TODO: implement target pose based on vertical 
  public Pose2d getTargetPoseFromFirst(Pose2d starting_pose) {
    //
    // copied from drive()
    double delta_rad = mostRecentTargetYZ[1] + targetRot_offset; // get Horizontal when phone is veritcal
    return starting_pose.transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(delta_rad)));
  }

  // returns radians per second
  public double rotationPIDCalculate(double currentRadians, double setpointRadians) {
    var x = m_visionPIDomega.calculate(currentRadians,setpointRadians);
    return (Math.abs(x) < kMaxRotationSpeed) ? -x : Math.copySign(kMaxRotationSpeed, -x);
  }

  // returns m/s
  public double vxPIDCalculate(double currentXValue, double setpointXValue) {
    var x = m_visionPIDvx.calculate(currentXValue, setpointXValue + targetX_offset);
    return (Math.abs(x) < kMaxVxSpeed) ? -x : Math.copySign(kMaxVxSpeed, -x);
  }

  public ChassisSpeeds getChassisSpeedsFromFirst(Pose2d currentPose) {
    ChassisSpeeds speeds;
    if (m_visionPIDomega.atSetpoint() && m_visionPIDvx.atSetpoint()) {
      speeds = new ChassisSpeeds(0,0,0);
    }
    else if (mVisionServer.getFirstTargetYZ() == null){
      speeds = new ChassisSpeeds(0,0,0);
    }
    else {
      Pose2d target = getTargetPoseFromFirst(currentPose);
      double rot = rotationPIDCalculate(currentPose.getRotation().getRadians(), 
                                        target.getRotation().getRadians());
      double vx = -1*vxPIDCalculate(mostRecentTargetYZ[0], -0.23);

      speeds = new ChassisSpeeds(vx, 0, rot);
    }
    return speeds;
  }

  public void turnLEDon(){
    VisionCamera.set(true);
  }

  public void turnLEDoff(){
    VisionCamera.set(false);
  }
 

}