// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.vision.VisionServer;
import java.util.List;

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  private Drivetrain m_drive = new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;

  private VisionServer mVisionServer = VisionServer.getInstance();
  private double[] mostRecentTargetYZ = null;
  private PIDController m_visionPIDomega = new PIDController(20, 0, 700, 0.02);
  private PIDController m_visionPIDvx = new PIDController(7, 0, 240, 0.02);
  private ShuffleboardTab stRobot = Shuffleboard.getTab("Robot");
  private NetworkTableEntry ntPoseX = stRobot.add("PoseX", 0).getEntry();
  private NetworkTableEntry ntPoseOmega = stRobot.add("PoseOmega", 0).getEntry();
  private NetworkTableEntry ntTargetX = stRobot.add("TargetX", 0).getEntry();
  private NetworkTableEntry ntTargetOmega = stRobot.add("TargetOmega", 0).getEntry();
  private NetworkTableEntry ntVisionY = stRobot.add("VisionY",0).getEntry();
  private NetworkTableEntry ntVisionZ = stRobot.add("VisionZ",0).getEntry();
  private NetworkTableEntry ntVisionValid = stRobot.add("Vision Valid", false).getEntry();
  private NetworkTableEntry ntOmegaPIDout = stRobot.add("Omega PID out", 0).getEntry();
  private NetworkTableEntry ntVxPIDout = stRobot.add("Vx PID out", 0).getEntry();
  private double m_pidOut = 0;
  private double m_vxpidOut = 0;
  private double goalTS;

  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    System.out.println("RobotInit-----------");
    setNetworkTablesFlushEnabled(true);

    //vision
    m_visionPIDomega.setSetpoint(0);
    m_visionPIDomega.setTolerance(0.03);
    m_visionPIDvx.setTolerance(0.015);
    

    m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 2, new Rotation2d(0)),
        List.of(
          new Translation2d(8, 2),
          new Translation2d(8, 5)
          ), 
        new Pose2d(2, 5, new Rotation2d(Math.PI)), 
        new TrajectoryConfig(5, 4)
      );
  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
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

      // copied from drive()
      double delta_rad = mostRecentTargetYZ[1]; // get Horizontal when phone is veritcal
      Pose2d target = m_drive.getPose().transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(delta_rad)));
      ntTargetOmega.setNumber(target.getRotation().getRadians());
      ntOmegaPIDout.setNumber(m_visionPIDomega.calculate(m_drive.getPose().getRotation().getRadians(), target.getRotation().getRadians()));

      double delta_y = mostRecentTargetYZ[0]; // get Horizontal when phone is veritcal
      // double delta_meters = (delta_y + 0.23) * 4.6; // roughly 4.6 meters / normallized screen units
      // var t = new Translation2d(delta_meters, m_drive.getPose().getRotation());
      // Pose2d target = m_drive.getPose().transformBy(new Transform2d(t, new Rotation2d()));
      // m_vxpidOut = -1*m_visionPIDvx.calculate(m_drive.getPose().getTranslation().getDistance(target.getTranslation()), 0);
      m_vxpidOut = -1*m_visionPIDvx.calculate(delta_y, -0.23);
      // ntTargetX.setNumber(targetx.getX());
      ntVxPIDout.setNumber(m_vxpidOut);
    }
    ntPoseX.setNumber(m_drive.getPose().getX());
    ntPoseOmega.setNumber(m_drive.getPose().getRotation().getRadians());

    
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous init-----------");
    m_timer.reset();
    m_timer.start();
    var pose=m_trajectory.getInitialPose();
    m_drive.resetOdometry(pose);
    goalTS=0;
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    if (elapsed-goalTS>8){
      m_drive.targetX+=1;
      goalTS=elapsed;
    }
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);

    // System.out.println(reference.toString());
    // System.out.println(m_drive.getPose());
    // System.out.println(speeds.toString());
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  @Override
  public void teleopInit() {
    // TODO Auto-generated method stub
    super.teleopInit();
    m_timer.reset();
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void teleopPeriodic() {
    // if A button then joystick control, else vision
    if (m_controller.getAButton()) {
      // vision control
      // assuming 5 degree = 0.43 screen units
      // double delta_rad = Math.toRadians(mVisionServer.getFirstTargetYZ()[1] * (5.0 / 0.43)); // from 
      // assuming output from camera is radians
      ChassisSpeeds speeds;
      if (m_visionPIDomega.atSetpoint()) {
        speeds = new ChassisSpeeds(0,0,0);
      }
      else if (mVisionServer.getFirstTargetYZ() == null){
        speeds = new ChassisSpeeds(0,0,0);
      }
      else {
        double delta_rad = mostRecentTargetYZ[1]; // get Horizontal when phone is veritcal
        Pose2d target = m_drive.getPose().transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(delta_rad)));
        m_pidOut = m_visionPIDomega.calculate(m_drive.getPose().getRotation().getRadians(), target.getRotation().getRadians());
        speeds = new ChassisSpeeds(0,0,m_pidOut);
      }

      m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);

    }
    else if (m_controller.getBButton()) {
      ChassisSpeeds speeds;
      if (m_visionPIDvx.atSetpoint()) {
        speeds = new ChassisSpeeds(0,0,0);
      }
      else if (mVisionServer.getFirstTargetYZ() == null){
        speeds = new ChassisSpeeds(0,0,0);
      }
      else {
        double delta_y = mostRecentTargetYZ[0]; // get Horizontal when phone is veritcal
        // double delta_meters = (delta_y + 0.23) * 4.6; // roughly 4.6 meters / normallized screen units
        // var t = new Translation2d(delta_meters, m_drive.getPose().getRotation());
        // Pose2d target = m_drive.getPose().transformBy(new Transform2d(t, new Rotation2d()));
        // m_vxpidOut = -1*m_visionPIDvx.calculate(m_drive.getPose().getTranslation().getDistance(target.getTranslation()), 0);
        m_vxpidOut = -1*m_visionPIDvx.calculate(delta_y, -0.23);
        speeds = new ChassisSpeeds(m_vxpidOut,0,0);
      }
      m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }
    else if (m_controller.getXButton()) {
      ChassisSpeeds speeds;
      if (m_visionPIDvx.atSetpoint() && m_visionPIDomega.atSetpoint()) {
        speeds = new ChassisSpeeds(0,0,0);
      }

      else if (mVisionServer.getFirstTargetYZ() == null){
        speeds = new ChassisSpeeds(0,0,0);
      }
      else {
        double delta_rad = mostRecentTargetYZ[1]; // get Horizontal when phone is veritcal
        Pose2d target = m_drive.getPose().transformBy(new Transform2d(new Translation2d(0, 0), new Rotation2d(delta_rad)));
        m_pidOut = m_visionPIDomega.calculate(m_drive.getPose().getRotation().getRadians(), target.getRotation().getRadians());

        double delta_y = mostRecentTargetYZ[0];
        m_vxpidOut = -1*m_visionPIDvx.calculate(delta_y, -0.23);

        speeds = new ChassisSpeeds(m_vxpidOut,0,m_pidOut);
      }

      m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);

    }
    else {
      // Get the x speed. We are inverting this because Xbox controllers return
      // negative values when we push forward.
      double temp_y_left = util.deadband(m_controller.getY(GenericHID.Hand.kLeft), 0.15);
      double xSpeed = -m_speedLimiter.calculate(temp_y_left) * Drivetrain.kMaxSpeed;

      // Get the rate of angular rotation. We are inverting this because we want a
      // positive value when we pull to the left (remember, CCW is positive in
      // mathematics). Xbox controllers return positive values when you pull to
      // the right by default.
      double temp_x_right = util.deadband(m_controller.getX(GenericHID.Hand.kRight), 0.15);
      double rot = -m_rotLimiter.calculate(temp_x_right) * Drivetrain.kMaxAngularSpeed;

      m_drive.drive(xSpeed, rot);
    } 
  }

  @Override
  public void simulationInit() {
    System.out.println("Simulation init-----------");
  }

  @Override
  public void simulationPeriodic() {
    m_drive.simulationPeriodic();
  }

  @Override
  public void disabledInit() {
    m_timer.reset();
    var pose=m_trajectory.getInitialPose();
    m_drive.resetOdometry(pose);
    m_drive.setSpeeds(new DifferentialDriveWheelSpeeds());
    System.out.println("Disabled init-----------");
  }

}
