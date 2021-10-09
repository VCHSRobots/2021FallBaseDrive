// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.vision.VisionServer;
import java.util.List;

//import org.graalvm.compiler.lir.alloc.trace.lsra.TraceLinearScanLifetimeAnalysisPhase;
// update comment
// update again

public class Robot extends TimedRobot {
  private final XboxController m_controller = new XboxController(0);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0
  // to 1.
  private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(5);

  private Drivetrain m_drive=new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;
  private Trajectory m_trajectoryA;
  private Trajectory m_trajectoryB;
  private Trajectory m_trajectoryC;

  private VisionServer mVisionServer = VisionServer.getInstance();

  public NetworkTableInstance instance = NetworkTableInstance.getDefault();
  public NetworkTable table = instance.getTable("/auto");
  public NetworkTableEntry ntAutoA = table.getEntry("AutoA");
  public NetworkTableEntry ntAutoB = table.getEntry("AutoB");
  public NetworkTableEntry ntAutoC = table.getEntry("AutoC");
  private boolean isAChecked = false;
  private boolean isBChecked = false;
  private boolean isCChecked = false;

  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    System.out.println("RobotInit-----------");
    setNetworkTablesFlushEnabled(true);

    ntAutoA.setBoolean(true);
    ntAutoB.setBoolean(false);
    ntAutoC.setBoolean(false);

    m_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(1, 1, new Rotation2d(0)),
        List.of(), 
        new Pose2d(1, 0, new Rotation2d(Math.PI)), 
        new TrajectoryConfig(1, 1)
    );

    // 2.8 max first run x
    // 3.85 max y
    m_trajectoryA = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.4, 0),
            new Translation2d(0.7, 0),
            new Translation2d(1.2, 0),
            new Translation2d(1.7, 0),
            new Translation2d(2, 0),
            new Translation2d(2.3, 0),
            new Translation2d(2.6, 0),
            new Translation2d(3, 0),
            new Translation2d(3.2, 0),
            // new Translation2d(3.3, 0),
            // left turn first
            // new Translation2d(3.35, 0.1),
           new Translation2d(3.4, 0.2),
          //  new Translation2d(3.35, 0.3),
          //   new Translation2d(3.36, 0.4),
            // new Translation2d(3.39, 0.5),
            // new Translation2d(3.59, 0.7),
            // new Translation2d(3.69, 1.3), 
            // new Translation2d(3.69, 2),
            new Translation2d(3.7, 2.7), 
            new Translation2d(3.65, 3.4), 
            new Translation2d(3.6, 4.4),
            // left turn 2nd
            // new Translation2d(3.5, 4.4), 
            new Translation2d(3, 4.4), 
            new Translation2d(2.7, 4.4), 
            new Translation2d(2.5, 4.4), 
            new Translation2d(1.7, 4.4),
            new Translation2d(1, 4.4)

        // stop*/ss
        ), 
        new Pose2d(1.15, 4.4, new Rotation2d(Math.PI)), 
        new TrajectoryConfig(1.5, 1.5));

    m_trajectoryB = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(new Translation2d(2, 2), 
          new Translation2d(0, 0)
        ), 
        new Pose2d(3, 1, new Rotation2d(Math.PI / 2)),
        new TrajectoryConfig(1, 1));

    m_trajectoryC = TrajectoryGenerator.generateTrajectory(
        new Pose2d(2, 2, new Rotation2d(0)),
        List.of(
          new Translation2d(2, 4), 
          new Translation2d(1, 2)
        ), 
        new Pose2d(6, 1, new Rotation2d(Math.PI)),
        new TrajectoryConfig(2, 2));

  }

  @Override
  public void robotPeriodic() {
    m_drive.periodic();
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous init-----------");
    m_timer.reset();
    m_timer.start();


     isAChecked = ntAutoA.getBoolean(false);
     isBChecked = ntAutoA.getBoolean(false);
     isCChecked = ntAutoA.getBoolean(false);

     Pose2d pose;

    if (isAChecked){
      pose=m_trajectoryA.getInitialPose();
    }
    else if (isBChecked){
      pose=m_trajectoryB.getInitialPose();
    }
    else if (isCChecked){
      pose=m_trajectoryC.getInitialPose();
    }
    else{
      pose=m_drive.getPose();
    }
    m_drive.resetOdometry(pose);
    
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
  
    
    Trajectory.State reference;
    ChassisSpeeds speeds;
    if (isAChecked){
      reference = m_trajectoryA.sample(elapsed);
      speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    }
    else if (isBChecked){
      reference = m_trajectoryB.sample(elapsed);
      speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    }
    else if (isCChecked){
      reference = m_trajectoryC.sample(elapsed);
      speeds = m_ramsete.calculate(m_drive.getPose(), reference);
    }
    else{
      speeds = new ChassisSpeeds(0,0,0);
    }
    
      
      m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);

    
  

    // System.out.println(reference.toString());
    // System.out.println(m_drive.getPose());
    // System.out.println(speeds.toString());
    
  }

  @Override
  @SuppressWarnings("LocalVariableName")
  public void teleopPeriodic() {
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
    System.out.println("Disabled init-----------");
  }

}
