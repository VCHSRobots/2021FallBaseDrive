// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
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

  private Drivetrain m_drive=new Drivetrain();
  private final RamseteController m_ramsete = new RamseteController();
  private final Timer m_timer = new Timer();
  private Trajectory m_trajectory;

  private VisionServer mVisionServer = VisionServer.getInstance();

  @Override
  public void robotInit() {
    // Flush NetworkTables every loop. This ensures that robot pose and other values
    // are sent during every iteration.
    System.out.println("RobotInit-----------");
    setNetworkTablesFlushEnabled(true);
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
  }

  @Override
  public void autonomousInit() {
    System.out.println("Autonomous init-----------");
    m_timer.reset();
    m_timer.start();
    var pose=m_trajectory.getInitialPose();
    m_drive.resetOdometry(pose);
  }

  @Override
  public void autonomousPeriodic() {
    double elapsed = m_timer.get();
    Trajectory.State reference = m_trajectory.sample(elapsed);
    ChassisSpeeds speeds = m_ramsete.calculate(m_drive.getPose(), reference);

    // System.out.println(reference.toString());
    // System.out.println(m_drive.getPose());
    // System.out.println(speeds.toString());
    m_drive.drive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
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
    m_timer.reset();
    m_timer.start();
    var pose=m_trajectory.getInitialPose();
    m_drive.resetOdometry(pose);
    m_drive.setSpeeds(new DifferentialDriveWheelSpeeds());
    System.out.println("Disabled init-----------");
  }

}
