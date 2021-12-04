// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
//import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

@SuppressWarnings("PMD.TooManyFields")
public class Drivetrain {
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable table = instance.getTable("/drive");
  public NetworkTableEntry tableYaw = table.getEntry("Yaw");
  public NetworkTableEntry tableLeftEncoder = table.getEntry("LeftEnc");
  public NetworkTableEntry tableRightEncoder = table.getEntry("RightEnc");
  
  public NetworkTableEntry tableLeftVelocity = table.getEntry("LeftVel");
  public NetworkTableEntry tableRightVelocity = table.getEntry("RightVel");
  

  // 3 meters per second.
  public boolean isSimulation;
  public static final double kMaxSpeed = 1.5;
  // 1/2 rotation per second.
  public static final double kMaxAngularSpeed = 3*Math.PI;

  // private static final double kTrackWidth = 0.346075; // 13'5/8"
  private static final double kTrackWidth = 0.346075;
  private static final double kWheelRadius = 0.1016 * 0.5; // 4 inch
  private static final int kEncoderResolution = 2048; // falcon 500 + talon FX encoder
  private static final double kEncoderTicksPerMeter = (50.0/12.0) * kEncoderResolution / ( 2.0*kWheelRadius*Math.PI ); // replace 12 with real Motor:Wheel gear Ratio

  private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(11);
  private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(12);

  private final PIDController m_leftPIDController = new PIDController(2.4, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(2.4, 0, 0);

  private AHRS m_gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(kTrackWidth);
  private DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(0.821, 1.5, 0.14);

  // Simulation classes help us simulate our robot
  private double leftVolt,rightVolt;
  private Encoder m_leftEncoder;
  private Encoder m_rightEncoder;
  private EncoderSim m_leftEncoderSim;
  private EncoderSim m_rightEncoderSim;
  private final Field2d m_fieldSim = new Field2d();
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(1.48, 0.215, 1.65,
       0.073);
  private final DifferentialDrivetrainSim m_drivetrainSimulator = new DifferentialDrivetrainSim(m_drivetrainSystem, 
       DCMotor.getFalcon500(1), 50.0/12.0, kTrackWidth, kWheelRadius, null);

  /** Subsystem constructor. */
  public Drivetrain() {
   
    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
    if (Robot.isSimulation()){
      m_leftEncoder = new Encoder(6, 7);
      m_rightEncoder = new Encoder(8, 9);
      m_leftEncoderSim = new EncoderSim(m_leftEncoder);
      m_rightEncoderSim = new EncoderSim(m_rightEncoder);
      m_leftEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
      m_rightEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);
      m_leftEncoder.reset();
      m_rightEncoder.reset();
    }   

    m_gyro.calibrate();

    TalonFXConfiguration _drive_config = new TalonFXConfiguration();
    _drive_config.feedbackNotContinuous = true;
    _drive_config.neutralDeadband = 0.005;
    _drive_config.nominalOutputForward = 0.0;
    _drive_config.nominalOutputReverse = 0.0;
    _drive_config.openloopRamp = 0.0;
    _drive_config.peakOutputForward = 1.0;
    _drive_config.peakOutputReverse = -1.0;
    _drive_config.voltageCompSaturation = 9.0;

    m_leftLeader.configFactoryDefault();
    m_rightLeader.configFactoryDefault();
    m_leftLeader.configAllSettings(_drive_config);
    m_rightLeader.configAllSettings(_drive_config);
    m_leftLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    m_rightLeader.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);

    m_leftLeader.setSelectedSensorPosition(0);
    m_rightLeader.setSelectedSensorPosition(0);
    m_leftLeader.setInverted(InvertType.InvertMotorOutput);
    m_rightLeader.setInverted(InvertType.None);
    
    SmartDashboard.putData("Field", m_fieldSim);
  }

  /** Sets speeds to the drivetrain motors. */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
    double leftOutput;
    double rightOutput;
    if (Robot.isSimulation()){
      leftOutput = m_leftPIDController.calculate(m_leftEncoder.getRate(), speeds.leftMetersPerSecond);
      rightOutput = m_rightPIDController.calculate(m_rightEncoder.getRate(), speeds.rightMetersPerSecond);
    }else{
      leftOutput = m_leftPIDController.calculate(m_leftLeader.getSelectedSensorVelocity()*10/kEncoderTicksPerMeter, speeds.leftMetersPerSecond);
      rightOutput = m_rightPIDController.calculate(m_rightLeader.getSelectedSensorVelocity()*10/kEncoderTicksPerMeter, speeds.rightMetersPerSecond);
    }
    this.leftVolt=leftOutput + leftFeedforward;
    this.rightVolt=rightOutput + rightFeedforward;
    m_leftLeader.setVoltage(this.leftVolt);
    m_rightLeader.setVoltage(this.rightVolt);
    //System.out.printf("set speed %.2f,%.2f\n",this.leftVolt,this.rightVolt);
  }

  /**
   * Controls the robot using arcade drive.
   *
   * @param xSpeed the speed for the x axis
   * @param rot the rotation
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double rot) {
    setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
  }

  /** Update robot odometry. */
  public void updateOdometry() {
    if (Robot.isSimulation()){
      m_odometry.update(m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    }else{
      m_odometry.update(m_gyro.getRotation2d(), m_leftLeader.getSelectedSensorPosition()/kEncoderTicksPerMeter, m_rightLeader.getSelectedSensorPosition()/kEncoderTicksPerMeter);
    }
  }

  /** Resets robot odometry. */
  public void resetOdometry(Pose2d pose) {
    m_leftLeader.setSelectedSensorPosition(0);
    m_rightLeader.setSelectedSensorPosition(0);
    m_drivetrainSimulator.setPose(pose);
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
    if (Robot.isSimulation()){
      m_leftEncoder.reset();
      m_rightEncoder.reset();
    }
  }

  /** Check the current robot pose. */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /** Update our simulation. This should be run every robot loop in simulation. */
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the
    // simulation, and write the simulated positions and velocities to our
    // simulated encoder and gyro. We negate the right side so that positive
    // voltages make the right side move forward.
    m_drivetrainSimulator.setInputs(
        //m_leftLeader.getMotorOutputPercent() * RobotController.getInputVoltage(), -m_rightLeader.getMotorOutputPercent() * RobotController.getInputVoltage()
        this.leftVolt,this.rightVolt
        );
    //System.out.printf("get speed %.2f,%.2f\n", this.leftVolt,this.rightVolt);
    m_drivetrainSimulator.update(0.02);

    m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

    // this code updates the navX angle in Sim
    // m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_drivetrainSimulator.getHeading().getDegrees());
  }

  /** Update odometry - this should be run every robot loop. */
  public void periodic() {
    updateOdometry();
    tableYaw.setNumber(m_gyro.getRotation2d().getDegrees());
    tableLeftEncoder.setNumber(m_leftLeader.getSelectedSensorPosition());
    tableRightEncoder.setNumber(m_rightLeader.getSelectedSensorPosition());
    tableLeftVelocity.setNumber(m_leftLeader.getSelectedSensorVelocity() *10.0/ kEncoderTicksPerMeter);
    tableRightVelocity.setNumber(m_rightLeader.getSelectedSensorVelocity() *10.0/ kEncoderTicksPerMeter);
    m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
  }
}
