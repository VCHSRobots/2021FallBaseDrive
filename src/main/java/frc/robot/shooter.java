// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class shooter {
  /** Creates a new shooter. */

  // xbox controller and rate limiter
  private final XboxController m_controller = new XboxController(0);
  private final double kRampTime = 0.25;
  private final double kMaxRPM = 1500;

  //  motor controllers
  private final WPI_TalonFX m_shootTalon = new WPI_TalonFX(21);  

  // these constants must be calculated with the robot configuration tool!!!
  private final SimpleMotorFeedforward m_shootFeedForward = new SimpleMotorFeedforward(0.2, 0.2, 0.02);

  // shooter system constants
  private final int kEncoderResolution = 2048;
  private final double kMotorToWheelGearRatio = 14.0/50.0;
  private final int k100msPerSecond = 10;

  // networktables
  private NetworkTableInstance instance = NetworkTableInstance.getDefault();
  private NetworkTable nt = instance.getTable("/shoot");
  public NetworkTableEntry ntNativeVelocity = nt.getEntry("Native Vel");
  public NetworkTableEntry ntNativeVelocityTarget = nt.getEntry("Native Vel Target");
  public NetworkTableEntry ntShootPosition = nt.getEntry("Shoot Encoder Position");
  public NetworkTableEntry ntFeedForward = nt.getEntry("Feed Forward");

  public shooter() {
    // motor config
    TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
    shooterConfig.voltageCompSaturation = 11.0;
    shooterConfig.nominalOutputForward = 0;
    shooterConfig.nominalOutputReverse = 0;
    shooterConfig.peakOutputForward = 0.5;
    shooterConfig.peakOutputReverse = -0.5;
    shooterConfig.slot0.kF = 0.0;
    shooterConfig.slot0.kP = 0.0;
    shooterConfig.slot0.kI = 0.0;
    shooterConfig.slot0.kD = 0.0;
    shooterConfig.closedloopRamp = kRampTime;
    shooterConfig.openloopRamp = kRampTime;
    shooterConfig.neutralDeadband = 0.005;

    m_shootTalon.configFactoryDefault();
    m_shootTalon.setSensorPhase(false);
    m_shootTalon.setInverted(InvertType.None);
    m_shootTalon.setNeutralMode(NeutralMode.Coast);
    m_shootTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 100);
    m_shootTalon.configAllSettings(shooterConfig);
    
  }

  public void robotPeriodic() {
    ntNativeVelocity.setNumber(m_shootTalon.getSelectedSensorVelocity());

  }

  public void autonomousInit() {

  }

  public void autonomousPeriodic() {

  }

  public void teleopInit() {

  }

  public void teleopPeriodic() {
    if (m_controller.getAButton()) {
      double speed = 0.3;
      m_shootTalon.set(ControlMode.PercentOutput, speed);
      return;
    }

    double rightTrigger = m_controller.getTriggerAxis(Hand.kRight);
    if (Math.abs(rightTrigger) > 0.05 ) {
      double targetRPM = rightTrigger * kMaxRPM;
      setRPM(targetRPM);
      return;
    }

    m_shootTalon.neutralOutput();
    
  }

  public void disabledInit() {

  }

  private double nativeVelocityToWheelRPM(double encoderCountsPer100ms) {
    double motorRevPerSec = encoderCountsPer100ms * k100msPerSecond;
    double wheelRevPerSec = motorRevPerSec / kMotorToWheelGearRatio;
    double wheelRevPerMin = wheelRevPerSec * 60.0;
    return wheelRevPerMin;
  }

  private int wheelRPMToNativeUnitsPer100ms(double wheelRPM){
    double wheelRevPerSec = wheelRPM / 60.0;
    double motorRevPerSec = wheelRevPerSec * kMotorToWheelGearRatio;
    int nativeUnitsPer100ms = (int) (motorRevPerSec * kEncoderResolution / k100msPerSecond);
    return nativeUnitsPer100ms;
  }

  private void setRPM(double targetRPM) {
    int nativeVelocity = wheelRPMToNativeUnitsPer100ms(targetRPM);
    int arbFeedForward = (int) m_shootFeedForward.calculate(targetRPM);
    m_shootTalon.set(ControlMode.Velocity, nativeVelocity, DemandType.ArbitraryFeedForward, arbFeedForward);
    ntNativeVelocityTarget.setNumber(nativeVelocity);
    ntFeedForward.setNumber(arbFeedForward);
  }
}
