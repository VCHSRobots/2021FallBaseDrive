// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Talon;

public class shooter_Ian {

  /** Creates a new shooter. */
  private final XboxController m_controller = new XboxController(0);
  private final SlewRateLimiter m_loadLimiter = new SlewRateLimiter(4);
 // private final SlewRateLimiter m_filterLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_shootLimiter = new SlewRateLimiter(4);
  DigitalInput toplimitSwitch = new DigitalInput(0);
  DigitalInput bottomlimitSwitch = new DigitalInput(1);

  // motor controllers
 // private final WPI_TalonSRX m_filterMotor = new WPI_TalonSRX(21);
  private final WPI_TalonSRX m_turntableMotor = new WPI_TalonSRX(21);
  private final Talon m_loadMotor = new Talon(0); 
  private final WPI_TalonSRX m_tiltMotor = new WPI_TalonSRX(22);
  private final WPI_TalonSRX m_shootMotor = new WPI_TalonSRX(10);
  


  ShuffleboardTab ShootMotorTab;
  NetworkTableEntry ntShootMotorOutput;

  ShuffleboardTab ShootMotorSupplyTab;
  NetworkTableEntry ntShootMotorSupply;

  ShuffleboardTab LoadMotorTab;
  NetworkTableEntry ntLoadMotorOutput;

  ShuffleboardTab LoadMotorSupplyTab;
  NetworkTableEntry ntLoadMotorSupply;

  ShuffleboardTab TiltMotorTab;
  NetworkTableEntry ntTiltMotorOutput;

  ShuffleboardTab TiltMotorSupplyTab;
  NetworkTableEntry ntTiltMotorSupply;

  ShuffleboardTab FilterMotorTab;
  NetworkTableEntry ntFilterMotorOutput;

  ShuffleboardTab FilterMotorSupplyTab;
  NetworkTableEntry ntFilterMotorSupply;

  public shooter_Ian() {

  }

  public void robotInit() {
    ShootMotorTab = Shuffleboard.getTab("Shooter Output");
    ntShootMotorOutput = ShootMotorTab.add("Shoot Motor Output Percent", 0).withPosition(3, 4).withSize(2, 1)
        .withWidget(BuiltInWidgets.kDial).getEntry();
    ntShootMotorSupply = ShootMotorTab.add("Shoot Motor Supply Current", 0).withPosition(5, 4).withSize(2, 1)
        .withWidget(BuiltInWidgets.kVoltageView).getEntry();
    ntLoadMotorOutput = ShootMotorTab.add("Load Motor Output Percent", 0).withPosition(3, 3).withSize(2, 1)
        .withWidget(BuiltInWidgets.kDial).getEntry();
    ntLoadMotorSupply = ShootMotorTab.add("Loader Motor Supply Current", 0).withPosition(5, 3).withSize(2, 1)
        .withWidget(BuiltInWidgets.kVoltageView).getEntry();
    ntTiltMotorOutput = ShootMotorTab.add("Tilter Motor Output Percent", 0).withPosition(3, 2).withSize(2, 1)
        .withWidget(BuiltInWidgets.kDial).getEntry();
    ntTiltMotorSupply = ShootMotorTab.add("Tilter or Supply Current", 0).withPosition(5, 2).withSize(2, 1)
        .withWidget(BuiltInWidgets.kVoltageView).getEntry();
    ntFilterMotorOutput = ShootMotorTab.add("Filter Motor Output Percent", 0).withPosition(3, 1).withSize(2, 1)
        .withWidget(BuiltInWidgets.kDial).getEntry();
    ntFilterMotorSupply = ShootMotorTab.add("Filter Motor Supply Current", 0).withPosition(5, 1).withSize(2, 1)
        .withWidget(BuiltInWidgets.kVoltageView).getEntry();
    
    

  }

  public void robotPeriodic() {
    // ntShootMotorOutput.setNumber(m_shootMotor.getMotorOutputPercent());
    // ntShootMotorSupply.setNumber(m_shootMotor.getSupplyCurrent());

    ntTiltMotorOutput.setNumber(m_tiltMotor.getMotorOutputPercent());
    ntTiltMotorSupply.setNumber(m_tiltMotor.getSupplyCurrent());

    // ntLoadMotorOutput.setNumber(m_loadMotor.getMotorOutputPercent());
    // ntLoadMotorSupply.setNumber(m_loadMotor.getSupplyCurrent());

    // ntFilterMotorOutput.setNumber(m_filterMotor.getMotorOutputPercent());
    // ntFilterMotorSupply.setNumber(m_filterMotor.getSupplyCurrent());

    /*
     * SmartDashboard.putNumber("Shoot Motor Output Percent",
     * m_shootMotor.getMotorOutputPercent());
     * SmartDashboard.putNumber("Shoot Motor Supply Current",
     * m_shootMotor.getSupplyCurrent());
     * 
     * SmartDashboard.putNumber("Tilt Motor Output Percent",
     * m_tiltMotor.getMotorOutputPercent());
     * SmartDashboard.putNumber("Tilt Motor Supply Current",
     * m_tiltMotor.getSupplyCurrent());
     * 
     * SmartDashboard.putNumber("Load Motor Output Percent",
     * m_loadMotor.getMotorOutputPercent());
     * SmartDashboard.putNumber("Load Motor Supply Current",
     * m_loadMotor.getSupplyCurrent());
     * 
     * SmartDashboard.putNumber("Filter Motor Output Percent",
     * m_filterMotor.getMotorOutputPercent());
     * SmartDashboard.putNumber("Filter Motor Supply Current",
     * m_filterMotor.getSupplyCurrent());
     */
  }

  public void autonomousInit() {

  }

  public void autonomousPeriodic() {

  }

  public void teleopInit() {

  }

  public void teleopPeriodic() {
    double shootSpeed = 0;
    double loaderSpeed = 0;
    boolean loaderToggle = false;
    double tiltSpeed = 0;
   // double filterSpeed = 0;
   double turntableSpeed = 0;
   
  
   

   // System.out.println(toplimitSwitch.get());
    if (m_controller.getTriggerAxis(Hand.kLeft) > 0.05) {
    loaderSpeed = 0.40;
    }
    else{
      loaderSpeed = 0;
    }
    
  
    if(m_controller.getBackButton()){
      tiltSpeed = -0.2;
    }
    else if(m_controller.getStartButton()){
      tiltSpeed = 0.3;
    }
    else{
      tiltSpeed = 0;
  }
 
  /*  if (m_controller.getYButton()) {
      filterSpeed = 0.1;
   } */

   // tilt start & end
   if(m_controller.getBumper(Hand.kLeft)){
     turntableSpeed = -0.25;
   }
   else if(m_controller.getBumper(Hand.kRight)){
     turntableSpeed = 0.25;
   }
   else{
     turntableSpeed = 0;
   }
/*
    if (toplimitSwitch.get() == false) {
      tiltSpeed = -0.25;
    } else if (toplimitSwitch.get()) {
      if (m_controller.getBumper(Hand.kLeft)) {
        tiltSpeed = -0.5;
      } else if (m_controller.getBumper(Hand.kRight)) {
        tiltSpeed = 0.5;
      }
    }

    if (bottomlimitSwitch.get() == false) {
      tiltSpeed = 0.25;
    } else if (bottomlimitSwitch.get()) {

      if (m_controller.getBumper(Hand.kLeft)) {
        tiltSpeed = -0.5;
      } else if (m_controller.getBumper(Hand.kRight)) {
        tiltSpeed = 0.5;
      }

    } else {
      tiltSpeed = 0;
    }
    */
    if (Math.abs(m_controller.getTriggerAxis(Hand.kRight)) > 0.05) {
      shootSpeed = Math.abs(m_controller.getTriggerAxis(Hand.kRight));
    }
    if(m_controller.getAButton()){
      shootSpeed = -0.2;
    }
    m_shootMotor.set(ControlMode.PercentOutput, -m_shootLimiter.calculate(shootSpeed));
    //m_filterMotor.set(ControlMode.PercentOutput, m_filterLimiter.calculate(filterSpeed));
    m_loadMotor.set(-m_loadLimiter.calculate(loaderSpeed));
    m_tiltMotor.set(ControlMode.PercentOutput, tiltSpeed);
    m_turntableMotor.set(ControlMode.PercentOutput, turntableSpeed);

  }

  public void disabledInit() {

  }
}