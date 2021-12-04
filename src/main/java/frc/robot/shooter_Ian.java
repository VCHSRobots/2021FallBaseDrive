// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;

import java.lang.System;

public class shooter_Ian {

  /** Creates a new shooter. */
  private final XboxController m_controller = new XboxController(0);
  private final SlewRateLimiter m_loadLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_filterLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter m_shootLimiter = new SlewRateLimiter(4);
  // DigitalInput toplimitSwitch = new DigitalInput(0);
  // DigitalInput bottomlimitSwitch = new DigitalInput(1);
  
  enum Direction {
    CW, CCW
  }
  double m_load_timestamp = 0.0;
  Direction direction = Direction.CW;
  

  // motor controllers
  private final WPI_TalonSRX m_filterMotor = new WPI_TalonSRX(20);
  private final WPI_TalonSRX m_turntableMotor = new WPI_TalonSRX(21);
  private final Talon m_loadMotor = new Talon(0); 
 
  private final WPI_TalonSRX m_tiltMotor = new WPI_TalonSRX(22);
  private final WPI_TalonSRX m_shootMotor = new WPI_TalonSRX(10);

  boolean isEnabled;
  double maxTimeEnabled;
  double minTimeEnabled;
  double currentTime;
  Timer timer;

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

  ShuffleboardTab TurnTableTab;
  NetworkTableEntry ntTurnTableOutput;

  ShuffleboardTab TurnTableMotorSupplyTab;
  NetworkTableEntry ntTurnTableMotorSupply;

  NetworkTableEntry ntShooterSpeed;

  

  public shooter_Ian() {

  }

  public void robotInit() {
    // MOTOR CONFIGS
    m_shootMotor.configFactoryDefault();
    m_tiltMotor.configFactoryDefault();
    m_filterMotor.configFactoryDefault();
    m_turntableMotor.configFactoryDefault();

    TalonSRXConfiguration base_config = new TalonSRXConfiguration();
    base_config.voltageCompSaturation = 10;
    base_config.continuousCurrentLimit = 12;
    base_config.peakCurrentLimit = 20;
    base_config.peakCurrentDuration = 500;
    
    m_shootMotor.configAllSettings(base_config);
    m_tiltMotor.configAllSettings(base_config);
    m_filterMotor.configAllSettings(base_config);
    m_turntableMotor.configAllSettings(base_config);


// SHUFFLEBOARD!!!!!!!!!!!!!!!!!!
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
    ntTurnTableOutput = ShootMotorTab.add("TurnTable Motor Output Percent", 0).withPosition(3, 1).withSize(2, 1)
        .withWidget(BuiltInWidgets.kDial).getEntry();
    ntTurnTableMotorSupply = ShootMotorTab.add("TurnTable Motor Supply Current", 0).withPosition(5, 1).withSize(2, 1)
        .withWidget(BuiltInWidgets.kVoltageView).getEntry();
        ntShooterSpeed = ShootMotorTab.add("set Shooter Speed", 0.5).withPosition(6, 1).withSize(2, 1).getEntry();
        ntShooterSpeed.setNumber(0.75);

        timer = new Timer();
        maxTimeEnabled = 0.9;
        minTimeEnabled = 0.0;
        currentTime = 0.0;
    

  }

  public void robotPeriodic() {
     ntShootMotorOutput.setNumber(m_shootMotor.getMotorOutputPercent());
     ntShootMotorSupply.setNumber(m_shootMotor.getSupplyCurrent());

    ntTiltMotorOutput.setNumber(m_tiltMotor.getMotorOutputPercent());
    ntTiltMotorSupply.setNumber(m_tiltMotor.getSupplyCurrent());

     ntFilterMotorOutput.setNumber(m_filterMotor.getMotorOutputPercent());
     ntFilterMotorSupply.setNumber(m_filterMotor.getSupplyCurrent());

     ntTurnTableOutput.setNumber(m_turntableMotor.getMotorOutputPercent());
     ntTurnTableMotorSupply.setNumber(m_turntableMotor.getSupplyCurrent());

  }

  public void autonomousInit() {

  }

  public void autonomousPeriodic() {

  }

  public void teleopInit() {
    currentTime = 0.0;
  }

  public void teleopPeriodic() {
    // SPEED!!!!!!!
    double shootSpeed = 0;
    double loaderSpeed = 0;
    double tiltSpeed = 0;
    double filterSpeed = 0;
    double turntableSpeed = 0;  

    // if the absolute value is greater that 0.01 then start the window motor;
    if (Math.abs(tiltSpeed) > 0.01) {
      isEnabled = true;
    } else if (Math.abs(tiltSpeed) < 0.01) {
      isEnabled = false;
    }

    // System.out.println(toplimitSwitch.get());
    // LOADER MOTOR !!!!!!!!!
    // if (m_controller.getTriggerAxis(Hand.kLeft) > 0.05) {
    // loaderSpeed = 0.4;
    // }
    // else{
    //   loaderSpeed = 0;
    // }
    
  // TILT MOTOR
    if(m_controller.getBackButton()){
      tiltSpeed = -0.3;
    }
    else if(m_controller.getStartButton()){
      tiltSpeed = 0.5;
    }
    else{
      tiltSpeed = 0;
  }
 
  //   if (m_controller.getXButton()) {
  //     filterSpeed = 0.25;
  //  }
   if(m_controller.getBButton()){
     filterSpeed = 0.25;
   }

   // TURN TABLE SPEED
   if(m_controller.getBumper(Hand.kLeft)){
     turntableSpeed = 0.3;
   }
   else if(m_controller.getBumper(Hand.kRight)){
     turntableSpeed = -0.3;
   }
   else{
     turntableSpeed = 0;
   }

    // SHOOT MOTOR
     if (Math.abs(m_controller.getTriggerAxis(Hand.kRight)) > 0.05) {
      // shootSpeed = Math.abs(m_controller.getTriggerAxis(Hand.kRight));
      shootSpeed = ntShooterSpeed.getNumber(0).doubleValue();

      // if(m_controller.getTriggerAxis(Hand.kLeft) > 0.05){
      // loaderSpeed = 0.4;
      // filterSpeed = -0.3;
      // }
        if(m_controller.getTriggerAxis(Hand.kLeft) > 0.05){
          loaderSpeed = 0.4;
          if(direction == Direction.CW){
            filterSpeed = -0.3;
            if(System.currentTimeMillis() > m_load_timestamp + 2000){
              m_load_timestamp = System.currentTimeMillis();
              direction = Direction.CCW;
            }
          }
          else if(direction == Direction.CCW){
            filterSpeed = 0.3;
            if(System.currentTimeMillis() > m_load_timestamp + 1000){
              m_load_timestamp = System.currentTimeMillis();
              direction = Direction.CW;
            }
          }
        }

    }


    if(m_controller.getAButton()){
      shootSpeed = -0.4;
    }

    // SET MOTOR TO DOUBLES !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    m_shootMotor.set(ControlMode.PercentOutput, -m_shootLimiter.calculate(shootSpeed));
    m_filterMotor.set(ControlMode.PercentOutput, m_filterLimiter.calculate(filterSpeed));
    m_loadMotor.set(-m_loadLimiter.calculate(loaderSpeed));
    m_tiltMotor.set(ControlMode.PercentOutput, tiltSpeed);
    m_turntableMotor.set(ControlMode.PercentOutput, turntableSpeed);

  }

  public void disabledInit() {

  }

  // private int normalize(double percent) {
  //   if (percent > 0.01) {
  //       return 1;
  //   } else if (percent < -0.01) {
  //       return -1;
  //   } else {
  //       return 0;
  //   }
}

// RIGHT TRIFGGER SHOOT
// LEFT BUTTON BUMPR LOAD
// START BACK IS TILT
// X AND B IS FILTER MOTOR
