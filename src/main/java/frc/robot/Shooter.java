// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Controller;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/** Add your docs here. */
public class Shooter {

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("shooter");
  private final SendableChooser<String> motorOutputList = new SendableChooser<>(); 

  public NetworkTableEntry ntMotorModeSp;
  public NetworkTableEntry ntTiltMotorPower;
  public NetworkTableEntry ntMotorSpOutput;
  public NetworkTableEntry ntMotorShOutput;
  public NetworkTableEntry ntSelectedMotorOutput;
  public NetworkTableEntry ntSelectedReader;
  public NetworkTableEntry ntButtonPressed;

  private final XboxController m_controller = new XboxController(0);
  private final SlewRateLimiter m_spinRateLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_shootRateLimiter = new SlewRateLimiter(5);
  private final SlewRateLimiter m_tiltRateLimiter = new SlewRateLimiter(5);

  private final WPI_TalonSRX m_spin = new WPI_TalonSRX(1);
  private final WPI_TalonSRX m_shoot = new WPI_TalonSRX(14);
  private final WPI_TalonSRX m_tilt = new WPI_TalonSRX(16);

  private String spinMotorMode = "stopped";

  public void robotInit () {

    motorOutputList.setDefaultOption("mSpin Output", "mSpin");
    motorOutputList.addOption("mShoot Output", "mShoot");
 
    shooterTab.add("Motor Output List", motorOutputList)
    .withSize(2,1)
    .withPosition(3, 3);
    ntSelectedMotorOutput = shooterTab.add("sMotor Output", 0)
    .withSize(1, 1)
    .withPosition(5, 3).getEntry();

    ntMotorModeSp = shooterTab.add("Motor Mode Sp", spinMotorMode)
    .withSize(1, 1)
    .withPosition(1, 3).getEntry();
    ntTiltMotorPower = shooterTab.add("Tilt Motor Power", 0)
    .withSize(1, 1)
    .withPosition(2, 3).getEntry();
    ntMotorSpOutput = shooterTab.add("mSpin Output", 0)
    .withSize(1, 1) 
    .withPosition(1, 1).getEntry(); 
    ntMotorShOutput = shooterTab.add("mShoot Output", 0)
    .withSize(1, 1) 
    .withPosition(2, 1).getEntry(); 

  }

  public void robotPeriodic() {

    ntMotorSpOutput.setNumber(m_spin.getMotorOutputPercent());
    ntMotorShOutput.setNumber(m_shoot.getMotorOutputPercent());
    ntTiltMotorPower.setDouble(m_tilt.getMotorOutputPercent());
    if(motorOutputList.getSelected() == "mSpin") {
      ntSelectedMotorOutput.setNumber(m_spin.getMotorOutputPercent());
    } else if(motorOutputList.getSelected() == "mShoot") {
      ntSelectedMotorOutput.setNumber(m_shoot.getMotorOutputPercent());
    } else {

    }

  }

  public void teleopPeriodic() {
    //spins motor
    spin();
    //tilts
    tilt();
  }

  public void spin () {

    double spinSpeed;
    double shootSpeed;

    if(m_controller.getAButtonPressed()) {
      if(spinMotorMode == "stopped") {
        spinSpeed = -m_spinRateLimiter.calculate(.15);
        shootSpeed = -m_shootRateLimiter.calculate(.30);
        spinMotorMode = "running";
        ntMotorModeSp.setString(spinMotorMode);
      } else {
        spinSpeed = -m_spinRateLimiter.calculate(0);
        shootSpeed = -m_shootRateLimiter.calculate(0);
        spinMotorMode = "stopped";
        ntMotorModeSp.setString(spinMotorMode);
      }
      m_spin.set(spinSpeed);
      m_shoot.set(shootSpeed); 
    }

  }

  public void tilt () {

    double tiltSpeed;
    double temp_axis;

    if(m_controller.getRawAxis(3) > 0.01) {
      temp_axis = m_controller.getRawAxis(3);
      tiltSpeed = -m_tiltRateLimiter.calculate(temp_axis);
    } else {
      temp_axis = m_controller.getRawAxis(2);
      tiltSpeed = -m_tiltRateLimiter.calculate(temp_axis);
    }

    m_tilt.set(tiltSpeed);

  }

}
