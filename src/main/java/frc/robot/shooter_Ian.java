// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.SlewRateLimiter;
public class shooter_Ian {
  /** Creates a new shooter. */
    private final XboxController m_controller = new XboxController(0);
    private final SlewRateLimiter m_speedLimiter = new SlewRateLimiter(4);
  //  motor controllers
    private final WPI_TalonFX m_Shooter = new WPI_TalonFX(21);
    private final WPI_TalonFX m_filterMotor = new WPI_TalonFX(22);
    

  public shooter_Ian() {
    
  }

  public void robotPeriodic() {

  }

  public void autonomousInit() {

  }

  public void autonomousPeriodic() {



  }

  public void teleopInit() {

  }

  public void teleopPeriodic() {
    boolean isPressed = m_controller.getAButton();
    double speed = 0;
    double loaderSpeed = 0;

    if(m_controller.getXButton()){
      loaderSpeed = 0.1;

    }

    if(isPressed){
      speed = 0.3;
    }
    else{
    speed = 0;
    }

   m_Shooter.set(ControlMode.PercentOutput, m_speedLimiter.calculate(speed));
   m_filterMotor.set(ControlMode.PercentOutput, loaderSpeed);

  }

  public void disabledInit() {

  }
}