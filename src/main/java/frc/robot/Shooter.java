
package frc.robot;

import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

public class Shooter {

  private ShuffleboardTab shooterTab = Shuffleboard.getTab("shooter");
  private final SendableChooser<String> motorOutputList = new SendableChooser<>();

  public NetworkTableEntry ntMotorModeSp;
  public NetworkTableEntry ntTiltControllerPercentage;
  public NetworkTableEntry ntMotorSpOutput;
  public NetworkTableEntry ntMotorShOutput;
  public NetworkTableEntry ntSelectedMotorOutput;
  public NetworkTableEntry ntSelectedReader;
  public NetworkTableEntry ntButtonPressed;
  public NetworkTableEntry ntTiltPlace;
  public NetworkTableEntry ntShooterSpeed;
  public NetworkTableEntry ntShooterTime;

  private final XboxController m_controller = new XboxController(0);
  private final SlewRateLimiter m_spinRateLimiter = new SlewRateLimiter(0.5);
  private final SlewRateLimiter m_shootRateLimiter = new SlewRateLimiter(2);
  private final SlewRateLimiter m_tiltRateLimiter = new SlewRateLimiter(0.5);

  private final WPI_TalonSRX m_spin = new WPI_TalonSRX(10);
  private final WPI_TalonSRX m_shoot = new WPI_TalonSRX(5);
  private final WPI_TalonSRX m_tilt = new WPI_TalonSRX(6);

  private final double positiveTiltMax = 11;
  private String motorMode = "stopped";
  private double tiltPlace = 0;
  double m_loadTimeStamp;
  public void robotInit() {

    m_shoot.setInverted(true);

    motorOutputList.setDefaultOption("mSpin Output", "mSpin");
    motorOutputList.addOption("mShoot Output", "mShoot");

    shooterTab.add("Motor Output List", motorOutputList).withSize(2, 1).withPosition(3, 3);
    ntSelectedMotorOutput = shooterTab.add("sMotor Output", 0).withSize(1, 1).withPosition(5, 3).getEntry();

    ntMotorModeSp = shooterTab.add("Motor Mode Sp", "N/A").withSize(1, 1).withPosition(1, 3).getEntry();
    ntTiltControllerPercentage = shooterTab.add("tPercntge Power", 0).withSize(1, 1).withPosition(2, 3).getEntry();
    ntMotorSpOutput = shooterTab.add("mSpin Output", 0).withSize(1, 1).withPosition(1, 1).getEntry();
    ntMotorShOutput = shooterTab.add("mShoot Output", 0).withSize(1, 1).withPosition(2, 1).getEntry();

    ntTiltPlace = shooterTab.add("TiltPlace", tiltPlace).withSize(1, 1).withPosition(4, 1).getEntry();

    ntShooterSpeed = shooterTab.add("set Shooter Speed", 0.5).withSize(1, 1).withPosition(3, 1).getEntry();
    ntShooterTime = shooterTab.add("Shooter Time millis", 100).withSize(1, 1).withPosition(4, 1).getEntry();

  }

  public void robotPeriodic() {

    // ntMotorSpOutput.setNumber(m_spin.get());
    // ntMotorShOutput.setNumber(m_shoot.getMotorOutputPercent());
    // if(motorOutputList.getSelected() == "mSpin") {
    // ntSelectedMotorOutput.setNumber(m_spin.get());
    // } else if(motorOutputList.getSelected() == "mShoot") {
    // ntSelectedMotorOutput.setNumber(m_shoot.getMotorOutputPercent());
    // } else {

    // }
    // ntTiltPlace.setNumber(tiltPlace);
    // if(m_tilt.getMotorOutputPercent() == 0) {
    // ntTiltControllerPercentage.setDouble(0);
    // } else {
    // ntTiltControllerPercentage.setDouble(-m_tilt.getMotorOutputPercent());
    // }

  }

  // public void autoSpin () {
  // if(colorSensor.ntDistanceClose == true) {
  // m_spin.set(-m_spinRateLimiter.calculate(.15));
  // } else {
  // m_spin.set(-m_spinRateLimiter.calculate(0));
  // }
  // }

  public void shoot() {
    
    /*
     * if (m_controller.getAButton()) { m_spin.set(-1); } else { m_spin.set(0); }
     */
    if (m_controller.getBumper(Hand.kRight)) {
      Date date = new Date();

      if (m_controller.getAButtonPressed()) {
          m_loadTimeStamp = date.getTime(); // in millis
      }
      m_shoot.set(ControlMode.PercentOutput, (double) ntShooterSpeed.getNumber(0));

      if (date.getTime() < m_loadTimeStamp + ntShooterTime.getNumber(0).doubleValue()) { //millis
        m_spin.set(-0.75);

      } else {

        m_spin.set(0);
      }
    } else if (m_controller.getXButton()) {
      // pequeno
      m_shoot.set(.25);
      m_spin.set(-1);

      motorMode = "running";
    } else if (m_controller.getYButton()) {
      // mediano
      m_shoot.set(.50);
      m_spin.set(-1);

      motorMode = "running";
    } else if (m_controller.getBButton()) {
      // MUY GORDO
      m_shoot.set(1);
      motorMode = "running";
      m_spin.set(-1);

    } else {
      m_shoot.set(0);
      motorMode = "stopped";
      m_spin.set(0);

    }

    if (m_controller.getStartButton()) {

      m_spin.set(.5);
    }


    ntMotorModeSp.setString(motorMode);

  }

  public void tilt() {

    double tiltSpeed;
    double temp_axis;

    if (m_controller.getRawAxis(3) > 0.02) {
      temp_axis = m_controller.getRawAxis(3);
      tiltSpeed = m_tiltRateLimiter.calculate(temp_axis) / 4;
      if (tiltSpeed < .1) {
        tiltSpeed = 0;
      }
    } else if (m_controller.getRawAxis(2) > 0.02) {
      temp_axis = m_controller.getRawAxis(2);
      tiltSpeed = -m_tiltRateLimiter.calculate(temp_axis) / 4;
    } else {
      tiltSpeed = 0;
    }

    if (tiltSpeed == 0) {
      m_tilt.set(.1);
    } else {
      m_tilt.set(tiltSpeed);
    }

  }

  public void teleopPeriodic() {
    // shoots
    shoot();
    // tilts
    tilt();
    // loading motor spin
    // m_spin.set(-1);
  }

}
