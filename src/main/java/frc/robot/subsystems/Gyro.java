// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  private Pigeon2 m_pigeon;
  private AHRS m_navX;
  private boolean m_usePigeon;

  public Gyro(boolean usePigeon) {
    m_pigeon = new Pigeon2(Constants.Swerve.pigeonID);
    m_pigeon.getConfigurator().apply(new Pigeon2Configuration()); // replaces .configFactoryDefault()
    m_navX = new AHRS(SPI.Port.kMXP);
    // m_usePigeon = Constants.Swerve.usePigeon;
    m_usePigeon = usePigeon;
  }

  public void setYaw(int pos){
    m_pigeon.setYaw(pos);
    setZeroAngleDegreesNavX(0.0);
    m_navX.zeroYaw();
    //navX.resetDisplacement(); //may break odometry on navX, idk :) we dont know why we need this, but dallion had it in his code
  }

  public double getYaw(){
    if (m_usePigeon) {
      return m_pigeon.getYaw().getValueAsDouble();
    } else {
      return -m_navX.getYaw();
    }
  }
  
  public double getRoll(){
    if (m_usePigeon) {
      return m_pigeon.getRoll().getValueAsDouble();
    } else {
      return m_navX.getRoll();
    }
  }
  
  public double getPitch(){
    if (m_usePigeon) {
      return m_pigeon.getPitch().getValueAsDouble();
    } else {
      return m_navX.getPitch();
    }
  }

  public void toggleGyro(){
    m_usePigeon = !(m_usePigeon);
  }

  private void setZeroAngleDegreesNavX (double degrees) {
    int nTries = 1;
    while (m_navX.isCalibrating()) { //wait to zero yaw if calibration is still running
      try {
        Thread.sleep(20);
        System.out.println("----calibrating gyro---- " + nTries);
      } catch (InterruptedException e) {

      }
      nTries++;
      if (nTries >= 50 && nTries%10==0) {
        System.out.println("Having trouble calibrating NavX");
      }
    }
    System.out.println("Setting angle adj to " + (-m_navX.getYaw()) + " + " + degrees + " after " + nTries + " attempts");
    m_navX.setAngleAdjustment(-m_navX.getYaw() + degrees);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double navXYaw = -m_navX.getYaw();
    var rawPigeonYaw = m_pigeon.getYaw();
    double pigeonYaw = rawPigeonYaw.getValueAsDouble();
    var pigeonStatus = rawPigeonYaw.getStatus();
    SmartDashboard.putNumber("Pigeon Yaw", pigeonYaw);
    SmartDashboard.putNumber("NavX Yaw", navXYaw);
    SmartDashboard.putNumber("Gyro Differnce", Math.abs(pigeonYaw + navXYaw));

    var gyroError = m_pigeon.getFaultField();
    SmartDashboard.putString("Pigeon Error Status", pigeonStatus.toString());
    SmartDashboard.putString("Pigeon Fault Field", gyroError.toString());
    SmartDashboard.putString("rawPigeonYaw", rawPigeonYaw.toString());

    SmartDashboard.putBoolean("Using Pigeon?", m_usePigeon);

    if (false) if (!gyroError.toString().equals("OK")) {
      if (m_usePigeon) {
        System.out.println("Pigeon Error: defaulting back to navX");
      }
      m_usePigeon = false;
      //System.out.println(gyroError.toString());
    }
  }
}

