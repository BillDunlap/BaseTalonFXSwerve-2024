// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * A wrapper for the DigitalInput class.  You can specify whether activated means the switch
 * is closed or open.  It also sends its state (isActivated) to SmartDashboard, tagged with its name.
 */
public class OnOffSwitch extends SubsystemBase {
  private final int m_channel;
  private final boolean m_normallyOpen;
  private final String m_name;
  private final DigitalInput m_digitalInput;

  /** Creates a new OnOffSwitch.
   * channel - the DIO channel for the digital input 0-9 are on the RoboRIO, 10-25 are on the MXP
   * normallyOpen - true if the circuit is open except when activated, false if it is closed except when activated
   * name - a name for sending data to the smartdashboard
   * 
   * See https://docs.wpilib.org/en/stable/docs/hardware/sensors/digital-inputs-hardware.html and
   * https://www.chiefdelphi.com/t/wiring-help-with-ada-fruit-ir-beam-break-sensors-in-dio/405318
   * for how to wire these things.
   */
  public OnOffSwitch(int channel, boolean normallyOpen, String name) {
    m_channel = channel;
    m_digitalInput = new DigitalInput(m_channel);
    m_normallyOpen = normallyOpen;
    m_name = name;
  }

  public boolean isActivated(){
    return m_normallyOpen ? m_digitalInput.get() : !m_digitalInput.get();
  }

  public String getName(){
    return m_name;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean(m_name + " (OnOffSwitch " + m_channel + ")", isActivated());
  }
}
