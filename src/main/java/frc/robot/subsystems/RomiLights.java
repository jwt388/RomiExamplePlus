// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.romi.OnBoardIO;
import edu.wpi.first.wpilibj.romi.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RomiLights extends SubsystemBase {

  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.OUTPUT, ChannelMode.OUTPUT); // Enable LEDs
  private Timer lightTimer;
  private Boolean blinkLight;
  private Boolean lightOff;

  /** Creates a new RomiLights. */
  public RomiLights() {

    m_onboardIO.setGreenLed(false);
    m_onboardIO.setRedLed(false);
    m_onboardIO.setYellowLed(true);

    // Start a timer to control the blinking light
    lightTimer = new Timer();
    lightTimer.start();
    blinkLight = false;
    lightOff = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    if (lightTimer.hasElapsed(0.5)) {
      lightTimer.restart();
      lightOff = !lightOff;
    }
    
    m_onboardIO.setYellowLed(!(blinkLight && lightOff));

  }

  public void enableBlink() {
    blinkLight = true;
  }

  public void disableBlink() {
    blinkLight = false;
  }

  public void setGreen(boolean state) {
    m_onboardIO.setGreenLed(state);
  }
}
