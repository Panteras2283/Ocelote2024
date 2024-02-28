// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalOutput;

public class LD extends SubsystemBase {
  private DigitalOutput btn1 = new DigitalOutput(2);  
  private DigitalOutput btn2 = new DigitalOutput(3);  
  private DigitalOutput btn3 = new DigitalOutput(4);

  /** Creates a new LD. */
  public LD() {}

  public void mode1() {
    btn1.set(false);
    //btn1.set(true);
  }
  public void mode2() {
    btn2.set(false);
    //btn2.set(true);
  }
  public void mode3() {
    btn3.set(false);
    btn3.set(true);
  }
  public void idle() {
    btn1.set(true);
    btn2.set(true);
    btn3.set(true);
    //btn3.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
