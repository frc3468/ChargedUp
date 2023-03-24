// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.config.OnboardModuleStates;
import frc.robot.Constants.Clawconstants;

public class LEDs extends SubsystemBase {

  
  private final Solenoid LedWhite = new Solenoid(
    Clawconstants.ModuleID,
    PneumaticsModuleType.REVPH,
    Clawconstants.LedWhite
  );
  private final Solenoid LedRed = new Solenoid(
    Clawconstants.ModuleID,
    PneumaticsModuleType.REVPH,
    Clawconstants.LedRed
  );
  private final Solenoid LedGreen = new Solenoid(
    Clawconstants.ModuleID,
    PneumaticsModuleType.REVPH,
    Clawconstants.LedGreen
  );
  private final Solenoid LedBlue = new Solenoid(
    Clawconstants.ModuleID,
    PneumaticsModuleType.REVPH,
    Clawconstants.LedBlue
  );

  /** Creates new LED channels. */

  public void WhiteLedOn() {
    LedWhite.set(true);
  }

  public void RedLedOn() {
    LedRed.set(true);
  }

  public void GreenLedOn() {
    LedGreen.set(true);
  }

  public void BlueLedOn() {
    LedBlue.set(true);
  }

  public void WhiteLedOff() {
    LedWhite.set(false);
  }

  public void RedLedOff() {
    LedRed.set(false);
  }

  public void GreenLedOff() {
    LedGreen.set(false);
  }

  public void BlueLedOff() {
    LedBlue.set(false);
  }


  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("Intake Sensor", !lazerSensor.get());
    // This method will be called once per scheduler run
  }
}
