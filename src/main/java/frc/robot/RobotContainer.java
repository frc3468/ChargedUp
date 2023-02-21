// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autos.BaseAuto;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final XboxController primaryDriver = new XboxController(0);
  private final Joystick CoDriverJ1 = new Joystick(1);
  private final Joystick CoDriverJ2 = new Joystick(2);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            () -> -primaryDriver.getRawAxis(translationAxis),
            () -> -primaryDriver.getRawAxis(strafeAxis),
            () -> -primaryDriver.getRawAxis(rotationAxis),
            () -> robotCentric.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
  }

   /* Subsystems */
   private final SwerveDrive s_Swerve = new SwerveDrive();
  
   /* End Subsystems */

  /* Joystick and Controller assignments */
  /* Driver Buttons - Xbox Controller */
  /* Analog Sticks */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  // button assignments
  private final JoystickButton getGyroReading = new JoystickButton(primaryDriver, XboxController.Button.kX.value); // get GyroPosition
  private final JoystickButton zeroGyro = new JoystickButton(primaryDriver, XboxController.Button.kY.value);  // reset GyroPosition
  private final JoystickButton FieldDrive = new JoystickButton(primaryDriver, XboxController.Button.kLeftBumper.value); // Set to Field Centric
  private final JoystickButton robotCentric = new JoystickButton(primaryDriver, XboxController.Button.kRightBumper.value);   // Set to Robot Centric
  /* Co-Driver Buttons - Dual Joysticks */
  private final double OuterArmAxis = Joystick.AxisType.kY.value;
  private final double InnerArmAxis = Joystick.AxisType.kY.value;
//TODO dual joystick commands to be entered here
 
  /* Button Bindings - Actions taken upon button press or hold */
  private void configureButtonBindings() { 
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); //Y
    getGyroReading.onTrue(new InstantCommand(() -> s_Swerve.GetGyroReading()));  //X
    robotCentric.onTrue(new InstantCommand(() -> s_Swerve.SetRobotCentric()));  //LEFT BUMPER
    FieldDrive.onTrue(new InstantCommand(() -> s_Swerve.SetFieldDrive()));  //RIGHT BUMPER
    
  }
   /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new exampleAuto(s_Swerve);
    return new BaseAuto(); // TODO place holder for now, replace once we have auto modes
  }

}
