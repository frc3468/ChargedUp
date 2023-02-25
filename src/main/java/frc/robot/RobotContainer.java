// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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

  private final InnerArm m_InnerArm = new InnerArm();
  private final OuterArm m_OuterArm = new OuterArm();

  // The robot's subsystems and commands are defined here...
  private final XboxController primaryDriver = new XboxController(0);

  /* Joystick and Controller assignments */
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;

  //
  /* Driver Buttons - Xbox Controller */
  // ABXY buttons
  // A - Bottom shelf position
  //private final JoystickButton eTier = new JoystickButton(primaryDriver, XboxController.Button.kA.value);
  // B - Middle shelf position
  //private final JoystickButton midTier = new JoystickButton(primaryDriver, XboxController.Button.kB.value);
  // Y - Top shelf position
 // private final JoystickButton sTeir = new JoystickButton(primaryDriver, XboxController.Button.kY.value);
  // X - toggle between robot- and field-centric - true is robot-centric
  // private final JoystickButton centricToggle = new JoystickButton(primaryDriver, XboxController.Button.kX.value);
  // X - Home position
  //private final JoystickButton home = new JoystickButton(primaryDriver, XboxController.Button.kX.value);
  private final JoystickButton outerRaise = new JoystickButton(primaryDriver, XboxController.Button.kY.value);
  private final JoystickButton outerLower = new JoystickButton(primaryDriver, XboxController.Button.kA.value);
  private final JoystickButton innerRaise = new JoystickButton(primaryDriver, XboxController.Button.kX.value);
  private final JoystickButton innerLower = new JoystickButton(primaryDriver, XboxController.Button.kB.value);

  // BACK/SELECT - Zero Gyro reading
  private final JoystickButton zeroGyro = new JoystickButton(primaryDriver, XboxController.Button.kBack.value);
  // START - Hard brake/ auto-level for balance board
  private final JoystickButton autoLevel = new JoystickButton(primaryDriver, XboxController.Button.kStart.value);

  // Bumpers and Triggers
  // Left Bumper - auto-load
  private final JoystickButton autoLoad = new JoystickButton(primaryDriver, XboxController.Button.kLeftBumper.value);
  // Right bumper - Open/Close the claw
  private final JoystickButton clawMovement = new JoystickButton(primaryDriver, XboxController.Button.kRightBumper.value);

  /* Co-Driver Buttons - Dual Joysticks */
  private final double OuterArmAxis = Joystick.AxisType.kY.value;
  private final double InnerArmAxis = Joystick.AxisType.kY.value;

  /* End of Joystick and Controller assignments */

  /* Subsystems */
  private final SwerveDrive s_Swerve = new SwerveDrive();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

  //  s_Swerve.setDefaultCommand(
  //      new TeleopSwerve(
  //        s_Swerve,
  //          () -> -primaryDriver.getRawAxis(translationAxis),
  //          () -> -primaryDriver.getRawAxis(strafeAxis),
  //          () -> -primaryDriver.getRawAxis(rotationAxis)
  //          () -> centricToggle.getAsBoolean()));

    // Configure the button bindings
    configureButtonBindings();
  }
  
   /* End Subsystems */

  /* Button Bindings - Actions taken upon button press or hold */
  private void configureButtonBindings() { 
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); //Y
   // centricToggle.onTrue(new InstantCommand(() -> s_Swerve.GetGyroReading()));  //X
    autoLoad.onTrue(new InstantCommand(() -> s_Swerve.SetRobotCentric()));  //LEFT BUMPER
    clawMovement.onTrue(new InstantCommand(() -> s_Swerve.SetFieldDrive()));  //RIGHT BUMPER

  //  eTier.onTrue(new ParallelCommandGroup(
  //    new OuterArmRaiseE(m_OuterArm),
  //    new InnerArmRaiseE(m_InnerArm)));
  //  midTier.onTrue(new ParallelCommandGroup(
  //    new OuterArmRaiseM(m_OuterArm),
  //    new InnerArmRaiseM(m_InnerArm)));
  //  sTeir.onTrue(new ParallelCommandGroup(
  //    new OuterArmRaiseS(m_OuterArm),
  //    new InnerArmRaiseS(m_InnerArm)));
    outerRaise.whileTrue(new OuterArmRaise(m_OuterArm));
    outerLower.whileTrue(new OuterArmLower(m_OuterArm));
    innerRaise.whileTrue(new InnerArmRaise(m_InnerArm));
    innerLower.whileTrue(new InnerArmLower(m_InnerArm));
      

    
    
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
