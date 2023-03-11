// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.CloseClaw;
import frc.robot.commands.OpenClaw;
import frc.robot.commands.arm.TeleopArm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Arms.ArmOverride;
import frc.robot.subsystems.Arms.InnerArm;
import frc.robot.subsystems.Arms.OuterArm;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
  private final Joystick overRideLeft = new Joystick(1);
  private final Joystick overRideRight = new Joystick(2);


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
  private final JoystickButton expandClaw = new JoystickButton(overRideLeft, Constants.Clawconstants.overideClawOpen);
  private final JoystickButton condenseClaw = new JoystickButton(overRideRight, Constants.Clawconstants.overideClawClose);

  // BACK/SELECT - Zero Gyro reading
  private final JoystickButton zeroGyro = new JoystickButton(primaryDriver, XboxController.Button.kBack.value);
  // START - Hard brake/ auto-level for balance board
  private final JoystickButton autoLevel = new JoystickButton(primaryDriver, XboxController.Button.kStart.value);

  // Bumpers and Triggers
  // Left Bumper - auto-load
  private final JoystickButton autoLoad = new JoystickButton(primaryDriver, XboxController.Button.kLeftBumper.value);
  private final Claw m_Claw = new Claw();
  private final JoystickButton openClaw = new JoystickButton(primaryDriver, XboxController.Button.kRightBumper.value);
  private final JoystickButton closeClaw = new JoystickButton(primaryDriver, XboxController.Button.kLeftBumper.value);  // Right bumper - Open/Close the claw
  

  /* Co-Driver Buttons - Dual Joysticks */
  private final int outerArmAxis = Joystick.AxisType.kY.value;
  private final int innerArmAxis = Joystick.AxisType.kY.value;

  /* End of Joystick and Controller assignments */

  /* Subsystems */
  private final SwerveDrive s_Swerve = new SwerveDrive();
  private final ArmOverride s_ArmOverride = new ArmOverride();

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
           () -> zeroGyro.getAsBoolean()));

  m_InnerArm.setDefaultCommand(
    new TeleopArm(m_InnerArm, m_OuterArm, 
    () -> overRideLeft.getRawAxis(outerArmAxis), 
    () -> overRideRight.getRawAxis(innerArmAxis))
  );

    // Configure the button bindings
    configureButtonBindings();
    configureBindings();
  }
  
   /* End Subsystems */

  /* Button Bindings - Actions taken upon button press or hold */
  private void configureButtonBindings() { 
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro())); //Y

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
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Trigger lasersense = new Trigger(m_Claw::getLazerSenser);
    
    lasersense.onFalse(new CloseClaw(m_Claw));

    //normal controller
    openClaw.onTrue(new OpenClaw(m_Claw));
    closeClaw.onTrue(new CloseClaw(m_Claw));
    //override controller
    expandClaw.onTrue(new OpenClaw(m_Claw));
    condenseClaw.onTrue(new CloseClaw(m_Claw));


    
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
