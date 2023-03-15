// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ForeArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.PneumaticsConstants;
import frc.robot.Constants.RearArmConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.ForeArm;
import frc.robot.subsystems.RearArm;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Compressor m_compressor = 
      new Compressor(PneumaticsConstants.kModuleID,PneumaticsConstants.kModuleType);
  
  private final RearArm m_rearArm = new RearArm();
  private final ForeArm m_foreArm = new ForeArm();
  private final Claw m_claw = new Claw();

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final CommandJoystick m_leftOperatorJoystick = 
      new CommandJoystick(OperatorConstants.kOperatorLeftJoystickPort);
  
  private final CommandJoystick m_rightOperatorJoystick = 
      new CommandJoystick(OperatorConstants.kOperatorRightJoystickPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_compressor.enableDigital();

    // Configure the trigger bindings
    configureBindings();
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
    Trigger m_armsStowed = new Trigger(() -> 
      m_rearArm.isAtPosition(RearArmConstants.kStowSetpoint) && m_foreArm.isAtPosition(ForeArmConstants.kStowSetpoint)
    );
    Trigger m_clawClosed = new Trigger(() -> m_claw.isClosed());

    m_driverController.leftBumper().onTrue(m_claw.openCommand());

    // Close Claw Stowed, goto "Move" Position
    m_driverController.rightBumper().and(m_armsStowed).onTrue(new SequentialCommandGroup(
      m_claw.closeCommand(),
      m_rearArm.setPositionCommand(() -> RearArmConstants.kMoveSetpoint),
      m_foreArm.setPositionCommand(() -> ForeArmConstants.kMoveSetpoint)
    ));

    // Close Claw Not-Stowed
    m_driverController.rightBumper().and(m_armsStowed.negate()).onTrue(
      m_claw.closeCommand()
    );

    // Stow
    m_driverController.x().and(m_clawClosed.negate()).onTrue(new SequentialCommandGroup(
      m_rearArm.setPositionCommand(() -> RearArmConstants.kStowSetpoint),
      m_foreArm.setPositionCommand(() -> ForeArmConstants.kStowSetpoint)
    ));

    // Stow, Claw Closed, goto "Move" Position
    m_driverController.x().and(m_clawClosed).onTrue(new SequentialCommandGroup(
      m_rearArm.setPositionCommand(() -> RearArmConstants.kMoveSetpoint),
      m_foreArm.setPositionCommand(() -> ForeArmConstants.kMoveSetpoint)
    ));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
