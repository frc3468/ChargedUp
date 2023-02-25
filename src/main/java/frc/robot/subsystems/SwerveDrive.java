/*This subsystem takes from ServeModule and treats the drive system as a whole.  Drive based methods
 * exist here
*/
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants.Swerve;

public class SwerveDrive extends SubsystemBase {
  private final Pigeon2 gyro;
  private final Field2d mField;

  private boolean turtleToggle = false;
  private double speed = Constants.Swerve.maxSpeed;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private SwerveDriveKinematics swerveDriveKinematics;
  private Field2d field;

  public SwerveDrive() {
    mField = new Field2d();
    SmartDashboard.putData("Field", mField);

    gyro = new Pigeon2(0); // NavX connected over MXP
    // gyro.restoreFactoryDefaults(); //for Pigeon

    zeroGyro();

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(0, Constants.Swerve.FrontLeftSwerveMod.constants),
        new SwerveModule(1, Constants.Swerve.FrontRightSwerveMod.constants),
        new SwerveModule(2, Constants.Swerve.RearLeftSwerveMod.constants),
        new SwerveModule(3, Constants.Swerve.RearRightSwerveMod.constants)
    };
    swerveOdometry = new SwerveDriveOdometry(
        Constants.Swerve.swerveKinematics, getYaw(), getModulePositions());
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                translation.getX(), translation.getY(), rotation, getYaw())
            : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, speed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, speed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(Translation2d translation, double rot, boolean fieldRelative) {
    // TODO THIS MIGHT BE THE 180 FLIP ERROR WE SEE IN FIELD CENTRIC MODE
    double fieldRelativeXVelocity = translation.getX() * Math.cos(-gyro.getYaw() * (Math.PI / 180))
        + translation.getY() * Math.sin(-gyro.getYaw() * (Math.PI / 180));
    double fieldRelativeYVelocity = -translation.getX() * Math.sin(-gyro.getYaw() * (Math.PI / 180))
        + translation.getY() * Math.cos(-gyro.getYaw() * (Math.PI / 180));

    double XVelocity = translation.getX();
    double YVelocity = translation.getY();

    // Constants.Swerve.swerveKinematics
    // TODO JB this code may not work
    var swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeXVelocity, fieldRelativeYVelocity, rot,
            Rotation2d.fromDegrees(gyro.getYaw()))
        : new ChassisSpeeds(XVelocity, YVelocity, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, Swerve.maxSpeed);
    mSwerveMods[0].setDesiredState(swerveModuleStates[0], true);
    mSwerveMods[1].setDesiredState(swerveModuleStates[1], true);
    mSwerveMods[2].setDesiredState(swerveModuleStates[2], true);
    mSwerveMods[3].setDesiredState(swerveModuleStates[3], true);

  }

  // BOTTOM SHELF - 'A' BUTTON
  public void bottomShelf() {
    System.out.println("A Button Pressed");
    System.out.println("Bottom Shelf");
  }

  // MIDDLE SHELF - 'B' BUTTON
  public void middleShelf() {
    System.out.println("B Button Pressed");
    System.out.println("Middle Shelf");
  }

  // TOP SHELF - 'Y' BUTTON
  public void topShelf() {
    System.out.println("Y Button Pressed");
    System.out.println("Top Shelf");
  }

  // CENTRIC TOGGLE - 'X' BUTTON
  public void turtleMode() {
    System.out.println("Y Button Pressed");
    turtleToggle = !turtleToggle;
    if(turtleToggle){
      speed = Constants.Swerve.turtleSpeed;
      System.out.println("Turtle mode enabled, speed is now " + speed + " meters per second." );
    }else{
      speed = Constants.Swerve.maxSpeed;
      System.out.println("Turtle mode disabled, speed is now " + speed + " meters per second." );
    }
  }


  // ZERO GYRO - 'BACK' BUTTON
  public void zeroGyro() {
    System.out.println("Back Button Pressed");
    System.out.println("GYRO BEFORE: " + gyro.getYaw());
    gyro.setYaw(0);System.out.println("GYRO AFTER: " + gyro.getYaw());
  }

  // AUTO LEVEL - 'START' BUTTON
  public void autoLevel(){
    System.out.println("Start Button Pressed");
    System.out.println("Auto Level");
  }

  // AUTO LOAD - LEFT BUMPER
  public void autoLoad(){
    System.out.println("Left Bumper Pressed");
    System.out.println("Auto Load");
  }

  // CLAW MOVEMENT - RIGHT BUMPER
  public void clawMovement(){
    System.out.println("Right Bumper Pressed");
    System.out.println("Claw Movement");
  }


  public SwerveModulePosition[] getModulePositions() { // TODO this is new, might need to double check
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  public Rotation2d getYaw() {
    // return (Constants.Swerve.invertGyro)
    // ? Rotation2d.fromDegrees(360 - gyro.getYaw())
    // : Rotation2d.fromDegrees(gyro.getYaw());
    // Brittany commented this out because it is not called

    // if (gyro.isMagnetometerCalibrated()) {
    // // We will only get valid fused headings if the magnetometer is calibrated
    // return Rotation2d.fromDegrees(gyro.getFusedHeading());
    // }
    //
    // // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(360.0 - gyro.getYaw());
    return Rotation2d.fromDegrees(gyro.getYaw());

  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getModulePositions());
    field.setRobotPose(getPose());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }

    mField.setRobotPose(getPose());
  }
}