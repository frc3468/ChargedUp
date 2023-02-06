/*This subsystem takes from ServeModule and treats the drive system as a whole.  Drive based methods
 * exist here
*/
package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.sim.CANcoderSimState;
import com.ctre.phoenix.sensors.WPI_CANCoder;
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
//import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import com.ctre.phoenix.sensors.WPI_CANCoder;

public class SwerveDrive extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;
  private SwerveDriveKinematics swerveDriveKinematics;
  private Field2d field;
  
  public SwerveDrive() {
      
      
     // public static final int canCoderID = 9;
            




    gyro = new Pigeon2(0); //for Pigeon
    //gyro.restoreFactoryDefaults(); //for Pigeon
   
    zeroGyro();

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };
    swerveOdometry = new SwerveDriveOdometry(
    Constants.Swerve.swerveKinematics
    , getYaw()
    , getModulePositions()
    );
    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

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

//   public SwerveModuleState[] getStates() { //TODO this can probably be removed, the getmodulepositions seems to replace it
//     SwerveModuleState[] states = new SwerveModuleState[4];
//     for (SwerveModule mod : mSwerveMods) {
//       states[mod.moduleNumber] = mod.getState();
//     }
//     return states;
//   }

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
//TODO THIS MIGHT BE THE 180 FLIP ERROR WE SEE IN FIELD CENTRIC MODE
    double fieldRelativeXVelocity = translation.getX() * Math.cos(-gyro.getYaw() * (Math.PI/180)) + translation.getY() * Math.sin(-gyro.getYaw() * (Math.PI/180));
    double fieldRelativeYVelocity = -translation.getX() * Math.sin(-gyro.getYaw() * (Math.PI/180)) + translation.getY() * Math.cos(-gyro.getYaw() * (Math.PI/180));

   // double fieldRelativeXVelocity = translation.getX() * Math.cos(-gyro.getYaw() * (Math.PI)) + translation.getY() * Math.sin(-gyro.getYaw() * (Math.PI));
   // double fieldRelativeYVelocity = -translation.getX() * Math.sin(-gyro.getYaw() * (Math.PI)) + translation.getY() * Math.cos(-gyro.getYaw() * (Math.PI));

    double XVelocity = translation.getX();
    double YVelocity = translation.getY(); 


 // Constants.Swerve.swerveKinematics
 // TODO JB this code may not work
    var swerveModuleStates = swerveDriveKinematics.toSwerveModuleStates(
      fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeXVelocity, fieldRelativeYVelocity, rot, Rotation2d.fromDegrees(gyro.getYaw()))
          : new ChassisSpeeds(XVelocity, YVelocity, rot));
  SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates,Swerve.maxSpeed);
      mSwerveMods[0].setDesiredState(swerveModuleStates[0],true);
      mSwerveMods[1].setDesiredState(swerveModuleStates[1],true);
      mSwerveMods[2].setDesiredState(swerveModuleStates[2],true);
      mSwerveMods[3].setDesiredState(swerveModuleStates[3],true);
  
  }
// Set the wheels to an X pattern to plant the robot
   public void setWheelsToX(){
    setModuleStates(new SwerveModuleState[]{
      //front left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)),
      //front right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)),
      //rear left
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)),
      //rear right
      new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0))
    });
  }
    
    
    ;

/*  CONTROLLER OUTPUTS  */
  public void zeroGyro() {
    System.out.println("Y Button Pressed");
    System.out.println(gyro);
    gyro.setYaw(0);
  }

  //declared here for testing, but got a firmware error upon readout
  CANcoder canCode0 = new CANcoder(3);
  CANcoder canCode1 = new CANcoder(6);
  CANcoder canCode2 = new CANcoder(9);
  CANcoder canCode3 = new CANcoder(12);

  public void GetGyroReading() {
    System.out.println("X Button Pressed");
    System.out.println(gyro.getYaw());
  }
    public void GetsetWheelsToX() {
      System.out.println("B Button Pressed");
      setWheelsToX();
  }
     //System.out.println(gyro.getYaw());
    // TODO The system out is producing an error -10022 and -1003
   // System.out.println()
    
    /*     "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
        "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees());
        SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
          */
      //System.out.println(modulenumber + "Cancoder", getCancoder().getDegrees)
    
    /*
    System.out.println("Front Right position");
    System.out.println(canCode1.getPosition());
    System.out.println("Rear Left position");
    System.out.println(canCode2.getPosition());
    System.out.println("Rear Right position");
    System.out.println(canCode3.getPosition());
    */
  
  public void SetFieldDrive() {
    System.out.println("Left bumper Button Pressed, Field Drive Enabled");

  }public void SetRobotCentric() {
    System.out.println("Right bumper Button Pressed, Robot Centric Drive Enabled");
    
  }

  public SwerveModulePosition[] getModulePositions(){ //TODO this is new, might need to double check
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for(SwerveModule mod : mSwerveMods){
        positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
}

  public Rotation2d getYaw() {

    // if (gyro.isMagnetometerCalibrated()) {
    //     // We will only get valid fused headings if the magnetometer is calibrated
    //     return Rotation2d.fromDegrees(gyro.getFusedHeading());
    //     }

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
  }
}