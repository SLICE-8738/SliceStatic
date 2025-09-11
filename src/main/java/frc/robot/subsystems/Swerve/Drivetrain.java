package frc.robot.subsystems.Swerve;

import frc.robot.libs.Constants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;

public class Drivetrain extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private final AHRS gyro;
    public Field2d field = new Field2d();
    
    RobotConfig config;{
        try{
            config = RobotConfig.fromGUISettings();
        } catch(Exception e) {
            e.printStackTrace();}
        }

    //private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public Drivetrain() {
        

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.kDrivetrain.Mod0.CONSTANTS),
            new SwerveModule(1, Constants.kDrivetrain.Mod1.CONSTANTS),
            new SwerveModule(2, Constants.kDrivetrain.Mod2.CONSTANTS),
            new SwerveModule(3, Constants.kDrivetrain.Mod3.CONSTANTS)
        };

        gyro = new AHRS(Constants.kDrivetrain.NAVX_PORT, 200);
        Timer.delay(1.0);
        resetModulesToAbsolute();
        zeroGyro();

        SmartDashboard.putData(field);
        swerveOdometry = new SwerveDriveOdometry(Constants.kDrivetrain.kSwerveKinematics, getGyroYaw(), getModulePositions());


        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getRobotRelativeSpeeds,
             (speeds, feedforwards) -> driveRobotRelative(speeds),
              new PPHolonomicDriveController(
                new PIDConstants(3.0, 0.0, 0.0),
                new PIDConstants(3.0, 0.0, 0.0)),
                 config, 
                 () -> {
                    var alliance = DriverStation.getAlliance();
                    if(alliance.isPresent()){
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                 }, 
                 this
                 );
    }


    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        Rotation2d heading;
        heading = getPose().getRotation();
        return heading;
        
       // return Constants.kDrivetrain.INVERT_GYRO ? -gyro.getYaw() + 180 : gyro.getYaw() + 180;
    }

    public  ChassisSpeeds getRobotRelativeSpeeds(){
        return Constants.kDrivetrain.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveRobotRelative(ChassisSpeeds speeds){
        SwerveModuleState[] states = Constants.kDrivetrain.kSwerveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.kDrivetrain.MAX_LINEAR_VELOCITY);
        setModuleStates(states);
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    } 
    public void zeroGyro() {

      gyro.reset();
  
    }
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(getGyroYaw(), getModulePositions());

        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder", mod.getCANcoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
            SmartDashboard.putNumber("Heading", getHeading().getDegrees());    
            
        }
    }
    public double[] getCANCoderAngles() {

      double[] angles = new double[4];
  
      for(SwerveModule mod : mSwerveMods) {
  
        angles[mod.moduleNumber] = mod.getCANcoder().getDegrees();
  
      }
  
      return angles;
  
    }

    public SwerveModuleState[] getStates() {

      SwerveModuleState[] states = new SwerveModuleState[4];
  
      for(SwerveModule mod : mSwerveMods) {
  
        states[mod.moduleNumber] = mod.getState();
  
      }
  
      return states;
  
    }
}