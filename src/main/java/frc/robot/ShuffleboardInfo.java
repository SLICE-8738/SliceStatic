package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;


public class ShuffleboardInfo {
    
   // ShuffleboardTab tab = Shuffleboard.getTab("Modules");
    
    private final ShuffleboardTab modulesTab;

    public  ShuffleboardInfo(Drivetrain drivebase) {
        
        modulesTab = Shuffleboard.getTab("Module Tab");
        //debugTab = Shuffleboard.getTab("Debug Tab");

            modulesTab.addDouble("Left Front CANCoder Angle", () -> drivebase.getCANCoderAngles()[0]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(0, 1).
            withSize(2, 1);
            //Displays the current CANCoder angle in degrees with no offset of the left back swerve module on Shuffleboard
            modulesTab.addDouble("Left Back CANCoder Angle", () -> drivebase.getCANCoderAngles()[1]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0,"Max", 360)).
            withPosition(0, 2).
            withSize(2, 1);
            //Displays the current CANCoder angle in degrees with no offset of the right front swerve module on Shuffleboard
            modulesTab.addDouble("Right Front CANCoder Angle", () -> drivebase.getCANCoderAngles()[2]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(7, 1).
            withSize(2, 1);
            //Displays the current CANCoder angle in degrees with no offset of the right back swerve module on Shuffleboard
            modulesTab.addDouble("Right Back CANCoder Angle", () -> drivebase.getCANCoderAngles()[3]).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(7, 2).
            withSize(2, 1);

           /* modulesTab.add("Drivetrain Heading", drivetrain.getHeading()).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(0, 0).
            withSize(2, 1);*/
            //Displays the current integrated encoder angle in degrees of the left front swerve module on Shuffleboard
            modulesTab.addDouble("Left Front Integrated Angle", () -> drivebase.getStates()[0].angle.getDegrees()).
            withPosition(2, 0).
            withSize(2, 1);
            //Displays the current integrated encoder angle in degrees of the left back swerve module on Shuffleboard
            modulesTab.addDouble("Left Back Integrated Angle", () -> drivebase.getStates()[1].angle.getDegrees()).
            withPosition(2, 3).
            withSize(2, 1);
            //Displays the current integrated encoder angle in degrees of the right front swerve module on Shuffleboard
            modulesTab.addDouble("Right Front Integrated Angle", () -> drivebase.getStates()[2].angle.getDegrees()).
            withPosition(5, 0).
            withSize(2, 1);
            //Displays the current integrated encoder angle in degrees of the right back swerve module on Shuffleboard
            modulesTab.addDouble("Right Back Integrated Angle", () -> drivebase.getStates()[3].angle.getDegrees()).
            withPosition(5, 3).
            withSize(2, 1);
            modulesTab.addDouble("heading", () -> drivebase.getHeading().getDegrees()).
            withPosition(5, 4).
            withSize(2, 1);
        
           /* //Displays the current heading of the robot in degrees on Shuffleboard
            debugTab.addDouble("Drivetrain Heading", drivetrain.getHeading()).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", 0, "Max", 360)).
            withPosition(0, 0).
            withSize(2, 1);
            //Displays the current roll of the robot in degrees on Shuffleboard
            debugTab.addDouble("Drivetrain Roll", drivetrain::getRoll).
            withWidget(BuiltInWidgets.kDial).
            withProperties(Map.of("Min", -180, "Max", 180)).
            withPosition(7, 0).
            withSize(2, 1);
                
            //Displays the current position of the robot on the field on Shuffleboard
            debugTab.add(drivetrain.m_field2d).
            withPosition(3, 2).
            withSize(3, 2);*/
        
            

        }

    }
 
    