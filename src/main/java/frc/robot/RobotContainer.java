package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import java.lang.ModuleLayer.Controller;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
//import frc.robot.subsystems.Hopper;
//import frc.robot.subsystems.Elevator;
//import frc.robot.subsystems.Intake;
import frc.robot.commands.TeleopSwerve;
//import frc.robot.autos.*;
//import frc.robot.commands.*;
//import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.Drivetrain;
//import frc.robot.subsystems.Arm;
//import frc.robot.subsystems.Hopper;
//import frc.robot.subsystems.Elevator;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    //private final Joystick driver = new Joystick(0);
    public Joystick manip = new Joystick(1);
    public CommandPS4Controller driverController = new CommandPS4Controller(0);
    public CommandGenericHID operatorController = new CommandPS4Controller(1);

    /* Subsystems */
    private final Drivetrain s_Swerve = new Drivetrain();
    private final SendableChooser<Command> autoChooser;

    private final Trigger zeroGyro = driverController.L1();
    private final Trigger robotCentric = driverController.R1();

    private final TeleopSwerve swerveDrive = new TeleopSwerve(s_Swerve, driverController);

    // Manip Subsystems
    //private final Elevator elevator = new Elevator();
    //private final Intake intake = new Intake();
    //private final Arm arm = new Arm();
    //private final RunElevator runElevator = new RunElevator(elevator, operatorController);
    //private final Hopper hopper = new Hopper();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //register named commands for pathplanner
        //NamedCommands.registerCommand("Run Hopper", hopper.runHopperAuto());
        //NamedCommands.registerCommand("Stop Hopper", hopper.stopHopperAuto());
        //NamedCommands.registerCommand("Elevator Position 0", elevator.setPosition0Auto());
        //NamedCommands.registerCommand("Elevator Position 1", elevator.setPosition1Auto());
        //NamedCommands.registerCommand("Elevator Position 2", elevator.setPosition2Auto());
        //NamedCommands.registerCommand("Elevator Position 3", elevator.setPosition3Auto());
        //NamedCommands.registerCommand("Arm Position 0", arm.setPosition0Auto());
        //NamedCommands.registerCommand("Arm Position 1", arm.setPosition1Auto());
        //NamedCommands.registerCommand("Arm Position 2", arm.setPosition2Auto());
        //NamedCommands.registerCommand("Arm Position 3", arm.setPosition3Auto());
        //NamedCommands.registerCommand("Run Intake", intake.runIntakeAuto());
        //NamedCommands.registerCommand("Stop Intake", intake.stopIntakeAuto());


        s_Swerve.setDefaultCommand(swerveDrive);
        //elevator.setDefaultCommand(runElevator);

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        
        //intake controls
        /*
        new JoystickButton(driver, 6).onTrue(intake.run(intake.intakeSpeed*-1));
        new JoystickButton(driver, 6).onFalse(intake.rest());
        */


        /* Manip Buttons */ 
        //hopper in
        /*
        new JoystickButton(manip, 5).onTrue(hopper.runLeft(hopper.hopperSpeed));
        new JoystickButton(manip, 5).onTrue(hopper.runRight(hopper.hopperSpeed));
        new JoystickButton(manip, 5).onFalse(hopper.runLeft(0));
        new JoystickButton(manip, 5).onFalse(hopper.runRight(0));
        
        //hopper out
        new JoystickButton(manip, 6).onTrue(hopper.reverseLeft(hopper.reverseHopperSpeed));
        new JoystickButton(manip, 6).onTrue(hopper.reverseRight(hopper.reverseHopperSpeed));    
        new JoystickButton(manip, 6).onFalse(hopper.reverseLeft(0));
        new JoystickButton(manip, 6).onFalse(hopper.reverseRight(0));
        */
        //hopper controls
        // new JoystickButton(manip, 5).onTrue(hopper.in());
        // new JoystickButton(manip, 5).onFalse(hopper.stop());
        // new JoystickButton(manip, 6).onTrue(hopper.out());
        // new JoystickButton(manip, 6).onFalse(hopper.stop());
        
        //elevator + arm + intake
      //  new JoystickButton(manip, 3).onTrue(system.prepToGrab()); //X
      //  new JoystickButton(manip, 1).onTrue(system.grab()); //A
      //  new JoystickButton(manip, 4).onTrue(system.rest()); //Y

        //new and improved (theoretically)
     //   new JoystickButton(manip, 4).onTrue(system.grabFunction);
     //   new JoystickButton(manip, 3).onTrue(system.intakeGo());
      //  new JoystickButton(manip, 3).onFalse(system.intakeStop());

      //  new POVButton(manip, 0).onTrue(system.setPosition0());
     //   new POVButton(manip, 90).onTrue(system.setPosition1());
    //    new POVButton(manip, 180).onTrue(system.setPosition2());
   //     new POVButton(manip, 270).onTrue(system.setPosition3());

    //    SmartDashboard.getNumber("Elevator pos", elevator.currentPosition);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoChooser.getSelected();
    }
}
