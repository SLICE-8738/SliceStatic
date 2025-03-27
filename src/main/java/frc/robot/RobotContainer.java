// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Subsystems.RunElevator;
import frc.robot.commands.Subsystems.RunHopper;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  private final Elevator elevator = new Elevator();
  private final Intake intake = new Intake();
  private final RunElevator system = new RunElevator(elevator, intake);
  private final Hopper hopper = new Hopper();
  private final RunHopper hopperSubsystem = new RunHopper(hopper);

  public CommandXboxController manipXbox = new CommandXboxController(1);
  public XboxController otherManipXbox = new XboxController(2);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer(){
    configureBindings();
  }

  private void configureBindings()
  {
    /**button commands**/
    //hopper in
    // new JoystickButton(otherManipXbox, 5).onTrue(hopper.runLeft(hopper.hopperSpeed));
    // new JoystickButton(otherManipXbox, 5).onTrue(hopper.runRight(hopper.hopperSpeed));
    // new JoystickButton(otherManipXbox, 5).onFalse(hopper.runLeft(0));
    // new JoystickButton(otherManipXbox, 5).onFalse(hopper.runRight(0));
    
    // //hopper out
    // new JoystickButton(otherManipXbox, 6).onTrue(hopper.reverseLeft(hopper.reverseHopperSpeed));
    // new JoystickButton(otherManipXbox, 6).onTrue(hopper.reverseRight(hopper.reverseHopperSpeed));    
    // new JoystickButton(otherManipXbox, 6).onFalse(hopper.reverseLeft(0));
    // new JoystickButton(otherManipXbox, 6).onFalse(hopper.reverseRight(0));

    //hopper controls
    new JoystickButton(otherManipXbox, 5).onTrue(hopperSubsystem.in());
    new JoystickButton(otherManipXbox, 5).onFalse(hopperSubsystem.stop());
    new JoystickButton(otherManipXbox, 6).onTrue(hopperSubsystem.out());
    new JoystickButton(otherManipXbox, 6).onFalse(hopperSubsystem.stop());
    
    //intake controls
    new JoystickButton(otherManipXbox, 2).onTrue(intake.run(intake.intakeSpeed*-1));
    new JoystickButton(otherManipXbox, 2).onFalse(intake.rest());
    
    //elevator + arm + intake
    // new JoystickButton(otherManipXbox, 3).onTrue(system.grab()); //X
    // new JoystickButton(otherManipXbox, 4).onTrue(system.rest()); //Y
    // new POVButton(otherManipXbox, 0).onTrue(system.setPosition(0)); //up
    // new POVButton(otherManipXbox, 90).onTrue(system.setPosition(1)); //right
    // new POVButton(otherManipXbox, 180).onTrue(system.setPosition(2)); //down
    // new POVButton(otherManipXbox, 270).onTrue(system.setPosition(3)); //left

    new POVButton(otherManipXbox, 0).onTrue(system.run());
  }
}
