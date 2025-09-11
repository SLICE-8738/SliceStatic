package frc.robot.commands;

import frc.robot.libs.Constants;
import frc.robot.subsystems.Swerve.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;


public class TeleopSwerve extends Command {    
    private Drivetrain s_Swerve;    
    /*
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    */
    private BooleanSupplier robotCentricSup;
    
    private CommandPS4Controller driverController;

    public TeleopSwerve(Drivetrain s_Swerve, CommandPS4Controller driverController, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        this.driverController = driverController;
        addRequirements(s_Swerve);

        /*
        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        */
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(driverController.getLeftX(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(driverController.getLeftY(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(driverController.getRightX(), Constants.stickDeadband);


        /* Drive */
      s_Swerve.drive(
           new Translation2d(translationVal, strafeVal).times(Constants.kDrivetrain.MAX_LINEAR_VELOCITY), 
            rotationVal * Constants.kDrivetrain.MAX_ANGULAR_VELOCITY, 
           true/*!robotCentricSup.getAsBoolean()*/, 
           true
        );
    }
}