package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DeployerIntakeSystem;
import swervelib.SwerveDriveTest;

public class Intake extends Command
{
    public DeployerIntakeSystem deployerIntakeSystem;

    public Intake(DeployerIntakeSystem deployerIntakeSystem)
    {
        this.deployerIntakeSystem = deployerIntakeSystem;
        addRequirements(deployerIntakeSystem);
    }

    @Override
    public void initialize() {
        deployerIntakeSystem.deployerLeftMotor.set(Constants.DeployerIntakeConstants.intakeSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
        deployerIntakeSystem.deployerLeftMotor.set(Constants.DeployerState.RESHOOTING.speed);
        //deployerIntakeSystem.deployerLeftMotor.set(Constants.DeployerState.RESHOOTING.speed);
        // deployerIntakeSystem.deployerLeftMotor.stopMotor();
    }
    
        @Override
        public boolean isFinished()
        {
            // return !deployerIntakeSystem.deployerSensor2.get();
    
            // return !deployerIntakeSystem.deployerSensor2.get()
            // public static double intakeSpeed = 0.8
    
            return !deployerIntakeSystem.deployerSensor2.get() && deployerIntakeSystem.deployerSensor1.get();
            // return !deployerIntakeSystem.deployerSensor2.get();
        }
        
}
