package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSystem;
public class AlgaeShoot extends Command
{
    public AlgaeSystem algaeSystem;

    public AlgaeShoot(AlgaeSystem algaeSystem)
    {
        this.algaeSystem = algaeSystem;
        addRequirements(algaeSystem);
    }

    @Override
    public void execute()
    {
        algaeSystem.algaeMotor.set(Constants.AlgaeState.SHOOT.speed);
    }
    
    @Override
    public boolean isFinished()
    {
        // return algaeSystem.algaeMotor.getOutputCurrent() < 30;
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        // Timer.delay(1);
        algaeSystem.algaeMotor.stopMotor();
    }
        
}
