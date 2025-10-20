package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeSystem;

public class AlgaeGet extends Command
{
    public AlgaeSystem algaeSystem;

    public AlgaeGet(AlgaeSystem algaeSystem)
    {
        this.algaeSystem = algaeSystem;
        addRequirements(algaeSystem);
    }

    @Override
    public void execute()
    {
        algaeSystem.algaeMotor.set(Constants.AlgaeState.GET.speed);
    }
    
    @Override
    public boolean isFinished()
    {
        // return algaeSystem.algaeMotor.getOutputCurrent() > 30;
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        algaeSystem.algaeMotor.stopMotor();
    }
        
}
