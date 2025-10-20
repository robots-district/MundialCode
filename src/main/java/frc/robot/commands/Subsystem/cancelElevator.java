package frc.robot.commands.Subsystem;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.ElevatorState;
import frc.robot.Constants.ModeElevator;
import frc.robot.subsystems.ElevatorSystem;

public class cancelElevator extends Command
{
    private ElevatorSystem elevatorSubsystem;
    //private ElevatorState elevatorState;

    public cancelElevator(ElevatorSystem elevatorSubsystem)
    {
        this.elevatorSubsystem = elevatorSubsystem;
        addRequirements(elevatorSubsystem);
    }

    @Override
    public void initialize() {
       elevatorSubsystem.stopElevator();
    }
}
