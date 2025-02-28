package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElevatorPositionSelector;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants.kElevator;

public class MoveElevatorToLevel extends Command {
    private final Elevator m_elevator;
    private final double m_elevatorThreshold;
    private double m_level;

    /// true is upwards movement. false is downwards movement.
    private boolean movementDirection = false;

    public MoveElevatorToLevel(Elevator elevator) {
        addRequirements(elevator);

        m_elevator = elevator;
        m_elevatorThreshold = kElevator.THRESHOLD;
    }

    public void initialize() {
        m_level = ElevatorPositionSelector.getSelectedPosition().height;
        if (m_level > m_elevator.getPosition()[0]) {
            movementDirection = true;
        }
    }

    public void execute() {
        m_elevator.moveTo(m_level);
    }

    public void end() {
        m_elevator.set(0);
    }

    public boolean isFinished() {
        boolean finished = false;
        if (m_elevator.atTarget(m_elevatorThreshold)) {
            finished = true;
        }
        // if (m_elevator.isAtTop() && movementDirection) {
        // finished = true;
        // }
        // if (m_elevator.isAtBottom() && !movementDirection) {
        // finished = true;
        // }

        return finished;
    }
}
