package frc.robot.commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ElevatorPositionSelector;
import frc.robot.subsystems.Elevator;

public class MoveToLevel extends Command {
    private final Elevator m_elevator;
    private final double m_level;
    private final double m_threshold;

    /// true is upwards movement. false is downwards movement.
    private boolean movementDirection = false;

    public MoveToLevel(Elevator elevator, double threshold) {
        addRequirements(elevator);

        m_elevator = elevator;
        m_threshold = threshold;
        switch (ElevatorPositionSelector.getSelectedPosition()) {
            case 1:
                m_level = 0;
                break;
            case 2:
                m_level = 1;
                break;
            case 3:
                m_level = 3;
                break;
            case 4:
                m_level = 4;
                break;
            default:
                m_level = 0;
                break;
        }
    }

    public void initialize() {
        if (m_level > m_elevator.getPosition()[0]) {
            movementDirection = true;
        }
    }

    public void execute() {
        m_elevator.moveTo(m_level);
    }

    public void end() {

    }

    public boolean isFinished() {
        boolean finished = false;
        if (m_elevator.atTarget(m_threshold)) {
            finished = true;
        }
        // if (m_elevator.isAtTop() && movementDirection) {
        //     finished = true;
        // }
        // if (m_elevator.isAtBottom() && !movementDirection) {
        //     finished = true;
        // }

        return finished;
    }
}
