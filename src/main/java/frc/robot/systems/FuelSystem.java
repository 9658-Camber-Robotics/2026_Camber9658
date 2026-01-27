package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import frc.robot.subsystems.*;

public class FuelSystem {

    private final Feeder m_feeder;
    private final Shooter m_shooter;
    private final Intake m_intake;
    private final Hood m_hood;
    private final Floor m_floor;

    public FuelSystem(Feeder feeder, Shooter shooter, Intake intake, Hood hood, Floor floor) {
        m_feeder = feeder;
        m_shooter = shooter;
        m_intake = intake;
        m_hood = hood;
        m_floor = floor;
    }

    public Command emptyFuel() {

        // Intake go down
        // Feeder go in
        /// Empties feeder
        return m_intake.intakeCommand().andThen(parallel(m_floor.outtake(), m_feeder.outtake()));
    }

    public Command intakeFuel() {
        /// Set Intake Pos
        /// Feeder Intake

        return m_intake.intakeCommand().andThen(m_feeder.intake());
    }

    public Command shootFuel() {
        /// Calculates the RPM of the shooter, and Hood angle.
        /// Sets the RPM and hood angle
        /// Fires when shooter is spun up and fuel is present
        /// Starts running Floor constantly
        /// Ends when fuel is not present for ~3 seconds
        /// While shooting, switch between intake pos and agitate pos
        return null;
    }

}
