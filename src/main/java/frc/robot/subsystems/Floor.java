package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.*;

public class Floor extends SubsystemBase {
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig()
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(30))
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE);

    private final TalonFX motor;
    private final TalonFXWrapper motorSMC;
    public Floor() {
        motor = new TalonFX(Ports.kFloor, Ports.kRoboRioCANBus);
        motorSMC = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(true));
    }

    private Command setVelocity(AngularVelocity rpm) {
        return run(()-> motorSMC.setVelocity(rpm));
    }
    private Command setPercentOutput(double percent) {
        return run(()-> motorSMC.setDutyCycle(percent));
    }
    public Command intake() {
        return run(()-> setPercentOutput(0.5));
    }
    public Command outtake() {
        return run(()-> setPercentOutput(-0.5));
    }
    public Command stop() {
        return run(()-> setPercentOutput(0));
    }

    @Override
    public void periodic() {
        motorSMC.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        motorSMC.simIterate();
    }
}
