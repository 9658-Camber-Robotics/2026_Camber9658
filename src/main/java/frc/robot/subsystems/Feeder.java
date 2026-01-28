package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import java.util.List;

import static edu.wpi.first.units.Units.*;

public class Feeder extends SubsystemBase {

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec).in(RotationsPerSecond)))
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(50))
            .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST);
//            .withMomentOfInertia(YUnits.PoundSquareFeet.of(1));

    private final TalonFX motor;
    private final TalonFXWrapper motorSMC;

    public Feeder() {
        motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);
        motorSMC = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(false));

        //this is so uglyyy
        (((TalonFX) motorSMC.getMotorController()).getConfigurator()).apply(((TalonFXConfiguration) motorSMC.getMotorControllerConfig()).withVoltage(new VoltageConfigs()
                .withPeakReverseVoltage(Volts.of(0))));
    }

    private Command setPercentOutput(double percentOutput) {
        return run(() -> motorSMC.setDutyCycle(percentOutput));
    }

    public Command intake() {
        return setPercentOutput(-0.5);
    }

    public Command outtake() {
        return setPercentOutput(0.5);
    }

    public Command stop() {
        return setPercentOutput(0);
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
