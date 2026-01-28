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
    public enum Speed {
        FEED(5000);

        private final double rpm;

        private Speed(double rpm) {
            this.rpm = rpm;
        }

        public AngularVelocity angularVelocity() {
            return RPM.of(rpm);
        }
    }

    private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec).in(RotationsPerSecond)))
            .withStatorCurrentLimit(Amps.of(120))
            .withSupplyCurrentLimit(Amps.of(50))
            .withIdleMode(SmartMotorControllerConfig.MotorMode.COAST);
//            .withMomentOfInertia(YUnits.PoundSquareFeet.of(1));

    private final TalonFX motor;
    private final TalonFXWrapper motorSMC;
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Feeder() {
        motor = new TalonFX(Ports.kFeeder, Ports.kRoboRioCANBus);
        motorSMC = new TalonFXWrapper(motor,DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(false));

        var cfg = (TalonFXConfiguration)motorSMC.getMotorControllerConfig();
        (((TalonFX)motorSMC.getMotorController()).getConfigurator()).apply(cfg.withVoltage(new VoltageConfigs()
                .withPeakReverseVoltage(Volts.of(0))));
    }

    private Command setPercentOutput(double percentOutput) {
        return run(()->motorSMC.setDutyCycle(percentOutput));
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
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("RPM", () -> motor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Stator Current", () -> motor.getStatorCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Supply Current", () -> motor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
