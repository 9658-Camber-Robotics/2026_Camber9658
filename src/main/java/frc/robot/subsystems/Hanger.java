package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.remote.TalonFXWrapper;

import static edu.wpi.first.units.Units.*;

public class Hanger extends SubsystemBase {
    public enum Position {
        HOMED(0),
        HANGING(6),
        HUNG(0.2);

        private final double inches;

        private Position(double inches) {
            this.inches = inches;
        }

        public Angle motorAngle() {
            final Measure<AngleUnit> angleMeasure = Inches.of(inches).divideRatio(kHangerExtensionPerMotorAngle);
            return Rotations.of(angleMeasure.in(Rotations)); // Promote from Measure<AngleUnit> to Angle
        }
    }

    private static final Distance kExtensionTolerance = Inches.of(1);
    private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig()
            .withClosedLoopController(10, 0, 0, RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec), RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec).per(Second))
            .withFeedforward(new SimpleMotorFeedforward(0, 12 / RadiansPerSecond.of(DCMotor.getKrakenX60(1).freeSpeedRadPerSec).in(RotationsPerSecond)))
            .withStatorCurrentLimit(Amps.of(20))
            .withSupplyCurrentLimit(Amps.of(70))
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE);
    private Distance circumference = Inches.of(1);

    private static final Per<DistanceUnit, AngleUnit> kHangerExtensionPerMotorAngle = Inches.of(6).div(Rotations.of(142));

    private final TalonFX motor;
    private final TalonFXWrapper motorSMC;
    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private boolean isHomed = false;

    public Hanger() {
        motor = new TalonFX(Ports.kHanger, Ports.kRoboRioCANBus);
        motorSMC = new TalonFXWrapper(motor, DCMotor.getKrakenX60(1), smcConfig.clone().withMotorInverted(false));

        //again, this is so ugly
        (((TalonFX) motorSMC.getMotorController()).getConfigurator()).apply(((TalonFXConfiguration) motorSMC.getMotorControllerConfig()).withVoltage(new VoltageConfigs()
                .withPeakReverseVoltage(Volts.of(0))));
    }

    private Command setPercentOutput(double percentOutput) {
        return run(() -> motorSMC.setDutyCycle(percentOutput));
    }

    public Command setPosition(Angle position) {
        return run(() -> motorSMC.setPosition(position));
    }

    //havent made yet, this is stock
    public Command homingCommand() {
        return Commands.sequence(
                        runOnce(() -> setPercentOutput(-0.05)),
                        Commands.waitUntil(() -> motor.getSupplyCurrent().getValue().in(Amps) > 0.4),
                        runOnce(() -> {
                            motor.setPosition(Position.HOMED.motorAngle());
                            isHomed = true;
                        })
                )
                .unless(() -> isHomed)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public boolean isHomed() {
        return isHomed;
    }

    private boolean isExtensionWithinTolerance() {
        if (motorSMC.getMechanismPositionSetpoint().isEmpty())
            return false;
        return motorSMC.getMechanismPosition().isNear(motorSMC.getMechanismPositionSetpoint().orElseThrow(), Rotations.of(kExtensionTolerance.in(Meter) / circumference.in(Meter)));
    }

    private Distance motorAngleToExtension(Angle motorAngle) {
        final Measure<DistanceUnit> extensionMeasure = motorAngle.timesRatio(kHangerExtensionPerMotorAngle);
        return Inches.of(extensionMeasure.in(Inches)); // Promote from Measure<DistanceUnit> to Distance
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
