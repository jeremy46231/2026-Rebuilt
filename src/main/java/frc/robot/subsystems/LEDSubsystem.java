package frc.robot.subsystems;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDSubsystem.LEDState;

public class LEDSubsystem extends SubsystemBase {
    private static final int STRIP_LENGTH = 41;

    private static FireAnimation FLAME = new FireAnimation(8, STRIP_LENGTH)
            .withFrameRate(20)
            .withBrightness(1)
            .withSparking(0.4)
            .withCooling(0.4)
            .withDirection(AnimationDirectionValue.Forward);

    private CANdle candle;

    public LEDSubsystem() {
        candle = new CANdle(39);

        LEDStateGroup.LEDS_OFF.run(candle);
    }

    @Override
    public void periodic() {
        LEDStateGroup.FLAME.run(candle);
    }

    /** Enum for animation info */
    enum LEDState {
        NONE_SLOT_0("LEDs off (slot 0)", new EmptyAnimation(0)),
        NONE_SLOT_1("LEDs off (slot 1)", new EmptyAnimation(1)),
        NONE_SLOT_2("LEDs off (slot 2)", new EmptyAnimation(2)),
        FLAME_LEFT("Flame (left side LEDs)", FLAME.clone().withLEDStartIndex(28).withLEDEndIndex(STRIP_LENGTH).withSlot(0)),
        FLAME_MIDDLE("Flame (middle LEDs)", FLAME.clone().withLEDStartIndex(21).withLEDEndIndex(27).withSlot(1)),
        FLAME_RIGHT("Flame (right side LEDs)", FLAME.clone().withLEDStartIndex(8).withLEDEndIndex(20).withSlot(2));

        String name;
        ControlRequest animation;

        LEDState(String name, ControlRequest animation) {
            this.name = name;
            this.animation = animation;
        }
    }
}

/**
 * Represents a group of LEDStates to run multiple at the same time, since some
 * animations need to run across multiple slots
 */
enum LEDStateGroup {
    LEDS_OFF("LEDs off", LEDState.NONE_SLOT_0, LEDState.NONE_SLOT_1, LEDState.NONE_SLOT_2),
    FLAME("Flame", LEDState.FLAME_LEFT, LEDState.FLAME_MIDDLE, LEDState.FLAME_RIGHT);

    String name;
    LEDState[] animations;

    LEDStateGroup(String name, LEDState... animations) {
        this.name = name;
        this.animations = animations;
    }

    void run(CANdle candle) {
        for (LEDState ledState : animations)
            candle.setControl(ledState.animation);
        DogLog.log("Subsystems/LEDs/runningAnimationGroup", name);
    }
}