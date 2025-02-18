package frc.robot.utils.emu

/**
 * Enum class representing the buttons on a joystick or game controller.
 *
 * @property buttonNumber The numerical identifier for the button.
 */
enum class Button(
    val buttonNumber: Int,
) {
    /** Button A with identifier 1. */
    A(1),

    /** Button B with identifier 2. */
    B(2),

    /** Button X with identifier 3. */
    X(3),

    /** Button Y with identifier 4. */
    Y(4),

    /** Start button with identifier 8. */
    START(8),

    /** Left bumper button with identifier 5. */
    LEFT_BUMPER(5),

    /** Right bumper button with identifier 6. */
    RIGHT_BUMPER(6),
}
