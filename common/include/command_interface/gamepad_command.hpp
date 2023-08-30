#ifndef GAMEPAD_COMMAND_HPP_
#define GAMEPAD_COMMAND_HPP_

#include "utilities/utilities.hpp"
#include "cpp_types.hpp"
#include "header/lcm_type/gamepad_lcmt.hpp"

/**
 * @brief The state of the gamepad
 * 
 */
struct GamepadCommand {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
     * @brief Construct a gamepad and set to zero.
     * 
     */
    GamepadCommand() {
        zero();
    }

    bool leftBumper, rightBumper, leftTriggerButton, rightTriggerButton, back, start, a, b, x, y, leftStickButton, rightStickButton, logitechButton;

    Vec2< float > leftStickAnalog, rightStickAnalog;
    float         leftTriggerAnalog, rightTriggerAnalog;

    /**
     * @brief Set all values to zero
     * 
     */
    void zero() {
        leftBumper         = false;
        rightBumper        = false;
        leftTriggerButton  = false;
        rightTriggerButton = false;
        back               = false;
        start              = false;
        a                  = false;
        b                  = false;
        x                  = false;
        y                  = false;
        leftStickButton    = false;
        rightStickButton   = false;

        leftTriggerAnalog  = 0;
        rightTriggerAnalog = 0;
        leftStickAnalog    = Vec2< float >::Zero();
        rightStickAnalog   = Vec2< float >::Zero();
    }

    /**
     * @brief Set the values from an LCM message
     * @param lcmt : LCM message
     */
    void set( const gamepad_lcmt* lcmt ) {
        leftBumper         = lcmt->leftBumper;
        rightBumper        = lcmt->rightBumper;
        leftTriggerButton  = lcmt->leftTriggerButton;
        rightTriggerButton = lcmt->rightTriggerButton;
        back               = lcmt->back;
        start              = lcmt->start;
        a                  = lcmt->a;
        x                  = lcmt->x;
        b                  = lcmt->b;
        y                  = lcmt->y;
        leftStickButton    = lcmt->leftStickButton;
        rightStickButton   = lcmt->rightStickButton;
        leftTriggerAnalog  = lcmt->leftTriggerAnalog;
        rightTriggerAnalog = lcmt->rightTriggerAnalog;
        for ( int i = 0; i < 2; i++ ) {
            leftStickAnalog[ i ]  = lcmt->leftStickAnalog[ i ];
            rightStickAnalog[ i ] = lcmt->rightStickAnalog[ i ];
        }
    }

    /**
     * @brief Copy the values into an LCM message
     * @param lcmt : LCM message
     */
    void get( gamepad_lcmt* lcmt ) {
        lcmt->leftBumper         = leftBumper;
        lcmt->rightBumper        = rightBumper;
        lcmt->leftTriggerButton  = leftTriggerButton;
        lcmt->rightTriggerButton = rightTriggerButton;
        lcmt->back               = back;
        lcmt->start              = start;
        lcmt->a                  = a;
        lcmt->x                  = x;
        lcmt->b                  = b;
        lcmt->y                  = y;
        lcmt->leftStickButton    = leftStickButton;
        lcmt->rightStickButton   = rightStickButton;
        lcmt->leftTriggerAnalog  = leftTriggerAnalog;
        lcmt->rightTriggerAnalog = rightTriggerAnalog;
        for ( int i = 0; i < 2; i++ ) {
            lcmt->leftStickAnalog[ i ]  = leftStickAnalog[ i ];
            lcmt->rightStickAnalog[ i ] = rightStickAnalog[ i ];
        }
    }

    /**
     * @brief The Logitech F310's seem to do a bad job of returning to zero exactly, so a
     * deadband around zero is useful when integrating joystick commands
     * @param f : The deadband
     */
    void GamepadApplyDeadband( float f ) {
        EigenApplyDeadband( leftStickAnalog, f );
        EigenApplyDeadband( rightStickAnalog, f );
        leftTriggerAnalog  = ApplyDeadband( leftTriggerAnalog, f );
        rightTriggerAnalog = ApplyDeadband( rightTriggerAnalog, f );
    }

    /**
     * @brief Represent as human-readable string.
     * @return string representing state
     */
    std::string ToString() {
        std::string result = "Result:\nleftBumper: " + BoolToString( leftBumper ) + "\n" + "rightBumper: " + BoolToString( rightBumper ) + "\n"
                             + "leftTriggerButton: " + BoolToString( leftTriggerButton ) + "\n" + "rightTriggerButton: " + BoolToString( rightTriggerButton ) + "\n" + "back: " + BoolToString( back )
                             + "\n" + "start: " + BoolToString( start ) + "\n" + "a: " + BoolToString( a ) + "\n" + "b: " + BoolToString( b ) + "\n" + "x: " + BoolToString( x ) + "\n"
                             + "y: " + BoolToString( y ) + "\n" + "leftStickButton: " + BoolToString( leftStickButton ) + "\n" + "rightStickButton: " + BoolToString( rightStickButton ) + "\n"
                             + "leftTriggerAnalog: " + std::to_string( leftTriggerAnalog ) + "\n" + "rightTriggerAnalog: " + std::to_string( rightTriggerAnalog ) + "\n"
                             + "leftStickAnalog: " + EigenToString( leftStickAnalog ) + "\n" + "rightStickAnalog: " + EigenToString( rightStickAnalog ) + "\n";
        return result;
    }
};

#endif  // GAMEPAD_COMMAND_HPP_
