/**
 * @file    DroneDriver.ino
 * @author  Luka Jacobsen
 * @brief   Quadcopter drone control system.
 * @date    2026-02-26
 * * @details This file handles the controlling of a drone using PID. It has
 * the ability to float (autostabilize) midair, aswell as being controlable
 * from remote.
 * * HARDWARECONNECTIONS:
 */

// Include libraries
#include <RH_ASK.h>
#include <SPI.h>

// Create radio driver
RH_ASK rf_driver;

// ========================================================================== //
// CONFIGURATION                                                              //
// ========================================================================== //



// ========================================================================== //
// PIN DEFINITIONS                                                            //
// ========================================================================== //



// ========================================================================== //
// ENUMS / STRUCTS                                                            //
// ========================================================================== //



// ========================================================================== //
// RUNTIME GLOBAL VARIABLES                                                   //
// ========================================================================== //

// ========================================================================== //
// CLASSES                                                                    //
// ========================================================================== //

/**
 * @brief Non-blocking time scheduler.
 * 
 * Alternative design for a non-blocking version of the delay() functions.
 * 
 * @param intervalMicros uint32_t. Interval between each call.
 */
class IntervalMicros {
  public:
    IntervalMicros(uint32_t intervalMicros)
      : interval(intervalMicros), last(0)
    {}

    /**
     * @brief Non-blocking scheduler. Checks if enough time has elapsed to
     * pass the interval.
     * 
     * @return bool. True if enough time elapsed.
     * @return bool. False otherwise.
     */
    inline bool ready() {
      // Return true if interval has elapsed
      uint32_t now = micros();
      if ((uint32_t)(now - last) >= interval) {
        last = now;
        return true;
      }

      // Interval has not elapsed
      return false;
    }

    /**
     * @brief Reset last timestamp to current time.
     */
    inline void reset() {
      last = micros();
    }

    /**
     * @brief Sets interval for class.
     */
    inline void setInterval(uint32_t newInterval) {
      interval = newInterval;
    }

  private:
    uint32_t interval;
    uint32_t last;
};

/**
 * @brief .
 */
class UltrasonicSensor {
  public:
    UltrasonicSensor(uint8_t triggerPin, uint8_t echoPin, uint32_t intervalMicros)
        : trig(triggerPin), echo(echoPin), timer(intervalMicros),
          state(State::IDLE), pulseStart(0), distanceCm(0)
    {
      // Set modes
      pinMode(trig, OUTPUT);
      pinMode(echo, INPUT);
      
      // Make sure the trigger is low
      digitalWrite(trig, LOW);
    }

    /**
    * @brief Call every loop
    */
    void update() {
        switch (state) {
            case State::IDLE:
                if (timer.ready()) {
                    // Start trigger pulse
                    digitalWrite(trig, HIGH);
                    pulseStart = micros();
                    state = State::TRIGGER;
                }
                break;

            case State::TRIGGER:
                if ((uint32_t)(micros() - pulseStart) >= 10) { // 10 µs pulse
                    digitalWrite(trig, LOW);
                    pulseStart = micros();
                    state = State::WAIT_ECHO;
                }
                break;

            case State::WAIT_ECHO:
                if (digitalRead(echo) == HIGH) {
                    pulseStart = micros();
                    state = State::MEASURE;
                }
                break;

            case State::MEASURE:
                if (digitalRead(echo) == LOW) {
                    unsigned long duration = micros() - pulseStart;
                    distanceCm = duration * 0.034 / 2;  // cm
                    state = State::IDLE;
                }
                break;
        }
    }

    /**
     * @brief Retrieve the distance.
     *
     * @return float. The distance in centimeters.
     */
    float getDistance() const { return distanceCm; }

  private:
    enum class State { IDLE, TRIGGER, WAIT_ECHO, MEASURE };
    State state;

    uint8_t trig, echo;
    IntervalMicros timer;

    uint32_t pulseStart;
    float distanceCm;
};

/**
 * @brief PID controller, optimized for speed. Runs at a fixed timestep.
 * 
 * Designed for real-time control system of a drone stabilization. The
 * controller runs at a constant update interval defined in microseconds to
 * avoid time costly operations. Optimized to run on a low MHz clock-speed.
 * 
 * @param kp float. Proportional gain, controls response strength, higher = faster
 * reaction, too high = oscillation.
 * @param ki float. Integral gain, corrects long-term drifting, too high = overshoot
 * correction (slowly start to wobble).
 * @param kd float. Derivative gain, dampens rapid changes in error, too high = noise
 * amplification.
 * @param outputLimit float. Maximum return value of the PID, prevent violent
 * corrections and keep the controller stable.
 * @param integralLimit float. Clamping of the integral, prevent integral windup that
 * can eventually flip the drone.
 * @param intervalMicros uint32_t. Period to run the PID controller at.
 */
class PID {
  public:
    PID(float kp, float ki, float kd, float outputLimit,
        float integralLimit, uint32_t intervalMicros)
      : kp(kp), ki(ki), kd(kd), outputLimit(outputLimit),
        integralLimit(integralLimit), interval(intervalMicros), lastMicros(0),
        integral(0), lastError(0)
    {
      // Precomute delta-time
      dt = interval * 1e-6f;
      invDt = 1.0f / dt;
    }

    /**
     * @brief Non-blocking scheduler. Checks weather the PID update interval has
     * elapsed.
     *
     * @return bool. True if update should run this loop iteration.
     * @return bool. False otherwise.
     */
    inline bool ready() {
      // Return true if interval has elapsed
      uint32_t now = micros();
      if ((uint32_t)(now - lastMicros) >= interval) {
        lastMicros = now;
        return true;
      }
      
      // Interval have not elapsed
      return false;
    }

    /**
     * @brief Calculate the error from parameters and apply integral, derivative
     * and compute the correction from that.
     *
     * @param target float. Target setpoint, desired value.
     * @param measurement float. Feedback measurement from sensors.
     * 
     * @return float. Correction value.
    */
    inline float update(float target, float measurement) {
        float error = target - measurement;

        // Integral with clamp
        integral += error * dt;
        if (integral > integralLimit) integral = integralLimit;
        else if (integral < -integralLimit) integral = -integralLimit;

        // Derivative (cheap)
        float derivative = (error - lastError) * invDt;
        lastError = error;

        // Compute output
        float output = kp * error + ki * integral + kd * derivative;

        // Clamp output
        if (output > outputLimit) output = outputLimit;
        else if (output < -outputLimit) output = -outputLimit;

        return output;
    }

    /**
     * @brief Resets integral and last recorded error.
     */
    inline void reset() {
        integral = 0;
        lastError = 0;
    }

    // Gains can be changed live
    float kp, ki, kd;

  private:
    float integral;
    float lastError;
    float dt, invDt;

    float outputLimit;
    float integralLimit;

    uint32_t interval;
    uint32_t lastMicros;
};

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

// Need 3, roll, pitch, yaw (RC values go before PID update)
PID rateRollPID(0.12f, 0.02f, 0.001f,
                    400.0f,     // output limit
                    100.0f,     // integral limit
                    2000);      // 2000 µs = 500 Hz

void setup() {
  Serial.begin(9600);

  pinMode(5, OUTPUT); // Forward
  pinMode(9, OUTPUT); // Right
  pinMode(3, OUTPUT); // Left
  pinMode(6, OUTPUT); // Backward

  digitalWrite(5, HIGH);
  digitalWrite(9, HIGH);
  digitalWrite(3, HIGH);
  digitalWrite(6, HIGH);
}

void loop() {
  /*
  usensor.update();

  static IntervalMicros printTimer(100000); // 500ms
  if (printTimer.ready()) {
      Serial.println(usensor.getDistance());
  }
  */
}

// ========================================================================== //
// MAIN FUNCTIONS                                                             //
// ========================================================================== //



// ========================================================================== //
// HELPER FUNCTIONS                                                           //
// ========================================================================== //

