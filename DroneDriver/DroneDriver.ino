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

/**
 * @brief Packet containing flight control information, ability to decode.
 * 
 * Packet storing pitch, roll, throttle, yaw, autoland and a rolling counter
 * sequence. The information values (except autoland) are stored in 5 bits,
 * giving 32 values (intented 16 for both directions). The autoland is a single
 * bit because its a switch indicator.
 * 
 * The sequence is a 3 bit rolling value that has the job to stay consistant.
 * This is used when decoding the bytes from a 5 byte buffer, it makes sure all
 * the bytes have the same sequence - if not, the packet is broken. The packet
 * assumes that information is being sent 1 byte at a time, therefore theres a
 * security consistancy check using the rolling sequence.
 */
struct RemotePacket {
  uint8_t pitch;    // 5b (0-31)
  uint8_t roll;     // 5b (0-31)
  uint8_t throttle; // 5b (0-31)
  uint8_t yaw;      // 5b (0-31)
  uint8_t autoland; // 1b (0-1)
  uint8_t seq;      // 3b (0-9) rolling counter

  /**
   * @brief Encode a 5 byte buffer. Does not modify members of the struct.
   * 
   * Encodes values to the buffer in the exact following order: pitch, roll,
   * throttle, yaw & autoland.
   * 
   * When encoding values (except autoland) it masks the first 5 bits, then
   * shifts them to the left by 3 to make space for the sequence. It then
   * masks the first 3 bits of the sequence, and appends them to the byte.
   * [value5 | seq3]
   * 
   * For the autoland case, it masks the first bit, shifts it by 3 to the left,
   * and uses the above same procedure for the sequence. Resulting in the last 4
   * bits of the full byte being unused. [unused4 | autoland1 | seq3 ]
   * 
   * @param buffer uint8_t*. Byte array with a length of 5 - this will store the
   * encoded data.
   */
  void encode(uint8_t* buffer) const {
    buffer[0] = ((pitch    & 0x1F) << 3) | (seq & 0x07);
    buffer[1] = ((roll     & 0x1F) << 3) | (seq & 0x07);
    buffer[2] = ((throttle & 0x1F) << 3) | (seq & 0x07);
    buffer[3] = ((yaw      & 0x1F) << 3) | (seq & 0x07);
    buffer[4] = ((autoland & 0x01) << 3) | (seq & 0x07);
  }

  /**
   * @brief Decode from a 5 byte buffer. Does not modify the pointed buffers
   * values - writes to the struct members.
   * 
   * Decodes values from the buffer, expects the exact following order: pitch,
   * roll, throttle, yaw & autoland.
   * 
   * When decoding it first grabs all the sequences in each byte. It does this
   * by masking the first 3 bits (where the rolling sequences are located). It
   * then verifies the itegrity of the packet by checking if all the sequences
   * match eachother, if they dont, the function returns false to indicate the
   * packet is broken.
   * 
   * If verification passes, it starts extracting values and writing them to the
   * struct. When extracting a value it first shifts the bits to the right by 3.
   * Then it masks the first 5 bits, except for autoland - here its only the
   * first bit. Finally the function returns true to indicate a successful
   * decode.
   * 
   * @param buffer uint8_t*. Byte array with a length of 5 - this contains the
   * full encoded packet.
   * 
   * @return bool. True if the packet was decoded successfully.
   * @return bool. False otherwise.
   */
  bool decode(const uint8_t* buffer) {
    // Extract sequences
    uint8_t s0 = buffer[0] & 0x07;
    uint8_t s1 = buffer[1] & 0x07;
    uint8_t s2 = buffer[2] & 0x07;
    uint8_t s3 = buffer[3] & 0x07;
    uint8_t s4 = buffer[4] & 0x07;

    // Verify that all seqs match
    if ((s0 != s1) || (s0 != s2) || (s0 != s3) || (s0 != s4))
      return false;

    // Extract values
    seq      = s0;
    pitch    = (buffer[0] >> 3) & 0x1F;
    roll     = (buffer[1] >> 3) & 0x1F;
    throttle = (buffer[2] >> 3) & 0x1F;
    yaw      = (buffer[3] >> 3) & 0x1F;
    autoland = (buffer[4] >> 3) & 0x01;
    
    return true;
  }
}

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
    bool ready() {
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
    void reset() {
      last = micros();
    }

    /**
     * @brief Sets interval for class.
     */
    void setInterval(uint32_t newInterval) {
      interval = newInterval;
    }

  private:
    uint32_t interval;
    uint32_t last;
};

/**
 * @brief Ultrasonic sensor driver, uses non-blocking scheduling.
 * 
 * Below is the calculated call timing for object distances measured:
 * 1. 2cm ~= 118µs
 * 2. 50cm ~= 3ms
 * 3. 400cm ~= 23.5ms
 * 
 * The values are calculated using trigger timing of 10µs and the round-trip
 * distance at the speed of sound (~343m/s).
 * 
 * @param triggerPin uint8_t. The sensor PIN to trigger on.
 * @param echoPin uint8_t. The PIN to read echo from.
 * @param intervalMicros uint32_t. Interval in microseconds between each sensor
 * value check.
 */
class UltrasonicSensor {
  public:
    UltrasonicSensor(uint8_t triggerPin, uint8_t echoPin,
                     uint32_t intervalMicros)
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
    * @brief Call every loop. Uses state machine and non-blocking timer.
    * 
    * The state machine uses 4 states:
    * 
    * IDLE: Checks if the timer is ready, when ready it fires a pulse on the
    * trigger pin and marks the pulse start, then changes the state to TRIGGER.
    * 
    * TRIGGER: Checks if 10µs past since firing the pulse, if so it sets the
    * trigger PIN to low and marks the pulse start, then changes the state to
    * WAIT_ECHO.
    * 
    * WAIT_ECHO: Checks if echo is high, if so, mark the pulse start and change
    * the state to MEASURE.
    * 
    * MEASURE: Checks if the echo is low, if so, grab the pulse duration, then
    * calculate the distance in centimeters using the speed of sounds
    * return-trip. Finally change the state backto IDLE.
    */
    void update() {
      switch (state) {
        case State::IDLE:
          if (timer.ready()) {
            digitalWrite(trig, HIGH);
            pulseStart = micros();
            state = State::TRIGGER;
          }
          break;

        case State::TRIGGER:
          if ((uint32_t)(micros() - pulseStart) >= 10) {
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
            distanceCm = duration * 0.034 / 2;
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
 * All gain values are tuned for delta-time in seconds.
 * 
 * @param kp float. Proportional gain, controls response strength,
 * higher = faster
 * reaction, too high = oscillation.
 * @param ki float. Integral gain, corrects long-term drifting,
 * too high = overshoot correction (slowly start to wobble).
 * @param kd float. Derivative gain, dampens rapid changes in error,
 * too high = noise amplification.
 * @param outputLimit float. Maximum return value of the PID, prevent violent
 * corrections and keep the controller stable.
 * @param integralLimit float. Clamping of the integral, prevent integral windup
 * that can eventually flip the drone.
 * @param intervalMicros uint32_t. Period to run the PID controller at.
 */
class PID {
  public:
    PID(float kp, float ki, float kd, float outputLimit,
        float integralLimit, uint32_t intervalMicros)
      : kp(kp), ki(ki), kd(kd), outputLimit(outputLimit),
        integralLimit(integralLimit), dt(intervalMicros * 1e-6f),
        invDt(1.0f / dt), timer(intervalMicros), integral(0), lastError(0)
    {}

    /**
     * @brief Uses the IntervalMicros non-blocking timer.
     *
     * @return bool. True if update should run this loop iteration.
     * @return bool. False otherwise.
     */
    bool ready() { return timer.ready(); }

    /**
     * @brief Calculate the error from parameters and apply integral, derivative
     * and compute the correction from that.
     *
     * @param target float. Target setpoint, desired value.
     * @param measurement float. Feedback measurement from sensors.
     * 
     * @return float. Correction value.
    */
    float update(float target, float measurement) {
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
    void reset() {
        integral = 0;
        lastError = 0;
    }

    // Gains can be changed live
    float kp, ki, kd;

  private:
    float integral;
    float lastError;
    float dt, invDt;

    float outputLimit, integralLimit;

    IntervalMicros timer;
};

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

// Need 3, roll, pitch, yaw (RC values go before PID update)
PID rateRollPID(0.12f, 0.02f, 0.001f, // Values are fucking random, not tuned
                    400.0f,     // output limit
                    100.0f,     // integral limit
                    2000);      // 2000 µs = 500 Hz

void setup() {
  Serial.begin(9600);

  // Test shit before i have props and BLDCs
  // Note dont use the Arduino with the plastic shield, its retarded and broken.
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

