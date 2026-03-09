/** LAST-452-T648
 * @file    DroneDriver.ino
 * @author  Luka Jacobsen
 * @brief   Quadcopter drone control system.
 * @date    2026-02-26
 * * @details This file handles the controlling of a drone using PID. It has
 * the ability to float (autostabilize) midair, aswell as being controlable
 * from remote - it also has the ability to autoland when low enough to the
 * ground, aswell as autolanding it's able to sense when it's flipped to
 * dangerous angles, which from here it will try to perform an emergency
 * landing and reduce motor speed as to reduce possible damage from crash.
 * * HARDWARECONNECTIONS:
 */

// ========================================================================== //
// INCLUDE LIBRARIES                                                          //
// ========================================================================== //

// Needed for basic core operations
#include <Arduino.h>

// RadioHead Amplitude Shifting Keying, 433MHz frequenzy
#include <RH_ASK.h>

// Serial Peripheral Interface library
#include <SPI.h>

// Inter-Intergrated Circuit library (two-wire serial protocol)
#include <Wire.h>

// ========================================================================== //
// CONFIGURATION                                                              //
// ========================================================================== //

// Verbose messages, used when debugging. Ensure evaulation at compile-time.
constexpr bool VERBOSE = true;

// Pich PID tuning values
constexpr float pitchP = 3.0f;
constexpr float pitchI = 0.5f;
constexpr float pitchD = 0.8f;

// Roll PID tuning values
constexpr float rollP = 3.0f;
constexpr float rollI = 0.5f;
constexpr float rollD = 0.8f;

// Yaw PID tuning values
constexpr float yawP = 1.2f;
constexpr float yawI = 0.1f;
constexpr float yawD = 0.0f;

// ========================================================================== //
// PIN DEFINITIONS                                                            //
// ========================================================================== //



// ========================================================================== //
// ENUMS / STRUCTS                                                            //
// ========================================================================== //

/**
 * @brief Packet containing flight control information, ability to encode and
 * decode values.
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
  uint8_t pitch;    // 5bit (0-31)
  uint8_t roll;     // 5bit (0-31)
  uint8_t throttle; // 5bit (0-31)
  uint8_t yaw;      // 5bit (0-31)
  uint8_t autoland; // 1bit (0-1)
  uint8_t seq;      // 3bit (0-9) rolling counter

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
};

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
     * 
     * @param newInterval uint32_t. The new interval to run at.
     */
    void setInterval(uint32_t newInterval) {
      interval = newInterval;
    }

    /**
     * @brief Get the currently used interval.
     */
    uint32_t getInterval() const { return interval; }

  private:
    uint32_t interval;
    uint32_t last;
};

/**
 * @brief Drives an ESC at 50Hz with a pulse between 1000-2000µs. Uses Timer1.
 * 
 * Pins must be one of the below listed, these belong to Timer 4 & 5:
 * - OC4A = 6
 * - OC4B = 7
 * - OC5A = 46
 * - OC5B = 45
 * 
 * Bitshifting logic was found in the ATmega2560 datasheet.
 * 
 * @param pin uint8_t. Which pin to drive the motor from. Not all pins work.
 */
class Motor {
  public:
    Motor(uint8_t pin)
      : pin(pin), pulseMicros(1000), targetPulse(1000), armed(false)
    {}

    /**
     * @brief Sets pin modes and configures Timer to output PWM at 50Hz.
     * 
     * When configuring the timer, it clears the registers responsible for:
     * A. TCCR1A = COM & PWM mode bits.
     * B. TCCR1B = WGM & Prescaler.
     * 
     * It then recognizes which pin is being used and sets the pin to
     * non-inverting PWM. After that, it defines how the timer counts and
     * generates PWM, which is on fast mode.
     * 
     * Next, it sets the prescaler, which is resposible for dividing the main
     * clock (16MHz on an Arduino Uno) to slow down the timer. After this it
     * sets the ICR1 (Input Capture Register) to 40k, this is the top value.
     * 
     * The PWM frequency is calculated: F_CPU / (Prescaler * 40k) ~= 50Hz. With
     * filled values: 16MHz / (8 * 40,000) ~= 50Hz. This is the standard ESC
     * update rate needed to drive the motor.
     */
    void begin() {
      pinMode(pin, OUTPUT);
      digitalWrite(pin, LOW);

      // Clear interrupts.
      cli();

      // Initialize if not
      if (setupTimerForPin(pin) == false) {
        sei();
        return;
      }

      // Set initial pulse and interrupts.
      updateTimerOutput();
      sei();
    }
    
    /**
     * @brief Arm the motor, starting at 1000µs for at least 3 seconds.
     * 
     * This is needed to run the motor.
     * 
     * @param armTimeMs uint32_t. Duration to arm the motor, defaults to 3000ms.
     */
    void arm(uint32_t armTimeMs = 3000) {
      pulseMicros = 1000;
      targetPulse = 1000;
      armed = true;
      armEndMillis = millis() + armTimeMs;
      updateTimerOutput(); // Immediately update output compares
    }
    
    /**
     * @brief Set the desired pulse width.
     * 
     * @param us uint16_t. Desired motor speed, usually between 1000-2000µs.
     */
    void setPulseWidth(uint16_t us) {
      targetPulse = constrain(us, 1000, 2000);
    }

    /**
     * @brief Call every update, only runs if motor is armed.
     * 
     * Gradually increases pulse width until reaching target pulse.
     */
    void update() {
      if (!armed) return;

      // Make sure it's done arming
      if (millis() > armEndMillis) {
        if      (pulseMicros < targetPulse) pulseMicros++;
        else if (pulseMicros > targetPulse) pulseMicros--;

        updateTimerOutput();
      }
    }

  private:
    enum class Timer { T4, T5 };
    enum class Channel { A, B };

    Timer timer;
    Channel channel;

    uint8_t pin;

    uint16_t pulseMicros;
    uint16_t targetPulse;
    
    bool armed;
    uint32_t armEndMillis;

    static bool timer4Initialized;
    static bool timer5Initialized;

    /**
     * @brief Sets the Timer identifications based on the pin, then initializes
     * the timer and attaches the channel.
     * 
     * @param pin uint8_t. Pin to setup.
     */
    bool setupTimerForPin(uint8_t pin) {
      switch (pin) {
        case 6:
          timer   = Timer::T4;
          channel = Channel::A;
          break;

        case 7:
          timer   = Timer::T4;
          channel = Channel::B;
          break;

        case 46:
          timer   = Timer::T5;
          channel = Channel::A;
          break;

        case 45:
          timer   = Timer::T5;
          channel = Channel::B;
          break;

        default:
          return false;
      }

      initTimer();
      attachChannel();
      
      return true;
    }

    /**
     * @brief .
     */
    void initTimer() {
      switch (timer) {
        case Timer::T4:
          if (!timer4Initialized) {
            TCCR4A = 0;
            TCCR4B = 0;
  
            TCCR4A |= (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43);
  
            TCCR4B |= (1 << CS41);
  
            ICR4 = 40000;

            timer4Initialized = true;
          }
          break;
        
        case Timer::T5:
          if (!timer5Initialized) {
            TCCR5A = 0;
            TCCR5B = 0;
  
            TCCR5A |= (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53);
  
            TCCR5B |= (1 << CS51);
  
            ICR5 = 40000;

            timer5Initialized = true;
          }
          break;
      }
    }

    /**
     * @brief .
     */
    void attachChannel() {
      switch (timer) {
        case Timer::T4:
          if      (channel == Channel::A) TCCR4A |= (1 << COM4A1);
          else if (channel == Channel::B) TCCR4A |= (1 << COM4B1);
          break;

        case Timer::T5:
          if      (channel == Channel::A) TCCR5A |= (1 << COM5A1);
          else if (channel == Channel::B) TCCR5A |= (1 << COM5B1);
          break;
      }
    }

    /**
     * @brief Converts microseconds to timer ticks and sets compare register.
     */
    void updateTimerOutput() {
      uint16_t ticks = pulseMicros * 2;
      
      switch (timer) {
        case Timer::T4:
          if      (channel == Channel::A) OCR4A = ticks;
          else if (channel == Channel::B) OCR4B = ticks;
          break;

        case Timer::T5:
          if      (channel == Channel::A) OCR5A = ticks;
          else if (channel == Channel::B) OCR5B = ticks;
          break;
      }
    }
};

/**
 * @brief IMU Driver, calculate pitch, roll and yaw from the MPU-6050.
 * 
 * When calculating the pitch and roll, its somewhat accurate because both
 * values can be corrected by the accelerometer. But when calculating the yaw,
 * it will drift slightly, asuming around 5°/min, there is no correction for
 * this in the class.
 * 
 * @param address uint8_t. The MPU-6050 address, defualt is 0x68
 * @param intervalMicros uint32_t. The interval between hardware reads.
 */
class InertialUnit {
  public:
    InertialUnit(uint32_t intervalMicros, uint8_t address=0x68)
      : mpu_address(address), timer(intervalMicros), roll(0), pitch(0), yaw(0),
        dt(intervalMicros * 1e-6f)
    {}

    /**
     * @brief Initialize the wire, and wake up the MPU-6050.
     * 
     * When waking up the MPU, the wire starts a transmission to the address.
     * It then writes a 0 value first to the power management register address
     * to tell the chip to wake up. Then it ends the transmission, and
     * releasing the bus.
     */
    void begin() {
      Wire.begin();

      // Wake MPU-6050
      Wire.beginTransmission(mpu_address);
      Wire.write(0x6B); // Power management address
      Wire.write(0x00); // Write 0 to sleep
      Wire.endTransmission(true); // Release the bus
    }

    /**
     * @brief Call every loop. Reads hardware values when time scheduler is
     * ready.
     * 
     * When reading hardware values it starts at the start register which is the
     * accelerometer X. It then ends the transmission before reads, but doesn't
     * release the bus. Right after, the function performs a request of
     * 14 bytes. This is the exact length of [accel + temp + gyro].
     * 
     * There are 3 constants are defined in the update loop, find them listed
     * below along with an explaination on how they were found and their role:
     * 1. `half16bits`, this is just the half of 16 bits. This is used when
     * calculating acceleration angles. Becuase the hardware measures +-2g,
     * dividing by this number ensures correct angles.
     * 2. `degPerSec`, this is used to calculate the degrees per second when
     * converting the gyroscopics. The number comes from the datasheet and is
     * specified as the LSB (Least Significant Bit), where 1°/s = 131 LSB.
     * 3. `alphaFilter`, this is a bias filter, used when calculating roll and
     * pitch to get the best of both worlds and a result thats stable enough
     * throughout multiple minutes.
     * 
     * It then reads all the raw values. It uses the constant defined variables
     * to integrate the raw values into roll, pitch and yaw. After that it
     * defines the acceleration angles (also using the constants), then it
     * calculates the accelleration of roll and pitch (not yaw since it's not
     * possible with the current hardware).
     * 
     * From there it uses a bias where it blends gyro and acceleration to get
     * the best of both worlds. For stabilization gyro is good at short-term
     * motion and bad at long-term motion. But accelerometer is good at
     * long-term gravity angle and bad at suppressing noise & vibrations.
     */
    void update() {
      if (!timer.ready()) return;
      
      // Define constant units
      constexpr float half16bits  = 16384.0f;
      constexpr float degPerSec   = 131.07f;
      constexpr float alphaFilter = 0.98f;

      // Burst read 14 bytes
      Wire.beginTransmission(mpu_address);
      Wire.write(0x3B); // Start register for accelerometer data
      Wire.endTransmission(false); // Dont release the bus
      Wire.requestFrom(mpu_address, (uint8_t)14);

      // Read values
      int16_t ax = read16();
      int16_t ay = read16();
      int16_t az = read16();
      read16();
      int16_t gx = read16();
      int16_t gy = read16();
      int16_t gz = read16();

      // Convert gyroscopics
      float gxf = gx / degPerSec;
      float gyf = gy / degPerSec;
      float gzf = gz / degPerSec;

      // Integrate gyroscopics
      roll  += gxf * dt;
      pitch += gyf * dt;
      yaw   += gzf * dt;

      // Acceleration angles
      float axf = ax / half16bits;
      float ayf = ay / half16bits;
      float azf = az / half16bits;

      // Calculate accelleration
      float rollAcc  = atan2(ayf, azf) * RAD_TO_DEG;
      float pitchAcc = atan2(-axf, sqrt(ayf*ayf + azf*azf)) * RAD_TO_DEG;

      // Calculate the pitch and roll angle
      roll  = alphaFilter * roll  + (1 - alphaFilter) * rollAcc;
      pitch = alphaFilter * pitch + (1 - alphaFilter) * pitchAcc;
    }

    /**
     * @brief Get the roll value in degrees.
     */
    float getRoll() const { return roll; };
    
    /**
     * @brief Get the pitch value in degrees.
     */
    float getPitch() const { return pitch; };
    
    /**
     * @brief Get the yaw value in degrees.
     * 
     * World physics causes it to drift over short time, don't use for
     * important things, unreliable!
     */
    float getYaw() const { return yaw; };

  private:
    uint8_t mpu_address;
    IntervalMicros timer;

    float roll, pitch, yaw;

    float dt;

    /**
     * @brief Internal function for reading 16 bits from wire.
     * 
     * @return int16_t. 2 Byte result from the read.
     */
    inline int16_t read16() {
      return (Wire.read() << 8) | Wire.read();
    }
};

/**
 * @brief Send and recieve packets of 5 bytes. Used to communicate between
 * remote and drone controller.
 * 
 * Expects constant sending from the transmitter copy of this class.
 * 
 * Drone should NOT send data! (slows down runtime too much).
 */
class Radio {
  public:
    Radio() {}

    /**
     * @brief Initialize the RadioHead driver.
     */
    void begin() {
      driver.init();
    }

    /**
     * @brief Call every loop. Reads from the driver internal buffer if new data
     * is available.
     * 
     * When recieving a packet, the function checks if the length recieved is
     * exactly 5 bytes. This is to filter off noise or broken packets.
     */
    void update() {
      uint8_t length = sizeof(buffer);

      // Read if theres a packet ready
      if (driver.recv(buffer, &length) && length == 5) {
        if (rxPacket.decode(buffer)) {
          signal = true;
          lastRxTime = millis();
        }
      }

      // Check if the radio lost signal to the transmitter
      if (millis() - lastRxTime > 100) {
        signal = false;
      }
    }

    /**
     * @brief Send a packets data by reference. Blocking stack until finished!
     * 
     * RadioHead can send 2000bps. This class sends 5 bytes + overhead which is
     * approximatly ~80 bits. That makes sending take around ~40ms.
     * 
     * @param packet RemotePacket&. Reference to the remote packet whos data to
     * encode and send.
     */
    void send(const RemotePacket& packet) {
      packet.encode(buffer);
      driver.send(buffer, sizeof(buffer));
      driver.waitPacketSent();
    }

    /**
     * @brief Check if the RH has a signal.
     * 
     * Loses signal if theres more than 100ms since the last recorded packet
     * recieved.
     * 
     * @return bool. True if theres a signal.
     */
    bool hasSignal() const { return signal; }

    /**
     * @brief Exposes the packet, read-only access.
     */
    const RemotePacket& getRxPacket() const { return rxPacket; }
  
  private:
    RH_ASK driver;

    RemotePacket rxPacket;
    uint8_t buffer[5];

    bool signal;
    uint32_t lastRxTime;
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
    * IDLE. Checks if the timer is ready, when ready it fires a pulse on the
    * trigger pin and marks the pulse start, then changes the state to TRIGGER.
    * 
    * TRIGGER. Checks if 10µs past since firing the pulse, if so it sets the
    * trigger PIN to low and marks the pulse start, then changes the state to
    * WAIT_ECHO.
    * 
    * WAIT_ECHO. Checks if echo is high, if so, mark the pulse start and change
    * the state to MEASURE.
    * 
    * MEASURE. Checks if the echo is low, if so, grab the pulse duration, then
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

/**
 * @brief Mixes throttle, roll, pitch & yaw and outputs values to the ESCs.
 * 
 * Uses the following mixing (t = throttle, p = pitch, r = roll, y = yaw):
 * m1 = t + p + r - y
 * m2 = t + p - r + y
 * m3 = t - p - r - y
 * m4 = t - p + r + y
 * 
 * @param m1 uint8_t. Front-left motor pin.
 * @param m2 uint8_t. Front-right motor pin.
 * @param m3 uint8_t. Back-right motor pin.
 * @param m4 uint8_t. Back-left motor pin.
 */
class MotorMix {
  public:
    MotorMix(Motor* m1, Motor* m2, Motor* m3, Motor* m4) : motors{m1, m2, m3, m4} {}
  
    /**
     * @brief Call to update the ESC values. Mixes inputs to run the quadcopter
     * motors the correct way.
     * 
     * @param throttle float. The throttle input.
     * @param roll float. The roll input.
     * @param pitch float. The pitch input.
     * @param yaw float. The yaw input.
     */
    void update(float throttle, float roll, float pitch, float yaw) {
      float m[4];

      // Mix values for each motor
      m[0] = throttle + pitch + roll - yaw; // Front-left
      m[1] = throttle + pitch - roll + yaw; // Front-right
      m[2] = throttle - pitch - roll - yaw; // Back-right
      m[3] = throttle - pitch + roll + yaw; // Back-left

      // Write to motor ESCs
      for (int i=0; i<4; ++i) {
        m[i] = constrain(m[i], 0, 1000); // µs width max size
        motors[i]->setPulseWidth(map(m[i], 0, 1000, 1000, 2000));
      }

      // Call update on each motor
      for (int i=0; i<4; ++i)
        motors[i]->update();
    }

  private:
    Motor* motors[4];
};

// ========================================================================== //
// CLASS INSTANTIATION                                                        //
// ========================================================================== //

// Allocate memory for motor timer flags
bool Motor::timer4Initialized = false;
bool Motor::timer5Initialized = false;

// Mount this shit as close to center of mass.
// It has to have horizontal reading of pitch=0 and roll=0 to ensure NO bias.
InertialUnit IMU(1000);

// Mounting should be like the following:
/*
  TIP: Blade must cut into the direction.
  Front-left  = CCW
  Front-right = CW
  Back-right  = CCW
  Back-left   = CW
*/
Motor motorFR(45); // Pin 9 for OC1A (Output Compare 1 <Channel> A)

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

/**
 * @brief Called by system at the startup once.
 */
void setup() {
  if (VERBOSE) Serial.begin(9600);

  IMU.begin();
  motorFR.begin();
  motorFR.arm();

  // Non-blocking 3s arming
  uint32_t startMillis = millis();
  while (millis() - startMillis < 3000) {
      motorFR.update();
  }
}

/**
 * @brief Called by system every time possible.
 */
void loop() {
  // Read sensors
  // Read radio
  // Run PIDs
  // Mix outputs -> 4 motors
  // Write PWM

  IMU.update();
  motorFR.update();

  float pitch = IMU.getPitch();

  long maxPitch = 10.0;
  int us = map(max(0, pitch), 0, maxPitch, 1600, 2000);

  Serial.println(pitch);

  motorFR.setPulseWidth(us);
}

// ========================================================================== //
// MAIN FUNCTIONS                                                             //
// ========================================================================== //



// ========================================================================== //
// HELPER FUNCTIONS                                                           //
// ========================================================================== //

