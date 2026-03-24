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

// Debug logging, ensure only one toggled at once to avoid console clutter.
constexpr bool DEBUG_MAIN       = true; // Main information out
constexpr bool DEBUG_IMU        = false;
constexpr bool DEBUG_RADIO      = false;
constexpr bool DEBUG_MOTOR      = false;
constexpr bool DEBUG_MOTORMIX   = false;
constexpr bool DEBUG_USENSOR    = false;
constexpr bool DEBUG_BATTERY    = false;
constexpr bool NO_REMOTE_TEST   = true;

// Timings
constexpr int INERTIAL_INTERVAL_MICROS = 1000;
constexpr int USENSOR_INTERVAL_MICROS  = 1000;

// Pich PID tuning values
constexpr float PITCH_P = 3.0f;
constexpr float PITCH_I = 0.5f;
constexpr float PITCH_D = 3.0f;
constexpr int PITCH_OUTPUT_LIMIT    = 300;
constexpr int PITCH_INTEGRAL_LIMIT  = 45;
constexpr int PITCH_INTERVAL_MICROS = 15000;

// Roll PID tuning values
constexpr float ROLL_P = 3.0f;
constexpr float ROLL_I = 0.5f;
constexpr float ROLL_D = 3.0f;
constexpr int ROLL_OUTPUT_LIMIT    = 300;
constexpr int ROLL_INTEGRAL_LIMIT  = 45;
constexpr int ROLL_INTERVAL_MICROS = 15000;

// Yaw PID tuning values
constexpr float YAW_P = 1.2f;
constexpr float YAW_I = 0.1f;
constexpr float YAW_D = 0.0f;
constexpr int YAW_OUTPUT_LIMIT    = 300;
constexpr int YAW_INTEGRAL_LIMIT  = 45;
constexpr int YAW_INTERVAL_MICROS = 15000;

// Remote tolerances
constexpr float PITCH_TOLERANCE = 20.0f;
constexpr float ROLL_TOLERANCE  = 20.0f;
constexpr float YAW_TOLERANCE   = 5.0f; // Incementing
constexpr float THROTTLE_RANGE  = 400.0f;
constexpr int16_t THROTTLE_STEP = 5;

// Thresholds
constexpr float CRASH_ANGLE_DEG             = 20.0f;
constexpr float LANDING_SHUTOFF_DISTANCE_CM = 5.0f;

// Throttles
constexpr uint16_t HOVER_THROTTLE   = 1450;
constexpr uint16_t LANDING_THROTTLE = 1050;

// ========================================================================== //
// PIN DEFINITIONS                                                            //
// ========================================================================== //

// Motor ESCs
constexpr uint8_t MOTOR_FR = 45;
constexpr uint8_t MOTOR_FL = 46;
constexpr uint8_t MOTOR_BR = 6;
constexpr uint8_t MOTOR_BL = 7;

// Ultrasonic pins
constexpr uint8_t USENSOR_TRIGGER = 3;
constexpr uint8_t USENSOR_ECHO    = 4;

// Indicator LEDs
constexpr uint8_t ERROR_LED_PIN   = 8;
constexpr uint8_t WARNING_LED_PIN = 9;

// Battery cell pins
constexpr uint8_t BATTERY_CELL_1 = A2;
constexpr uint8_t BATTERY_CELL_2 = A3;
constexpr uint8_t BATTERY_CELL_3 = A4;

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
     *
     * @return uint32_t. Current interval in use.
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
      targetPulse = us;
    }

    /**
     * @brief Disarm the motor, shutting it off.
     */
    void disarm() {
      armed = false;
      pulseMicros = 0;
      updateTimerOutput();
    }

    /**
     * @brief Call every update, only runs if motor is armed.
     * 
     * Gradually increases pulse width until reaching target pulse.
     */
    void update() {
      //if (!armed) return;

      // Make sure it's done arming
      if (millis() > armEndMillis) {
        if      (pulseMicros < targetPulse) pulseMicros += THROTTLE_STEP;
        else if (pulseMicros > targetPulse) pulseMicros -= THROTTLE_STEP;

        // Debug logging
        if (DEBUG_MOTOR) {
          Serial.print(F("Pulse length: "));
          Serial.print(pulseMicros);
          Serial.println(F(" us"));
        }
      } else {
        // During arming
        pulseMicros = 1000;
      }

      updateTimerOutput();
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
     * 
     * @return bool. 
     * @return bool. If otherwise.
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
     * @brief Initialize the identified timer, setting the PWM to fast mode.
     * 
     * Timer counts from 0 - Input Capture Register. Doesn't configure the
     * Output Compare Register. Sets the prescaler to 8
     */
    void initTimer() {
      switch (timer) {
        case Timer::T4:
          if (!timer4Initialized) {
            // Clear registers
            TCCR4A = 0;
            TCCR4B = 0;
  
            // Configure for fast Waveform Generation Mode
            TCCR4A |= (1 << WGM41);
            TCCR4B |= (1 << WGM42) | (1 << WGM43);
            
            // Set prescaler (8 = 2MHz)
            TCCR4B |= (1 << CS41);
  
            ICR4 = 40000;

            timer4Initialized = true;
          }
          break;
        
        case Timer::T5:
          if (!timer5Initialized) {
            // Clear registers
            TCCR5A = 0;
            TCCR5B = 0;
            
            // Configure for fast Waveform Generation Mode
            TCCR5A |= (1 << WGM51);
            TCCR5B |= (1 << WGM52) | (1 << WGM53);
            
            // Set prescaler (8 = 2MHz)
            TCCR5B |= (1 << CS51);
  
            ICR5 = 40000;

            timer5Initialized = true;
          }
          break;
      }
    }

    /**
     * @brief Enable non-inverting PWM on the correct channel.
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

      // Debug logging
      if (DEBUG_IMU) {
        Serial.print(F("Pitch: "));
        Serial.print(pitch);
        Serial.print(F(" | Roll: "));
        Serial.print(roll);
        Serial.print(F(" | Yaw: "));
        Serial.println(yaw);
      }
    }

    /**
     * @brief Get the roll value in degrees.
     *
     * @return float. Measured roll.
     */
    float getRoll() const { return roll; }
    
    /**
     * @brief Get the pitch value in degrees.
     *
     * @return float. Measured pitch.
     */
    float getPitch() const { return pitch; }
    
    /**
     * @brief Get the yaw value in degrees.
     * 
     * World physics causes it to drift over short time, don't use for
     * important things, unreliable!
     *
     * @return float. Measured yaw.
     */
    float getYaw() const { return yaw; }

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
    Radio() : signal(false), lastRxTime(0) {}

    /**
     * @brief Initialize the RadioHead driver.
     */
    void begin() {
      if (!driver.init() && DEBUG_RADIO)
        Serial.println("Failed to initialize radio driver!");
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

          // Debug logging
          if (DEBUG_RADIO) {
            Serial.print(F("Signal: "));
            Serial.print(signal ? "YES" : "NO");
            Serial.print(F(" | Pitch: "));
            Serial.print(rxPacket.pitch);
            Serial.print(F(" | Roll: "));
            Serial.print(rxPacket.roll);
            Serial.print(F(" | Yaw: "));
            Serial.print(rxPacket.yaw);
            Serial.print(F(" | Throttle: "));
            Serial.print(rxPacket.throttle);
            Serial.print(F(" | Autoland: "));
            Serial.println(rxPacket.autoland);
          } 
        }
      }

      // Check if the radio lost signal to the transmitter
      if (millis() - lastRxTime > 500) {
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

      // Debug logging
      if (DEBUG_RADIO) {
        Serial.print(F("Pitch: "));
        Serial.print(packet.pitch);
        Serial.print(F(" | Roll: "));
        Serial.print(packet.roll);
        Serial.print(F(" | Yaw: "));
        Serial.print(packet.yaw);
        Serial.print(F(" | Throttle: "));
        Serial.print(packet.throttle);
        Serial.print(F(" | Autoland: "));
        Serial.println(packet.autoland);
      }

      // Send packet
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
     *
     * @return RemotePacket&. Reference to the decoded packet containing
     * the newest values available.
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

            // Debug logging
            if (DEBUG_USENSOR) {
              Serial.print(F("Distance: "));
              Serial.print(distanceCm);
              Serial.println(F(" cm"));
            }

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
 * Assumes correct mouning of motors and their spin-direcitons.
 *
 * @param m1 uint8_t. Front-left motor pin.
 * @param m2 uint8_t. Front-right motor pin.
 * @param m3 uint8_t. Back-right motor pin.
 * @param m4 uint8_t. Back-left motor pin.
 */
class MotorMix {
  public:
    MotorMix(Motor* m1, Motor* m2, Motor* m3, Motor* m4)
      : motors{m1, m2, m3, m4}
    {}
  
    /**
     * @brief Disarm all motors, powering them down.
     */
    void disarm() {
      for (int i=0; i<4; ++i) {
        motors[i]->disarm();
      }
    }

    /**
     * @brief Arm all motors, takes around 3 seconds. This call is blocking.
     */
    void arm() {
      for (int i=0; i<4; ++i) {
        motors[i]->begin();
        motors[i]->arm();
      }
    }

    /**
     * @brief Call to update the ESC values. Mixes inputs to run the quadcopter
     * motors the correct way.
     * 
     * @param throttle float. The throttle input.
     * @param roll float. The roll input.
     * @param pitch float. The pitch input.
     * @param yaw float. The yaw input.
     */
    void update(float throttle, float pitch, float roll, float yaw) {
      float m[4];

      // Mix values for each motor
      m[0] = throttle + pitch + roll - yaw; // Front-left
      m[1] = throttle + pitch - roll + yaw; // Front-right
      m[2] = throttle - pitch - roll - yaw; // Back-right
      m[3] = throttle - pitch + roll + yaw; // Back-left

      // Debug pulse widths
      if (DEBUG_MOTORMIX) {
        Serial.print(F("FL: "));
        Serial.print(m[0]);
        Serial.print(F(" | FR: "));
        Serial.print(m[1]);
        Serial.print(F(" | BR: "));
        Serial.print(m[2]);
        Serial.print(F(" | BL: "));
        Serial.println(m[3]);
      }

      // Write to motor ESCs
      for (int i=0; i<4; ++i) {
        m[i] = constrain(m[i], 1000, 2000); // µs width max size
        motors[i]->setPulseWidth(m[i]);
      }

      // Call update on each motor
      updateMotors();
    }

    /**
     * @brief DEBUGGING
     */
    void updateMotors() {
      for (int i=0; i<4; ++i)
        motors[i]->update();
    }

    /**
     * @brief DEBUGGING
     */
    void setMotorsPWM(uint16_t us) {
      for (int i=0; i<4; ++i)
        motors[i]->setPulseWidth(us);
    }

  private:
    Motor* motors[4];
};

/**
 * @brief Reads voltage limits from a LiPo S3 battery.
 * 
 * Makes sure the battery connected is within safe limits, and calulcates the
 * percent of safe charge left.
 * 
 * When measuring the voltage, use an IRL voltage divider for each cell,
 * currently, the setup is:
 * C1 = 0   ohm.
 * C2 = 18k ohm.
 * C3 = 27k ohm.
 * 
 * @param cell1Pin uint8_t. The analog pin the first cell is connected to.
 * @param cell2Pin uint8_t. The analog pin the second cell is connected to.
 * @param cell3Pin uint8_t. The analog pin the third cell is connected to.
 */
class BatteryManagement {
  public:
    BatteryManagement(uint8_t cell1Pin, uint8_t cell2Pin, uint8_t cell3Pin,
                      uint32_t intervalMicros=1000000)
      : cell1(cell1Pin), cell2(cell2Pin), cell3(cell3Pin), timer(intervalMicros)
    {}
  
    /**
     * @brief Call this every update. Reads individual cell pins, and calculates
     * the voltage on each cell.
     */
    void update() {
      if (timer.ready()) {
        // Read cells
        float rawC1 = analogRead(cell1);
        float rawC2 = analogRead(cell2);
        float rawC3 = analogRead(cell3);

        // Convert to voltage at pin
        float V_C1 = rawC1 * (5 / 1023.0f) * 2.0f;
        float V_C2 = rawC2 * (5 / 1023.0f) * 2.8f;
        float V_C3 = rawC3 * (5 / 1023.0f) * 3.7f;
        
        // Calculate cell voltages
        charge1 = V_C1;
        charge2 = V_C2 - V_C1;
        charge3 = V_C3 - V_C2;

        // Debug logging
        if (DEBUG_BATTERY) {
          Serial.print(F("C1: "));
          Serial.print(charge1);
          Serial.print(F("V ("));
          Serial.print(convertV2Pct(charge1) * 100);
          Serial.print(F("%) | C2: "));
          Serial.print(charge2);
          Serial.print(F("V ("));
          Serial.print(convertV2Pct(charge2) * 100);
          Serial.print(F("%) | C3: "));
          Serial.print(charge3);
          Serial.print(F("V ("));
          Serial.print(convertV2Pct(charge3) * 100);
          Serial.print(F("%) | Total: "));
          Serial.print(getBatteryCharge() * 100);
          Serial.println(F("%"));
        }
      }
    }

    /**
     * @brief Easy way to see if the battery is safe to draw from.
     * 
     * @param minPct float. The minimum percent that is considered safe,
     * defaults to 0.1
     * 
     * @return bool. True if it's safe to draw power from.
     * @return bool. False otherwise.
     */
    bool isSafe(float minPct=0.1f) {
      return true; //getBatteryCharge() > minPct;
    }

    /**
     * @brief Get the percent of charge left in cell 1.
     *
     * @return float. Charge left in cell, range values from 0-1.
     */
    float getChargeCell1() const { return convertV2Pct(charge1); }

    /**
     * @brief Get the percent of charge left in cell 2.
     *
     * @return float. Charge left in cell, range values from 0-1.
     */
    float getChargeCell2() const { return convertV2Pct(charge2); }

    /**
     * @brief Get the percent of charge left in cell 3.
     *
     * @return float. Charge left in cell, range values from 0-1.
     */
    float getChargeCell3() const { return convertV2Pct(charge3); }

    /**
     * @brief Get the total battery charge percentage left.
     *
     * @return float. Charge left in battery, value ranges from 0-1.
     */
    float getBatteryCharge() const {
      return convertV2Pct(charge1 + charge2 + charge3, 12.6f, 9.6f);
    }

  private:
    uint8_t cell1, cell2, cell3;

    float charge1, charge2, charge3;

    IntervalMicros timer;

    /**
     * @brief Get the measured battery charge by measured voltage.
     * 
     * Tuned to a S3 LiPo battery cell!
     * 
     * @param measuredV float. Measured voltage (eg. using a multimeter).
     * @param fullV float. The voltage when fully charged. Defaults
     * to 12.6V.
     * @param emptyV float. Voltage when the battery cell is empty. This should
     * be the absolute lowest, discharging below this will permanently damage
     * the battery.
     * 
     * @return float. 0 - 1 range of the calculated battery cell charge left.
     */
    float convertV2Pct(float measuredV, float fullV=4.2, float emptyV=3.2) {
        return constrain((measuredV - emptyV) / (fullV - emptyV), 0.0f, 1.0f);
    }
};

/**
 * @brief Simple non-PWM led controller.
 *
 * @param pin uint8_t. Pin that the LED is connected to.
 */
class LED {
  public:
    LED(uint8_t pin)
      : pin(pin), blinking(false), state(false), timer(250000)
    {
      // Set pin mode
      pinMode(pin, OUTPUT);
    }

  /**
   * @brief Call every loop. Handles blinking mode.
   */
  void update() {
    if (blinking && timer.ready()) {
      state = !state;
      digitalWrite(pin, state ? HIGH : LOW);
    }
  }

  /**
   * @brief Turn on the LED.
   */
  void on() {
    blinking = false;
    state = true;
    digitalWrite(pin, HIGH);
  }

  /**
   * @brief Turn off the LED.
   */
  void off() {
    blinking = false;
    state = false;
    digitalWrite(pin, LOW);
  }

  /**
   * @brief Get the light state of the LED.
   *
   * @return uint8_t. 0 = off, 1 = on, 2 = blinking
   */
  uint8_t getState() {
    return blinking ? 2 : state;
  }

  /**
   * @brief Blink the LED at a specified rate.
   *
   * @param rateMs uint32_t. Rate to blink at in milliseconds.
   */
  void blink(uint32_t rateMs) {
    if (blinking) return;

    blinking = true;
    timer.setInterval(rateMs * 1000);
    timer.reset();
  }

  private:
    bool blinking;
    bool state;

    IntervalMicros timer;

    uint8_t pin;
};

/**
 * @brief Central brain for the drone, ties together individual components.
 *
 * Onboard indicator LEDs meaning:
 * - Error LED static: Drone entered crash protection mode, restart needed.
 * - Error LED blinking: Critically low battery detected, wont power on.
 * - Warning LED static: Landing mode active
 * - Warning LED blinking: Signal lost from remote control OR arming drone.
 */
class FlightController {
  public:
    FlightController(Radio& radio, InertialUnit& imu, UltrasonicSensor& uSensor,
                     BatteryManagement& battery, PID& pitchPID, PID& rollPID,
                     PID& yawPID, MotorMix& motorMix, uint8_t errorPin,
                     uint8_t warningPin)
      : radio(radio), imu(imu), uSensor(uSensor), battery(battery),
        pitchPID(pitchPID), rollPID(rollPID), yawPID(yawPID),
        motorMix(motorMix), errorLED(errorPin), warningLED(warningPin),
        targetPitch(0), targetRoll(0), targetYaw(0), targetThrottle(0),
        armingStart(0), strState("IDLE")
    {}

    /**
     * @brief Arm the motors, and reset the PID controllers.
     */
    void begin() {
      // Initialize sensors
      imu.begin();
      radio.begin();

      // Reset PID controllers
      pitchPID.reset();
      rollPID.reset();
      yawPID.reset();

      state = FlightState::ARMING;
      armingStart = 0;

      if (DEBUG_MAIN)
        Serial.println("Started flight controller!");
    }

    /**
     * @brief Call every loop. Updates state machine and all sensors.
     */
    void update() {
      // Update all sensors
      imu.update();
      uSensor.update();
      radio.update();
      battery.update();
      errorLED.update();
      warningLED.update();

      // Safety check battery
      if (!battery.isSafe()) {
        state = FlightState::NO_BATTERY;
      }

      // Check if the drone has remote signal
      bool remoteSignal = radio.hasSignal();
      // Don't overrule landing indicator
      if (!remoteSignal
          && warningLED.getState() == 0
          && state != FlightState::ARMING)
        warningLED.blink(250);

      // Log output
      if (DEBUG_MAIN) {
        Serial.print(F("Battery: "));
        Serial.print(battery.getBatteryCharge() * 100);
        Serial.print(F("% | State: "));
        Serial.print(strState);
      }

      // State machine
      switch (state) {
        case FlightState::ARMING:
          strState = "ARMING";

          //if (warningLED.getState() == 2) break;
          warningLED.blink(250);

          if (armingStart == 0) {
            motorMix.arm();
            armingStart = millis();
          }

          motorMix.setMotorsPWM(1000);
          motorMix.updateMotors();

          if (millis() - armingStart >= 5000) {
            state = FlightState::FLYING;
            //motorMix.setMotorsPWM(1600); // Works when bypassing updateFlight()
          }

          break;

        case FlightState::FLYING:
          strState = "FLYING";
          if (warningLED.getState() == 2)
            warningLED.off();

          //motorMix.updateMotors(); // This works, bypassing updateFlight()
          updateFlight();
          break;

        case FlightState::AUTO_LAND:
          strState = "AUTO_LAND";
          if (warningLED.getState() == 1) break;

          warningLED.on();
          performAutoLand();
          break;

        case FlightState::CRASH:
          strState = "CRASH";
          if (errorLED.getState() == 1) break;

          errorLED.on();
          motorMix.disarm();
          break;

        case FlightState::NO_BATTERY:
          strState = "NO_BATTERY";
          if (errorLED.getState() == 2) break;

          errorLED.blink(250);
          motorMix.disarm();
          break;
        
        case FlightState::IDLE:
          strState = "IDLE";
          break;
      }

      if (DEBUG_MAIN)
        Serial.println(F(""));
    }

    /**
     * @brief Update the target pitch, roll, yaw and throttle of the drone.
     *
     * All parameters except throttle are expected to be in units of degrees.
     *
     * @param pitch float. Target pitch.
     * @param roll float. Target roll.
     * @param yaw float. Target yaw.
     * @param throttle float. Target throttle, value between 1000-2000µs.
     */
    void setTargets(float pitch, float roll, float yaw, float throttle) {
      targetPitch     = pitch;
      targetRoll      = roll;
      targetYaw       = yaw;
      targetThrottle  = throttle;
    }

  private:
    enum class FlightState {
      ARMING, FLYING, AUTO_LAND, CRASH, NO_BATTERY, IDLE
    };
    FlightState state;

    Radio& radio;
    InertialUnit& imu;
    UltrasonicSensor& uSensor;
    BatteryManagement& battery;
    PID& pitchPID;
    PID& rollPID;
    PID& yawPID;
    MotorMix& motorMix;
    LED warningLED, errorLED;

    String strState;

    uint32_t armingStart;

    float targetPitch, targetRoll, targetYaw, targetThrottle;

    /**
     * @brief Updates the motor mix based on remote values and more.
     *
     * Using remote values it maps the target angles and throttle based on
     * the controller configuration. It also checks for crashes and autoland
     * status.
     */
    void updateFlight() {
      RemotePacket packet = radio.getRxPacket();

      // Convert remote inputs
      targetPitch    = mapStick(packet.pitch, PITCH_TOLERANCE);
      targetRoll     = mapStick(packet.roll, ROLL_TOLERANCE);
      targetYaw     += mapStick(packet.yaw, YAW_TOLERANCE);
      targetThrottle = HOVER_THROTTLE + ((float)packet.throttle - 16.0f)
                         / 16.0f * THROTTLE_RANGE; // Should be >1000
        
      if (NO_REMOTE_TEST) {
        targetPitch = 0;
        targetRoll  = 0;
        targetYaw   = 0;
        targetThrottle = HOVER_THROTTLE;
      }

      // Read IMU
      float pitch = imu.getPitch();
      float roll  = imu.getRoll();
      float yaw   = imu.getYaw();

      // Log targets
      if (DEBUG_MAIN) {
        Serial.print(F(" | Pitch: "));
        Serial.print(packet.pitch);
        Serial.print(F("/"));
        Serial.print(pitch);
        Serial.print(F(" | Roll: "));
        Serial.print(packet.roll);
        Serial.print(F("/"));
        Serial.print(roll);
        Serial.print(F(" | Yaw: "));
        Serial.print(packet.yaw);
        Serial.print(F("/"));
        Serial.print(yaw);
        Serial.print(F(" | Throttle: "));
        Serial.print(targetThrottle);
      }

      // Switch to autoland if needed
      if (packet.autoland) {
        state = FlightState::AUTO_LAND;
        return;
      }

      // Detech crash
      if (abs(pitch) > CRASH_ANGLE_DEG || abs(roll) > CRASH_ANGLE_DEG) {
        state = FlightState::CRASH;
        return;
      }

      // Update PIDs (when ready)
      float pitchCorr = 0;
      float rollCorr  = 0;
      float yawCorr   = 0;

      // Update under same tick to eliminate possible oscilation
      if (pitchPID.ready()) {  
        pitchCorr = pitchPID.update(targetPitch, pitch);
        rollCorr = rollPID.update(targetRoll, roll);
        yawCorr = yawPID.update(targetYaw, yaw);
      }

      // Update motors
      motorMix.update(targetThrottle, pitchCorr, rollCorr, yawCorr);
    }

    /**
     * @brief Performs autolanding of drone. Ignoring remote control inputs.
     *
     * Keeping the angles at 0, 0 and unchanged yaw, it slowly decreases lift
     * until close to the ground, at which the motors are disarmed.
     */
    void performAutoLand() {
      // Read IMU
      float pitch = imu.getPitch();
      float roll  = imu.getRoll();
      float yaw   = imu.getYaw();

      // Update PIDs (when ready)
      float pitchCorr = 0;
      float rollCorr  = 0;
      float yawCorr   = 0;

      // Run under same tick
      if (pitchPID.ready()) {
        pitchCorr = pitchPID.update(0, pitch);
        rollCorr = rollPID.update(0, roll);
        yawCorr = yawPID.update(targetYaw, yaw);
      }

      // Shut off if close to ground
      float distance = uSensor.getDistance();
      if (distance < LANDING_SHUTOFF_DISTANCE_CM) {
        motorMix.disarm();
        state = FlightState::IDLE;
        return;
      }

      motorMix.update(LANDING_THROTTLE, pitchCorr, rollCorr, yawCorr);
    }

    /**
     * @brief Map remote control value to target +- range.
     *
     * @param value uint8_t. Raw value from remote.
     * @param range uint8_t. +- range/tolerance.
     *
     * @return float. Mapped range.
     */
    float mapStick(uint8_t value, float range) {
      return ((float)value - 16.0f) / 16.0f * range;
    }
};

// ========================================================================== //
// CLASS INSTANTIATION                                                        //
// ========================================================================== //

// Allocate memory for motor timer flags, set default to false
bool Motor::timer4Initialized = false;
bool Motor::timer5Initialized = false;

// Instantiate the inertial measurement unit driver
InertialUnit IMU(INERTIAL_INTERVAL_MICROS);

// Instantiate all motor ESC drivers
Motor motorFR(MOTOR_FR);
Motor motorFL(MOTOR_FL);
Motor motorBR(MOTOR_BR);
Motor motorBL(MOTOR_BL);

// Instatiate motor mixer
MotorMix motorMix(&motorFL, &motorFR, &motorBR, &motorBL);

// Instatiate the radio controller
Radio radio;

// Instatiate the ultrasonic sensor driver
UltrasonicSensor uSensor(
  USENSOR_TRIGGER, USENSOR_ECHO, USENSOR_INTERVAL_MICROS
);

// Instantiate the PID controllers
PID pitchPID(
  PITCH_P, PITCH_I, PITCH_D, PITCH_OUTPUT_LIMIT, PITCH_INTEGRAL_LIMIT,
  PITCH_INTERVAL_MICROS
);
PID rollPID(
  ROLL_P, ROLL_I, ROLL_D, ROLL_OUTPUT_LIMIT, ROLL_INTEGRAL_LIMIT,
  ROLL_INTERVAL_MICROS
);
PID yawPID(
  YAW_P, YAW_I, YAW_D, YAW_OUTPUT_LIMIT, YAW_INTEGRAL_LIMIT, YAW_INTERVAL_MICROS
);

// Instantiate the battery manager
BatteryManagement battery(BATTERY_CELL_1, BATTERY_CELL_2, BATTERY_CELL_3);

// Instantiate the flight controller
FlightController flightController(
  radio, IMU, uSensor, battery, pitchPID, rollPID, yawPID, motorMix,
  ERROR_LED_PIN, WARNING_LED_PIN
);

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

/**
 * @brief Called by system at the startup once.
 */
void setup() {
  // Debug initialization
  if (DEBUG_MAIN || DEBUG_IMU || DEBUG_RADIO || DEBUG_MOTOR || DEBUG_USENSOR
      || DEBUG_BATTERY) {
        Serial.begin(9600);
  }
  
  flightController.begin();
}

/**
 * @brief Called by system every time possible.
 */
void loop() {
  flightController.update();

  //motorFR.update();
}

/*
Missing features:
  - Some way to restart motors without replugging power to arduino.

  - Shutoff motors when detecting unrecoverable angle (indicate with red LED).
  - Indicate autoland (blue LED). Incorp ts in the flight controller.
    - Ignore controls.
    - Set target angles to 0.
    - Throttle down slowly until reaching ground (ultrasensor reading ~ 5cm).
    - Tween throttle to minimums, then shutoff motors.
*/