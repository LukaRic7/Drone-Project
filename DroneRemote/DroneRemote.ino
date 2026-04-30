/**
 * @file    DroneRemote.ino
 * @author  Jaqueline Liljedahl & Benjamin Benfeldt
 * @brief   Remote control for the drone.
 * @date    2026-04-30
 * * @details This file handles sending signals to the drone, telling it where
 * to relocate to.
 */

// ========================================================================== //
// INCLUDE LIBRARIES                                                          //
// ========================================================================== //

// RadioHead Amplitude Shifting Keying, 433MHz frequenzy
#include <RH_ASK.h>

// Serial Peripheral Interface library
#include <SPI.h>

// ========================================================================== //
// CONFIGURATION                                                              //
// ========================================================================== //

// Debug logging, ensure only one toggled at once to avoid console clutter.
constexpr bool DEBUG_MAIN       = true; // Main information out
constexpr bool DEBUG_RADIO      = false;

// ========================================================================== //
// PIN DEFINITIONS                                                            //
// ========================================================================== //

// Physical remote control pins
constexpr uint8_t AUTOLAND_BUTTON  = 7;
constexpr uint8_t JOYSTICK_LEFT_X  = A0;
constexpr uint8_t JOYSTICK_LEFT_Y  = A1;
constexpr uint8_t JOYSTICK_RIGHT_X = A2;
constexpr uint8_t JOYSTICK_RIGHT_Y = A3;

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
      bool success = driver.init();
      if (!success && DEBUG_RADIO)
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
        }
      }

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

      // Check if the radio lost signal to the transmitter
      if (millis() - lastRxTime > 2000) {
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

// ========================================================================== //
// CLASS INSTANTIATION                                                        //
// ========================================================================== //

Radio radio;
RemotePacket packet;

// ========================================================================== //
// LIFECYCLE FUNCTIONS                                                        //
// ========================================================================== //

/**
 * @brief Called by system at the startup once.
 */
void setup() {
  pinMode(AUTOLAND_BUTTON, INPUT_PULLUP);
  
  if (DEBUG_MAIN || DEBUG_RADIO)
    Serial.begin(9600);

  radio.begin();
}

/**
 * @brief Called by system every time possible.
 */
void loop() {
  // Læs joystick
  int joy1_x = analogRead(A0); // Yaw
  int joy1_y = analogRead(A1); // Throttle
  int joy2_x = analogRead(A2); // Roll
  int joy2_y = analogRead(A3); // Pitch

  // Konverter fra 0-1023 til 0-31
  packet.yaw      = map(joy1_x, 0, 1023, 0, 31);
  packet.throttle = map(joy1_y, 0, 1023, 0, 31);
  packet.roll     = map(joy2_x, 0, 1023, 0, 31);
  packet.pitch    = map(joy2_y, 0, 1023, 0, 31);

  packet.autoland = digitalRead(7);

  // Log if needed
  if (DEBUG_MAIN) {
    Serial.print(F("Pitch: "));
    Serial.print(packet.pitch);
    Serial.print(F(" | Roll: "));
    Serial.print(packet.roll);
    Serial.print(F(" | Throttle: "));
    Serial.print(packet.throttle);
    Serial.print(F(" | Yaw: "));
    Serial.println(packet.yaw);
    Serial.print(F(" | Autoland: "));
    Serial.println(packet.autoland ? "ON" : "OFF");
  }

  radio.send(packet);
}