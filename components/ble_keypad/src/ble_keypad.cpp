#if BLE_KEYPAD

  #define __BLE_KEYPAD__ 1
  #include "ble_keypad.hpp"

  #include "config.hpp"

  #include "esp_timer.h"
  #include "esp_log.h"
  #include "nimble/nimble_port.h"
  #include "nimble/nimble_port_freertos.h"
  #include "host/ble_uuid.h"

  #if TRACING_BLE_KEYPAD
    #include <iostream>
  #endif

  BLEKeypad *BLEKeypad::instance = nullptr;

  auto BLEKeypad::hexToInt(const char c) const -> uint8_t {
    if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
    if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
    if (c >= '0' && c <= '9') { return c - '0'; }
    return -1;
  }

  auto BLEKeypad::parseMacAddr(const std::string& macStr, uint8_t mac[6]) -> bool {
    // A standard MAC address (xx:xx:xx:xx:xx:xx) must be exactly 17 characters long
    if (macStr.length() != 17) {
      return false;
    }

    for (int i = 0; i < 6; ++i) {
      int strIdx = i * 3;

      // Extract high and low nibbles for the current byte
      int highNibble = hexToInt(macStr[strIdx]);
      int lowNibble = hexToInt(macStr[strIdx + 1]);

      // Fail if either character is an invalid hex digit
      if (highNibble == -1 || lowNibble == -1) {
        return false;
      }

      // Validate the delimiter between bytes (except after the final byte)
      if (i < 5 && macStr[strIdx + 2] != ':') {
        return false;
      }

      // Combine nibbles into a single byte using bitwise shifting
      mac[i] = static_cast<uint8_t>((highNibble << 4) | lowNibble);
    }

    return true;
  }

  // The processJ06ProPacket is taylored to the specific packet structures emitted by the TikTok Remote
  // Control (named "J06 Pro"), which is one of the two target device for this BLE keypad integration.
  // It decodes the raw HID report data into actionable events while managing internal state to handle
  // the unique burst patterns of the device's input stream.
  //
  // Here are all the packets received for all available keys:
  //
  // Right Key:
  // Packet: 0f bc 72 12
  // Packet: 0f 8a 12 13
  // Packet: 0f 58 72 12
  // Packet: 0f 26 12 13
  // Packet: 0f c2 11 13
  // Packet: 0f 90 71 12
  // Packet: 0f 5e 11 13
  // Packet: 0f c8 70 12
  // Packet: 08 00 10 13
  //
  // Left Key:
  // Packet: 0f c8 70 12
  // Packet: 0f fa 10 13
  // Packet: 0f 2c 71 12
  // Packet: 0f 5e 11 13
  // Packet: 0f 90 71 12
  // Packet: 0f c2 11 13
  // Packet: 0f f4 71 12
  // Packet: 0f 26 12 13
  // Packet: 0f bc 72 12
  // Packet: 08 e7 13 13
  //
  // Up Key:
  // Packet: 10 00 00
  // Packet: 00 00 00
  //
  // Down key:
  // Packet: 40 00 00
  // Packet: 00 00 00
  //
  // Select key:
  // Packet: 0f f4 41 1f
  // Packet: 08 f4 41 1f
  //
  // Home key:
  // Packet: 0f 03 02 32
  // Packet: 08 03 02 32
  //
  // The remote control can be purchased through AliExpress:
  //
  // https://www.aliexpress.com/item/1005011855666831.html
  //

  void BLEKeypad::processJ06ProPacket(const uint8_t* data, size_t length) {
    static bool    is4byteButtonHeld = false;
    static uint8_t last3byteState = 0x00;

    if (data == nullptr) {
      return;
    }

    #if TRACING_BLE_KEYPAD
      std::cout << "Packet: ";
      for (size_t i = 0; i < length; i++) {
        std::cout << std::format("{:02x} ", data[i]);
      }
      std::cout << std::endl;
    #endif

    Event event{ EventKind::NONE };

    // ==========================================
    // CASE 1: 3-BYTE PACKET HANDLING (Up / Down)
    // ==========================================
    if (length == 3) {
      uint8_t currentAction = data[0];

      if (currentAction == 0x00) {
        last3byteState = 0x00;     // Reset state on release
        return;
      }

      // Prevent continuous hold repetitions from hitting the display loop
      if (currentAction == last3byteState) {
        return;
      }

      last3byteState = currentAction;

      if (currentAction == 0x10) { event.kind = EventKind::DBL_PREV; }
      else if (currentAction == 0x40) { event.kind = EventKind::DBL_NEXT; }
    }

    // ====================================================================
    // CASE 2: 4-BYTE PACKET HANDLING (Right, Left, Select, Home)
    // ====================================================================
    else if (length == 4) {
      uint8_t stateByte = data[0];

      // --- Handle Release Edge ---
      if (stateByte == 0x08) {
        is4byteButtonHeld = false;

        // Differentiate Left vs Right strictly by parsing the unique release footprint
        if (data[1] == 0x00) {
          event.kind = EventKind::NEXT;       // Right Key released
        } else if (data[1] == 0xE7) {
          event.kind = EventKind::PREV;       // Left Key released
        }
      }
      // --- Handle Initial Press Edge ---
      else if (stateByte == 0x0F) {
        if (is4byteButtonHeld) {
          return;       // Safely ignore the continuous rolling code stream (Bytes 1-3)
        }
        is4byteButtonHeld = true;

        // Select and Home have unique, static layout identifiers in their payloads
        if (data[2] == 0x41 && data[3] == 0x1F) {
          event.kind = EventKind::SELECT;       // Select Key pressed
        } else if (data[2] == 0x02 && data[3] == 0x32) {
          event.kind = EventKind::DBL_SELECT;           // Home Key pressed
        }
        // Note: Left and Right are skipped here because we process them on the clean
        // Release Edge (0x08) where their data streams finally differentiate.
      }
    }

    if (event.kind != EventKind::NONE) {
      if (bleEventQueue) {
        xQueueSend(bleEventQueue, &event, 0);
      } else {
        LOG_E("Event bleEventQueue not initialized. Unable to send event.");
      }
    }
  }

  // The processBeautyR1Packet is taylored to the specific packet structures emitted by the TikTok Remote
  // Control (named "BeautyR1"), which is one of the two target device for this BLE keypad integration.
  // It decodes the raw HID report data into actionable events while managing internal state to handle
  // the unique burst patterns of the device's input stream.
  //
  // Here are all the packets received for all available keys:
  //
  // Right Key:
  // Packet: 07 50 05 50 06
  // Packet: 07 28 05 50 06
  // Packet: 07 60 04 50 06
  // Packet: 07 98 03 50 06
  // Packet: 07 d0 02 50 06
  // Packet: 07 08 02 50 06
  // Packet: 07 40 01 50 06
  // Packet: 07 78 00 50 06
  // Packet: 00 78 00 50 06
  //
  // Left Key:
  // Packet: 07 3a 01 50 06
  // Packet: 07 40 01 50 06
  // Packet: 07 08 02 50 06
  // Packet: 07 d0 02 50 06
  // Packet: 07 98 03 50 06
  // Packet: 07 60 04 50 06
  // Packet: 07 28 05 50 06
  // Packet: 07 50 05 50 06
  // Packet: 07 18 06 50 06
  // Packet: 07 e0 06 50 06
  // Packet: 00 e0 06 50 06
  //
  // Up Key:
  // Packet: 07 f0 04 f0 02
  // Packet: 07 f0 04 1c 04
  // Packet: 07 f0 04 48 05
  // Packet: 07 f0 04 74 06
  // Packet: 07 f0 04 a0 07
  // Packet: 07 f0 04 cc 08
  // Packet: 07 f0 04 f8 09
  // Packet: 62 f0 04 24 0b
  // Packet: 00 d0 f9 24 0b
  //
  // Down key:
  // Packet: 62 f0 04 24 0b
  // Packet: 07 f0 04 f8 09
  // Packet: 07 f0 04 cc 08
  // Packet: 07 f0 04 a0 07
  // Packet: 07 f0 04 74 06
  // Packet: 07 f0 04 48 05
  // Packet: 07 f0 04 1c 04
  // Packet: 07 f0 04 f0 02
  // Packet: 00 d0 f9 24 0b
  //
  // Select key:
  // Packet: 07 f0 04 74 06
  // Packet: 04 f4 03 cc 06
  //
  // Home key:
  // Packet: 06 00 00 00 00
  // Packet: 07 00 00 00 00
  // Packet: 02 00
  // Packet: 00 00
  // Packet: 06 00 00 00 00
  //
  // The remote control can be purchased through AliExpress:
  //
  // https://www.aliexpress.com/item/1005007944515439.html
  //

  #define DO while (1)
  #define END_DO break

  auto BLEKeypad::processBeautyR1Packet(uint8_t *data, size_t length) -> void {
    if (length == 0 || data == nullptr) { return; }

    auto reset_state = [&]() {
                         pendingVerticalIntent = VerticalIntent::INTENT_NONE;
                         isVerticalPressed     = false;
                       };

    Event event = { EventKind::NONE };

    #if TRACING_BLE_KEYPAD
      std::cout << "Packet: ";
      for (size_t i = 0; i < length; i++) {
        std::cout << std::format("{:02x} ", data[i]);
      }
      std::cout << std::endl;
    #endif

    DO {
      if (length == 2) {
        uint8_t reportId   = data[0];
        uint8_t clickValue = data[1];

        if ((reportId == 1 || reportId == 2) && clickValue == 0) {
          event.kind = EventKind::DBL_SELECT;
          reset_state();
          break;
        }
        break;
      }

      if (length < 4) { break; }
      uint8_t reportId = data[0];

      if (reportId == 7) {
        uint8_t marker = data[3];

        if (marker == 0xF0) {
          if (pendingVerticalIntent == VerticalIntent::INTENT_NONE) {
            pendingVerticalIntent = VerticalIntent::INTENT_UP;
          }
          break;
        }
        if (marker == 0xF8) {
          if (pendingVerticalIntent == VerticalIntent::INTENT_NONE) {
            pendingVerticalIntent = VerticalIntent::INTENT_DOWN;
          }
          break;
        }
        break;
      }

      if (reportId == 0) {
        int8_t deltaX = (int8_t)(data[1]);
        int8_t deltaY = (int8_t)(data[2]);

        if (deltaX == 0 && deltaY == 0) {
          break;
        }

        if (deltaX == (int8_t)0xE0 && deltaY == 6) {
          event.kind = EventKind::PREV;
          reset_state();
          break;
        }

        if (deltaX == 0x78 && deltaY == 0) {
          event.kind = EventKind::NEXT;
          reset_state();
          break;
        }

        if (deltaX == (int8_t)0xD0 && deltaY == (int8_t)0xF9) {
          if (!isVerticalPressed) {
            isVerticalPressed = true;
            if (pendingVerticalIntent == VerticalIntent::INTENT_UP) {
              event.kind = EventKind::DBL_PREV;
            } else if (pendingVerticalIntent == VerticalIntent::INTENT_DOWN) {
              event.kind = EventKind::DBL_NEXT;
            }
            reset_state();
          }
          pendingVerticalIntent = VerticalIntent::INTENT_NONE;
          isVerticalPressed = false;
          break;
        }
        break;
      }

      if (reportId == 4) {
        uint8_t targetByte = data[1];
        if (targetByte == 0xF4) {
          event.kind = EventKind::SELECT;
        }
      }

      END_DO;
    }

    #if TRACING_BLE_KEYPAD
      LOG_I("Vertical Pressed State: {}", isVerticalPressed ? "Yes" : "No");
      LOG_I("-------------------------------");
    #endif

    if (event.kind != EventKind::NONE) {
      if (bleEventQueue) {
        xQueueSend(bleEventQueue, &event, 0);
      } else {
        LOG_E("Event bleEventQueue not initialized. Unable to send event.");
      }
    }
  }

  // --- NIMBLE SCAN ENGINE MANAGER ---
  auto BLEKeypad::startScanning() -> void {
    if (isConnecting) { return; }

    struct ble_gap_disc_params discParams;
    memset(&discParams, 0, sizeof(discParams));

    discParams.filter_policy     = BLE_HCI_SCAN_FILT_NO_WL;
    discParams.passive           = 0; // Active scanning captures scan responses
    discParams.itvl              = 0x0050;
    discParams.window            = 0x0030;

    // FIX 1: Turn on duplicate filtering to stop catch every packet burst
    discParams.filter_duplicates = 1;

    // FIX 2: Change duration argument from 0 to BLE_HS_FOREVER for continuous scanning
    int rc = ble_gap_disc(BLE_OWN_ADDR_PUBLIC, BLE_HS_FOREVER, &discParams,
                          BLEKeypad::gapEventStub, nullptr);
    if (rc != 0) {
      LOG_E("Error initiating NimBLE discovery scan; rc={}", rc);
    } else {
      LOG_W("=== BLE SCANNER RUNNING ===");
    }
  }

  // The processMuzhtenPacket is tailored to the mini keypad advertising itself as "MUZHTEN". It is
  // sold under the Beauty-R1 name but emits a completely different set of HID reports, on two
  // streams: a 4 bytes one for the arrow and select keys, a 2 bytes one for the home key. Every
  // key press produces a burst of reports ending with a release report, recognized by a null first
  // byte, that is unique to the key and stable from one press to the next. Only the release
  // reports are decoded.
  //
  // Here are all the packets received for all available keys:
  //
  // Right Key:
  // Packet: 02 52 61 18
  // Packet: 02 52 01 19
  // Packet: 03 52 61 18
  // Packet: 03 36 61 18
  // Packet: 03 c6 60 18
  // Packet: 03 32 00 19
  // Packet: 02 32 60 18
  // Packet: 00 32 60 18
  //
  // Left Key:
  // Packet: 02 96 60 18
  // Packet: 03 96 00 19
  // Packet: 03 c6 60 18
  // Packet: 03 36 61 18
  // Packet: 03 c2 01 19
  // Packet: 02 c2 61 18
  // Packet: 00 c2 61 18
  //
  // Up Key:
  // Packet: 03 18 01 0d
  // Packet: 03 18 41 12
  // Packet: 03 18 c1 1c
  // Packet: 03 18 41 27
  // Packet: 03 22 c1 31
  // Packet: 00 18 01 37
  // Packet: 00 22 01 37
  //
  // Down key:
  // Packet: 03 2c 01 30
  // Packet: 03 18 e1 2a
  // Packet: 03 2c a1 20
  // Packet: 03 18 61 16
  // Packet: 03 2c 21 0c
  // Packet: 00 18 01 07
  // Packet: 00 18 01 07
  //
  // Select key:
  // Packet: 03 2c 81 1c
  // Packet: 00 2c 81 1c
  //
  // Home key, on the 2 bytes stream:
  // Packet: 01 00      (or 02 00)
  // Packet: 00 00
  //
  // Home key, on the 4 bytes stream (left undecoded):
  // Packet: 03 52 51 32
  // Packet: 02 52 51 32
  // Packet: 00 b5 51 32
  //
  // The keypad can be purchased through AliExpress:
  //
  // https://www.aliexpress.com/item/1005007944515439.html
  //

  auto BLEKeypad::processMuzhtenPacket(const uint8_t *data, size_t length) -> void {
    if (data == nullptr) { return; }

    Event event{ EventKind::NONE };

    if (length == 2) {
      // Home key. The trailing 00 00 packet is ignored.
      if ((data[0] == 1 || data[0] == 2) && data[1] == 0) {
        event.kind = EventKind::DBL_SELECT;
      }
    } else if ((length == 4) && (data[0] == 0x00)) {
      switch ((data[1] << 16) | (data[2] << 8) | data[3]) {
      case 0x326018: event.kind = EventKind::NEXT;       break;   // Right key
      case 0xC26118: event.kind = EventKind::PREV;       break;   // Left key
      case 0x180137:
      case 0x220137: event.kind = EventKind::DBL_PREV;   break;   // Up key
      case 0x180107: event.kind = EventKind::DBL_NEXT;   break;   // Down key
      case 0x2C811C: event.kind = EventKind::SELECT;     break;   // Select key
      default:                                                    // Home key release and the
        break;                                                    // intermediate reports
      }
    }

    if (event.kind == EventKind::NONE) { return; }

    // The up and down keys send their release report twice, up to 250 msecs apart. The other keys
    // send a single one and are left untouched to keep page turns responsive.
    if ((event.kind == EventKind::DBL_NEXT) || (event.kind == EventKind::DBL_PREV)) {
      static EventKind lastKind{ EventKind::NONE };
      static int64_t   lastTime{ 0 };

      int64_t now = esp_timer_get_time();

      if ((event.kind == lastKind) && ((now - lastTime) < 400000)) { return; }

      lastKind = event.kind;
      lastTime = now;
    }

    if (bleEventQueue) {
      xQueueSend(bleEventQueue, &event, 0);
    } else {
      LOG_E("Event bleEventQueue not initialized. Unable to send event.");
    }
  }

  // --- HID CHARACTERISTIC DISCOVERY LAUNCHER ---
  // Runs on connection and again once the link is encrypted, as some keypads only expose
  // their HID service to a bonded peer
  auto BLEKeypad::discoverHidChars() -> int {
    if (discoveryActive || hidFound) { return 0; }

    ble_uuid16_t hidUuid = {
      .u = { .type = BLE_UUID_TYPE_16 },
      .value = HID_REPORT_CHAR_UUID
    };

    discoveryActive = true;

    int rc = ble_gattc_disc_chrs_by_uuid(glConnId, 1, 0xffff, &hidUuid.u,
                                         BLEKeypad::discoveryStub, nullptr);
    if (rc != 0) {
      discoveryActive = false;
      LOG_E("GATT query initialization failure; rc={}", rc);
    }

    return rc;
  }

  // --- NIMBLE CENTRAL GAP EVENT LOOP ---
  auto BLEKeypad::handleGapEvent(struct ble_gap_event *event) -> int {
    int rc;

    switch (event->type) {

    case BLE_GAP_EVENT_DISC: {
      #if TRACING_BLE_KEYPAD
        if (event->disc.length_data > 0) {
          LOG_I("BLE Device Info: {:<{}}", (char *)(event->disc.data), event->disc.length_data);
        }
      #endif

      bool match = true;

      for (int i = 0; i < 6; i++) {
        if (event->disc.addr.val[5 - i] != targetMacAddress[i]) {
          match = false;
          break;
        }
      }

      if (!match) {
        std::string_view data{ reinterpret_cast<const char *>(event->disc.data),
                               event->disc.length_data };

        if (data.contains("Beauty-R1")) {
          match = true;
          keypadType = KeypadType::BEAUTY_R1;
          LOG_I("Found Beauty-R1!");
        } else if (data.contains("MUZHTEN")) {
          // Sold as a Beauty-R1 but advertising the manufacturer name, with its own reports
          match = true;
          keypadType = KeypadType::MUZHTEN;
          LOG_I("Found MUZHTEN keypad!");
        } else if (data.contains("J06 Pro")) {
          match = true;
          keypadType = KeypadType::J06_PRO;
          LOG_I("Found J06 Pro!");
        } else {
          match = false;
        }

        if (match) {

          const uint8_t * d = event->disc.addr.val;
          HimemString     macAddr = std::format("{:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
                                                d[5], d[4], d[3], d[2], d[1], d[0]).c_str();
          LOG_I("MAC Address: {}", macAddr);
          config.put(Config::Ident::BT_KEYPAD_MAC,  macAddr);
          config.put(Config::Ident::BT_KEYPAD_TYPE, static_cast<int8_t>(keypadType));
          config.save();
        }
      }

      if (match && !isConnecting) {
        isConnecting = true;
        LOG_W("TARGET MAC MATCHED! Terminating scan and establishing connection...");

        ble_gap_disc_cancel();

        // FIX 1: Provide explicit connection configuration parameters
        struct ble_gap_conn_params connParams;
        memset(&connParams, 0, sizeof(connParams));
        connParams.scan_itvl = 0x0010;
        connParams.scan_window = 0x0010;
        connParams.itvl_min = 24;            // 30ms interval minimum
        connParams.itvl_max = 40;            // 50ms interval maximum
        connParams.latency = 0;
        connParams.supervision_timeout = 512;   // Give it 5.12 seconds buffer to prevent dropouts

        // FIX 2: Pass &connParams instead of nullptr as the 4th argument
        rc = ble_gap_connect(BLE_OWN_ADDR_PUBLIC, &event->disc.addr, 30000,
                             &connParams, BLEKeypad::gapEventStub, nullptr);
        if (rc != 0) {
          LOG_E("Failed to initiate device pairing connection; rc={}", rc);
          isConnecting = false;
          startScanning();
        }
      }
      return 0;
    }


    // Replaces legacy Bluedroid: ESP_GATTC_CONNECT_EVT & ESP_GATTC_OPEN_EVT
    case BLE_GAP_EVENT_CONNECT: {
      if (event->connect.status == 0) {
        LOG_D("GATT Channel Active! Pulling internal service maps...");
        glConnId = event->connect.conn_handle;
        isConnecting = false;

        // Some keypads hide their HID service until the peer is bonded. Discovery is redone
        // in BLE_GAP_EVENT_ENC_CHANGE once encryption is up.
        rc = ble_gap_security_initiate(glConnId);
        if (rc != 0) {
          LOG_W("Unable to initiate BLE security; rc={}. Keeping the link unencrypted.", rc);
        }

        // Ground-level discovery pass passing discoveryStub to register the endpoints
        rc = discoverHidChars();
        if (rc == 0) {
          paired = true;

          Event event = { EventKind::PAIRING_ON };
          if (bleEventQueue) {
            xQueueSend(bleEventQueue, &event, 0);
          } else {
            LOG_E("Event bleEventQueue not initialized. Unable to send event.");
          }
        }
      } else {
        LOG_E("Failed to map GATT channel, error status: {}", event->connect.status);
        isConnecting = false;
        startScanning();
      }
      return 0;
    }

    // Replaces legacy Bluedroid: ESP_GATTC_DISCONNECT_EVT
    case BLE_GAP_EVENT_DISCONNECT: {
      LOG_W("BLE Device disconnected. Re-opening scanning state machine...");
      glConnId = BLE_HS_CONN_HANDLE_NONE;
      isConnecting = false;
      paired = false;
      hidFound = false;
      discoveryActive = false;

      Event event = { EventKind::PAIRING_OFF };
      if (bleEventQueue) {
        xQueueSend(bleEventQueue, &event, 0);
      } else {
        LOG_E("Event bleEventQueue not initialized. Unable to send event.");
      }

      startScanning();
      return 0;
    }

    case BLE_GAP_EVENT_NOTIFY_RX: {
      // Only process notifications coming from our connected remote device
      if (event->notify_rx.conn_handle == glConnId) {

        #if TRACING_BLE_KEYPAD
          LOG_D("Notification received on attribute handle {}", event->notify_rx.attr_handle);
        #endif

        // Forward the raw memory payload straight into your static C++ bridging stub
        BLEKeypad::onNotifyStub(event->notify_rx.conn_handle,
                                event->notify_rx.attr_handle,
                                event->notify_rx.om,
                                nullptr);
      }
      return 0;
    }

    case BLE_GAP_EVENT_ENC_CHANGE: {
      if (event->enc_change.status == 0) {
        LOG_D("Security Encryption established successfully! Device is now secured & bonded.");

        // A keypad gating its HID service behind encryption only reveals it now.
        discoverHidChars();
      } else {
        // Not fatal: keypads exposing their HID service on a plain link keep working.
        LOG_W("Security encryption negotiation failed; status={}", event->enc_change.status);
      }
      return 0;
    }

    default:
      break;
    }
    return 0;
  }

  // --- GATT CHARACTERISTIC PARSER LOOP ---
  auto BLEKeypad::handleDiscovery(const struct ble_gatt_chr *chr) -> void {
    if (chr->properties & BLE_GATT_CHR_PROP_NOTIFY) {
      hidFound = true;
      LOG_W("Found valid HID Notification Handle at: {}. Activating stream...", chr->val_handle);

      uint16_t cccdHandle = chr->val_handle + 1;
      uint8_t  value[] = { 0x01, 0x00 };

      LOG_D("Writing CCCD descriptor to subscribe to notifications...");

      // FIX: Use subscriptionStub here to handle the write confirmation event safely
      int rc = ble_gattc_write_flat(glConnId, cccdHandle, value, sizeof(value),
                                    BLEKeypad::subscriptionStub, nullptr);
      if (rc != 0) {
        LOG_E("Error trying to update peripheral notification permissions; rc={}", rc);
      }
    }
  }

  // --- NOTIFICATION REGISTRATION TRACKER ---
  auto BLEKeypad::handleSubscription(int status, uint16_t attrHandle) -> void {
    if (status == 0) {
      LOG_D("CCCD descriptor written successfully. Handle {} subscribed!", attrHandle);

      // REMOVED: The recursive ble_gattc_disc_chrs_by_uuid call.
      // The stream is already live. Doing nothing here breaks the infinite query loop!
    } else {
      LOG_E("Descriptor subscription mapping error returned; status={}", status);
    }
  }

  // --- HARDWARE STACK INITIALIZER ---
  auto BLEKeypad::setup(QueueHandle_t eventQueue) -> bool {
    instance      = this;
    bleEventQueue = eventQueue;

    // Retrieved the BLE mac address saved in the config file
    HimemString macAddr;
    int8_t      kType = 0;

    config.get(Config::Ident::BT_KEYPAD_MAC,  macAddr);
    config.get(Config::Ident::BT_KEYPAD_TYPE, &kType);
    keypadType = static_cast<KeypadType>(kType);

    if (macAddr.length() == 17) {
      if (parseMacAddr(macAddr.c_str(), targetMacAddress)) {
        LOG_I("BLE Keypad MAC address: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
              targetMacAddress[0],
              targetMacAddress[1],
              targetMacAddress[2],
              targetMacAddress[3],
              targetMacAddress[4],
              targetMacAddress[5]);
      }
    }

    // 2. Initialize NimBLE platform controller driver configurations
    if (nimble_port_init() != ESP_OK) {
      return false;
    }

    // 3. Security parameters context mapping (Aligns with original pairing parameters)
    ble_hs_cfg.reset_cb   = blecentOnReset;
    ble_hs_cfg.sync_cb    = blecentOnSync;
    ble_hs_cfg.sm_io_cap  = BLE_SM_IO_CAP_NO_IO; // Replaces legacy ESP_IO_CAP_NONE
    ble_hs_cfg.sm_bonding = 1;                   // Replaces legacy ESP_LE_AUTH_BOND

    // 4. Create the background tracking thread daemon task execution context
    xTaskCreate(blecentHostTask, "blecentHostTask", 4096, nullptr, 5, nullptr);

    return true;
  }

  // --- GLOBAL SYSTEM CONTEXT LINKAGE WRAPPERS ---
  void BLEKeypad::blecentOnReset(int reason) {
    ESP_LOGE("BLEKeypad", "Resetting NimBLE stack host context; reason={}", reason);
  }

  void BLEKeypad::blecentOnSync(void) {
    ESP_LOGI("BLEKeypad", "GATT Client registered. NimBLE host and controller sync achieved.");
    if (instance) {
      instance->startScanning(); // Perfectly accessible now!
    }
  }

  void BLEKeypad::blecentHostTask(void *param) {
    ESP_LOGI("BLEKeypad", "NimBLE core thread tracking initiated.");
    nimble_port_run();
    nimble_port_freertos_deinit();
  }

#endif
