# MyFriend ESP-NOW Mesh Network Specification

**Version:** 0.2  
**Purpose:** Distributed ESP32 wireless control network using ESP-NOW mesh communication.

---

## 1. System Overview

The MyFriend system consists of:

- **One Head device**
- **Multiple Remote devices**
- A local Wi-Fi network
- ThingSpeak for cloud-based configuration/control
- ESP-NOW for all communication between the Head and Remote devices

The **Head device is the only device that connects to Wi-Fi or the Internet**.

Remote devices:

- Do **not** connect to Wi-Fi.
- Do **not** access ThingSpeak.
- Do **not** require Internet access.
- Communicate exclusively using ESP-NOW.
- May relay ESP-NOW messages for other remote devices.

The system is designed so that remote devices can communicate even when they cannot directly reach the Head.

---

# 2. Network Architecture

```text
                         INTERNET
                            │
                            │
                       ┌────▼────┐
                       │ThingSpeak│
                       │Channel   │
                       │ 2060365  │
                       └────┬────┘
                            │
                         Wi-Fi
                            │
                     ┌──────▼──────┐
                     │     HEAD    │
                     │             │
                     │ Wi-Fi       │
                     │ ThingSpeak  │
                     │ ESP-NOW     │
                     └──────┬──────┘
                            │
                         ESP-NOW
                            │
                    ┌───────▼───────┐
                    │    Remote 1   │
                    │  ESP-NOW only │
                    └───────┬───────┘
                            │
                       ESP-NOW
                       /         \
              ┌────────▼───┐   ┌──▼────────┐
              │  Remote 2  │   │ Remote 3  │
              │ ESP-NOW    │   │ ESP-NOW   │
              └──────┬─────┘   └───────────┘
                     │
                  ESP-NOW
                     │
              ┌──────▼─────┐
              │  Remote 4  │
              │ ESP-NOW    │
              └────────────┘
```

The network is effectively a **multi-hop ESP-NOW mesh**.

A message may travel through several Remote devices before reaching its destination.

---

# 3. Head Device

The Head is the gateway between the Internet and the ESP-NOW network.

The Head performs the following functions:

1. Connects to the local Wi-Fi network.
2. Determines the Wi-Fi channel currently being used.
3. Initializes ESP-NOW on that channel.
4. Monitors ThingSpeak channel `2060365`.
5. Determines whether new ThingSpeak data is available.
6. Reads the ThingSpeak fields when new data is detected.
7. Converts the ThingSpeak data into a mesh message.
8. Broadcasts the update into the ESP-NOW network.
9. Sends directed commands to individual Remote devices.
10. Receives status responses from Remote devices.
11. Manages the network session and startup process.

The Head remains powered continuously.

---

# 4. Remote Devices

Remote devices communicate **only through ESP-NOW**.

They do not:

- Connect to the local Wi-Fi network
- Obtain an IP address
- Access the Internet
- Access ThingSpeak
- Require Wi-Fi credentials

Their communication path is:

```text
Remote → ESP-NOW → Remote → ESP-NOW → ... → Head
```

or:

```text
Head → ESP-NOW → Remote → ESP-NOW → ... → Remote
```

A Remote may act as both:

- An endpoint that executes commands
- A repeater that forwards messages

This allows the network to extend beyond the direct radio range of the Head.

---

# 5. Wi-Fi and ESP-NOW Channel

The Head's ESP-NOW channel must match the channel used by its Wi-Fi connection.

Therefore:

```text
Head Wi-Fi Channel
       ↓
ESP-NOW Channel
       ↓
Entire MyFriend mesh
```

The Remote devices do not connect to the Wi-Fi network, but they must operate their ESP-NOW radios on the same 2.4 GHz channel as the Head.

The Head should determine its current Wi-Fi channel rather than assuming a fixed channel.

If the Wi-Fi router changes channels, the Head must reinitialize the ESP-NOW network on the new channel.

---

# 6. Remote Startup and Channel Discovery

When the Head receives a command requiring the Remote devices to operate, it turns on power to the Remote devices.

All Remote devices may therefore boot at approximately the same time.

At startup, a Remote does not initially know which Wi-Fi channel the MyFriend network is using.

The Remote enters:

```text
NETWORK_CHANNEL_DISCOVERY
```

and scans the supported 2.4 GHz channels.

The Head periodically transmits a:

```text
MSG_JOIN_BEACON
```

The first Remote that hears a valid MyFriend beacon determines the channel from the channel on which the beacon was received.

The Remote then:

1. Locks onto that channel.
2. Joins the MyFriend network.
3. Begins transmitting JOIN_BEACON messages itself.
4. Allows additional Remote devices to discover the network.

This allows channel information to propagate through the physical network.

---

# 7. Join Beacon Propagation

Example:

```text
             HEAD
              │
          JOIN BEACON
              │
              ▼
          Remote 1
              │
       JOIN BEACON
              │
              ▼
          Remote 2
              │
       JOIN BEACON
              │
              ▼
          Remote 3
```

Remote 3 does not need to be within direct radio range of the Head.

It only needs to be within range of another Remote that has already joined.

JOIN_BEACON messages must contain enough information for a device to determine that the beacon belongs to the MyFriend network.

The receiving device already knows the radio channel on which the beacon was received.

---

# 8. Network Startup Sequence

The intended startup sequence is:

```text
1. Head is powered
        ↓
2. Head connects to Wi-Fi
        ↓
3. Head determines Wi-Fi channel
        ↓
4. Head initializes ESP-NOW
        ↓
5. Head waits for/receives external command
        ↓
6. Head begins JOIN_BEACON transmission
        ↓
7. Head powers Remote devices
        ↓
8. Remotes boot
        ↓
9. Remotes scan channels
        ↓
10. Remote hears JOIN_BEACON
        ↓
11. Remote locks onto channel
        ↓
12. Remote begins beacon propagation
        ↓
13. Additional Remotes join
        ↓
14. Neighbor discovery begins
        ↓
15. Network becomes READY
```

---

# 9. Network States

The network uses the following logical states:

```cpp
NETWORK_OFF
NETWORK_STARTING
NETWORK_CHANNEL_DISCOVERY
NETWORK_DISCOVERY
NETWORK_READY
NETWORK_STOPPING
```

### NETWORK_OFF

Remote devices are powered down or inactive.

### NETWORK_STARTING

Devices are booting and initializing hardware.

### NETWORK_CHANNEL_DISCOVERY

Devices are scanning for a valid MyFriend JOIN_BEACON.

### NETWORK_DISCOVERY

Devices have found the channel and are determining which neighboring devices are reachable.

### NETWORK_READY

Normal mesh operation is permitted.

### NETWORK_STOPPING

The Head is shutting down the network and preparing to remove power from Remote devices.

---

# 10. ThingSpeak Integration

The Head is the **only device that communicates with ThingSpeak**.

### ThingSpeak Channel

```text
Channel ID: 2060365
```

The Head periodically checks the age of the most recent ThingSpeak entry.

Conceptually:

```text
Check ThingSpeak
       ↓
Is data newer than last processed entry?
       │
   ┌───┴───┐
   │       │
  NO      YES
   │       │
   │       ▼
   │   Read fields 1-8
   │       │
   │       ▼
   │   Create mesh message
   │       │
   │       ▼
   │   Broadcast via ESP-NOW
   │
   └───> Wait for next check
```

The Head should track the last ThingSpeak update it has processed so that the same update is not repeatedly transmitted unnecessarily.

---

# 11. ThingSpeak Data Fields

ThingSpeak fields 1 through 8 contain:

| Field | Name |
|---|---|
| 1 | Brightness |
| 2 | Color1 |
| 3 | Color2 |
| 4 | Color3 |
| 5 | Pattern |
| 6 | Time On |
| 7 | SleepTime |
| 8 | FXSpeed |

Recommended internal representation:

```cpp
struct ThingSpeakData {
    uint16_t brightness;
    uint16_t color1;
    uint16_t color2;
    uint16_t color3;
    uint16_t pattern;
    uint16_t timeOn;
    uint16_t sleepTime;
    uint16_t fxSpeed;
};
```

The exact data types can be changed if individual ThingSpeak fields require values outside the `uint16_t` range.

---

# 12. ThingSpeak Update Message

A new mesh message type is defined:

```text
MSG_THINGSPEAK_UPDATE
```

The Head creates this message whenever it detects new ThingSpeak data.

The message contains:

- Session ID
- Message ID
- Source device ID
- Target device ID
- Message type
- TTL
- ThingSpeak data

Example:

```cpp
struct ThingSpeakData {
    uint16_t brightness;
    uint16_t color1;
    uint16_t color2;
    uint16_t color3;
    uint16_t pattern;
    uint16_t timeOn;
    uint16_t sleepTime;
    uint16_t fxSpeed;
};
```

The complete network packet will contain the common mesh header plus this payload.

---

# 13. ThingSpeak Data Flow

```text
             ThingSpeak
                  │
                  │ New entry
                  ▼
               HEAD
                  │
          Read fields 1-8
                  │
                  ▼
        MSG_THINGSPEAK_UPDATE
                  │
                  │ ESP-NOW
                  ▼
              Remote 1
             /       \
            /         \
      Remote 2       Remote 3
          │
          ▼
      Remote 4
```

Every Remote that receives the update may:

1. Validate the message.
2. Check whether it has already processed the message.
3. Save/apply the new ThingSpeak values.
4. Forward the message to neighboring devices.

---

# 14. Mesh Message Structure

The common mesh message should contain:

```cpp
struct MeshMessage {
    uint32_t session_id;
    uint32_t message_id;

    uint8_t source_id;
    uint8_t target_id;

    uint8_t type;
    uint8_t ttl;

    uint8_t command;

    // Payload follows
};
```

The exact implementation may use a union or separate packet structures for different message types.

---

# 15. Message Identity

Every message is identified by:

```text
SESSION_ID
SOURCE_ID
MESSAGE_ID
```

For example:

```text
Session: 872341
Source: 0
Message: 27
```

A Remote maintains a short history of recently received messages.

If it receives the same message again, it does not execute or forward the message again.

This prevents duplicate processing caused by multiple paths through the mesh.

---

# 16. Time-To-Live (TTL)

Every forwarded message contains a TTL value.

Example:

```text
Initial TTL = 10
```

Each forwarding device decrements the TTL:

```text
10 → 9 → 8 → 7 → ...
```

When TTL reaches zero, the message is not forwarded.

This prevents messages from circulating indefinitely.

---

# 17. Broadcast Messages

A broadcast message is intended for all Remote devices.

A reserved target ID is used:

```text
TARGET_ALL = 255
```

ThingSpeak updates will normally be broadcast messages because all Remote devices may need the new configuration.

---

# 18. Directed Messages

A directed message contains a specific target device ID.

Example:

```text
Source: HEAD
Target: Remote 4
Type: MSG_STATUS_REQUEST
```

The intermediate devices forward the message until Remote 4 receives it.

Remote 4 executes the request and sends a response back through the mesh.

---

# 19. Status Requests

The Head may request status information from a specific Remote device.

Example:

```text
HEAD
  │
  │ STATUS_REQUEST
  ▼
Remote 1
  │
  │ forward
  ▼
Remote 2
  │
  │ forward
  ▼
Remote 4
```

Remote 4 responds:

```text
Remote 4
    │
    │ STATUS_RESPONSE
    ▼
Remote 2
    │
    ▼
Remote 1
    │
    ▼
HEAD
```

Potential status information includes:

- Battery voltage
- Operating state
- Error flags
- Active outputs
- Sensor status
- Firmware version
- Network information

---

# 20. Message Types

Initial message types:

```text
MSG_JOIN_BEACON
MSG_DISCOVER
MSG_TEST
MSG_STATUS_REQUEST
MSG_STATUS_RESPONSE
MSG_THINGSPEAK_UPDATE
```

Additional message types can be added later for:

```text
MSG_COMMAND
MSG_ACK
MSG_ROUTE_REQUEST
MSG_ROUTE_RESPONSE
MSG_NETWORK_SHUTDOWN
```

---

# 21. Neighbor Discovery

Once a Remote has found the MyFriend channel, it participates in neighbor discovery.

A Remote identifies other MyFriend devices that are directly reachable.

Neighbor information may eventually include:

- Device ID
- MAC address
- RSSI
- Link quality
- Last-seen time

Initially, a flooding architecture can be used without maintaining sophisticated routing tables.

---

# 22. Initial Routing Strategy

The initial implementation will use **controlled flooding**.

When a new message is received:

```text
1. Validate packet
2. Check message ID
3. If already seen:
       Ignore
4. If new:
       Record message ID
5. Process if applicable
6. Decrement TTL
7. Forward to neighbors
```

This approach is simple and robust for the relatively small number of devices expected in the MyFriend network.

More sophisticated routing can be added later.

---

# 23. Duplicate Suppression

Duplicate suppression is particularly important because flooding can cause the same message to arrive through multiple paths.

Example:

```text
             Head
            /    \
           /      \
      Remote 1   Remote 2
           \      /
            \    /
            Remote 3
```

Remote 3 could receive the same message through both Remote 1 and Remote 2.

Remote 3 must process the message only once.

The combination of:

```text
SESSION_ID
SOURCE_ID
MESSAGE_ID
```

provides the unique message identity.

---

# 24. Test Message

A test message can be generated by the Head:

```text
MSG_TEST
```

Example:

```text
TEST #27
```

Each Remote receiving a new test message:

1. Prints the message to Serial.
2. Flashes its LED twice.
3. Records the message as seen.
4. Forwards the message.

If the same test arrives again through another path, the Remote does not flash or print it again.

This provides a simple physical test of mesh propagation.

---

# 25. Power Control

The Head controls power to the Remote devices.

The intended sequence is:

```text
Head detects need for network
        ↓
Head starts JOIN_BEACON
        ↓
Head powers Remote devices
        ↓
Remotes boot
        ↓
Remotes join mesh
        ↓
Network becomes READY
```

When network operation is complete:

```text
Head sends shutdown message
        ↓
Remotes shut down gracefully
        ↓
Head removes power
```

---

# 26. ThingSpeak and Remote Independence

The network is deliberately designed so that Remote operation does not depend on Internet connectivity.

Only this device requires Internet access:

```text
             HEAD
              │
           Wi-Fi
              │
         ThingSpeak
```

The rest of the system is:

```text
Remote ↔ Remote ↔ Remote ↔ Remote
       ESP-NOW only
```

If the Internet connection is unavailable:

- The Head cannot obtain new ThingSpeak updates.
- The ESP-NOW mesh can still operate.
- Previously received configuration can remain active on the Remote devices.
- Local/head-originated commands can still be distributed through ESP-NOW.

---

# 27. Security

Security can be added after the basic mesh is operational.

Potential future features:

- ESP-NOW encryption
- Network authentication
- Encrypted payloads
- Network/session keys
- Message authentication codes
- Replay protection

The message ID/session architecture provides a foundation for replay protection.

---

# 28. Reliability

The initial implementation prioritizes simplicity and range.

Future reliability features may include:

- ACK messages
- Retries
- RSSI monitoring
- Link-quality measurements
- Route selection
- Automatic route repair
- Message expiration
- Delivery confirmation to the Head

---

# 29. Future Routing

The first implementation should use controlled flooding.

Once the network is stable, routing can be optimized.

Possible future architecture:

```text
               HEAD
              /    \
           Remote  Remote
              \      /
               Remote
                 │
               Remote
```

Devices could eventually maintain routing information such as:

```text
Destination | Next Hop | Hops | RSSI
------------|----------|------|-----
Remote 2    | Remote 1 |  1   | -55
Remote 3    | Remote 1 |  1   | -67
Remote 4    | Remote 2 |  2   | -71
```

This would reduce unnecessary transmissions compared with flooding.

---

# 30. Development Phases

## Phase 1 — Basic ESP-NOW

Already demonstrated:

- Head and Remote communication
- Multiple devices
- Multi-hop message propagation
- LED indication
- Serial diagnostics

## Phase 2 — Network Startup

Implement:

- Head Wi-Fi connection
- Wi-Fi channel detection
- JOIN_BEACON
- Remote channel scanning
- Channel locking
- Beacon propagation

## Phase 3 — Network Discovery

Implement:

- Device IDs
- Neighbor discovery
- MAC discovery
- ESP-NOW peer registration

## Phase 4 — Mesh Messaging

Implement:

- Common message header
- Session IDs
- Message IDs
- TTL
- Duplicate suppression
- Controlled flooding

## Phase 5 — ThingSpeak Integration

Implement:

- Head monitoring of channel `2060365`
- Detection of new data
- Reading fields 1–8
- `ThingSpeakData` structure
- `MSG_THINGSPEAK_UPDATE`
- Mesh distribution of ThingSpeak updates

## Phase 6 — Device Control

Implement:

- Directed commands
- Status requests
- Status responses
- Remote device state management

## Phase 7 — Reliability

Implement:

- ACKs
- Retries
- Delivery confirmation
- Link monitoring

## Phase 8 — Routing Optimization

Implement:

- Routing tables
- Route discovery
- Better path selection
- Automatic route repair

---

# 31. Initial Success Criteria

The initial network implementation will be considered successful when:

1. The Head connects to the local Wi-Fi network.
2. The Head determines its Wi-Fi channel.
3. Remote devices boot without connecting to Wi-Fi.
4. Remote devices scan for the MyFriend JOIN_BEACON.
5. Remote devices discover the correct ESP-NOW channel.
6. Joined devices propagate the JOIN_BEACON.
7. Devices beyond direct Head range can join through another Remote.
8. Neighbor discovery completes.
9. A Head-generated test message can travel through multiple hops.
10. Duplicate messages are suppressed.
11. A directed status request can reach a specific Remote.
12. The Remote can return a status response to the Head.
13. The Head detects a new ThingSpeak entry.
14. The Head reads ThingSpeak fields 1–8.
15. The Head broadcasts the ThingSpeak update through the mesh.
16. All appropriate Remote devices receive and process the update.
17. Remote devices never require a Wi-Fi or Internet connection.

---

# 32. Core Design Principle

The fundamental architecture is:

```text
                    INTERNET
                       │
                       │ Wi-Fi
                       ▼
                    ┌─────┐
                    │HEAD │
                    └──┬──┘
                       │
                    ESP-NOW
                       │
              ┌────────┴────────┐
              │                 │
           Remote             Remote
              │                 │
           ESP-NOW            ESP-NOW
              │                 │
           Remote ─────────── Remote
```

**The Head is the only Internet/Wi-Fi device.**

**The Remote devices form an ESP-NOW-only mesh.**

**ThingSpeak data enters the system through the Head and is distributed through the mesh.**

This separation keeps the Remote devices simple, reduces power and software requirements, and allows the ESP-NOW network to operate independently of Internet connectivity once configuration has been received.