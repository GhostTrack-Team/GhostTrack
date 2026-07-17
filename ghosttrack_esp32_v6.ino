#include <WiFi.h>
#include <WebServer.h>
#include <SoftwareSerial.h>   // EspSoftwareSerial library — now used for Pi link
#include <s3km1110.h>         // Waveshare S3KM1110 radar library
#include <SPI.h>
#include <SD.h>

// =============================================================================
// GHOSTTRACK ESP32-S3 FIRMWARE — v4
// =============================================================================
// WHAT CHANGED FROM v3 (confirmed via standalone testing):
//   - Radars now use the proper s3km1110 library instead of naive "Range xxx"
//     text parsing. The S3KM1110 sends binary hex frames, not plain text —
//     the old parsing approach never actually worked correctly for either
//     radar; this was only caught during isolated dual-radar testing.
//   - RIGHT radar's data pin is OT1 (not OT2 — OT2 is a separate simple
//     presence-trigger pin, not real UART data). RX/TX pins also had to be
//     swapped from earlier versions: ESP32 RX=22, TX=21 (was backwards).
//   - RIGHT radar moved from SoftwareSerial to hardware Serial1. Testing
//     showed SoftwareSerial's bit-banged timing isn't reliable enough for
//     this binary protocol (garbled frames). Hardware UART fixed it.
//   - CONFLICT THIS CREATED: Serial1 was already used for the Pi 5 link.
//     ESP32 only has 3 hardware UARTs total (Serial=USB debug, Serial1,
//     Serial2), and Serial2 is already the LEFT radar — so Serial1 can't
//     serve both the right radar and the Pi. FIX: the Pi link moved to
//     SoftwareSerial instead (same physical GPIO 26/27 wires, no rewiring
//     needed) since its simple text protocol tolerates SoftwareSerial's
//     timing jitter far better than the radar's binary protocol did.
//     Pi baud also dropped to 9600 (from 115200) for reliability under
//     SoftwareSerial + WiFi AP + web server running concurrently — the
//     ghosttrack_camera_dual script on the Pi needs its ESP32_BAUD updated
//     to match before reconnecting.
//   - Added a 1000ms boot-stabilization delay before radar/UART init —
//     testing showed both radars could fail to initialize together on a
//     cold power-cycle without it (inrush current settling).
//   - LED pins/logic: UNCHANGED, confirmed working via led_test.ino.
//   - SD card: UNCHANGED, still not working — parked for later per your
//     testing (sd_card_test_default_pins.ino). Code left in place; if the
//     card isn't detected it just logs a warning and continues, no crash.
//   - Coin motor code: FIXED. RIGHT motor confirmed on GPIO 27 via
//     dual_coin_motor_test.ino (not 12 as originally drafted). This created
//     a conflict with PI_TX_PIN (also 27) — Pi TX moved to GPIO 23 to
//     resolve it. Motor PWM code also converted from the old channel-based
//     ledc API (ledcSetup/ledcAttachPin, removed in Arduino-ESP32 core v3.x)
//     to the new pin-based API (ledcAttach/ledcWrite), matching what your
//     installed core actually supports — the old API would not compile.
//   - FUSION LOGIC / ALERT CONCEPT: UNCHANGED. Still radar distance + Pi
//     vision + selected side → SAFE/AWARENESS/CRITICAL, same thresholds,
//     same dashboard, same Pi UART message format (ALERT:SIDE:ZONE / SAFE).
//     Only the low-level transport/wiring changed, not the decision logic.
//
// !! IMPORTANT PIN NOTE !!
// The default ESP32 hardware SPI pins (SCK=18, MISO=19) COLLIDE with the
// LEFT radar's UART pins. This version explicitly initializes SPI on
// different pins (see SD_SCK/SD_MISO/SD_MOSI/SD_CS below) to avoid that
// conflict. Double-check these against your specific ESP32-S3 board's
// pinout diagram before wiring — avoid strapping pins (0, 3, 45, 46) and
// any pins reserved for flash/PSRAM on your particular module variant.
// =============================================================================

const char* ssid     = "ghosttrack";
const char* password  = "leharin2008";

WebServer server(80);

// ── Coin vibration motor pins (PWM → BC547 transistor base → motor) ─────────
// CONFIRMED via dual_coin_motor_test.ino: RIGHT motor is on GPIO 27, not 12
// (12 is a strapping pin — moved off it during testing).
#define MOTOR_LEFT_PIN    13
#define MOTOR_RIGHT_PIN   27
#define MOTOR_PWM_FREQ    2000     // Hz, well above audible engine rumble band
#define MOTOR_PWM_RES     8        // 8-bit → 0-255 duty range

#define MOTOR_PWM_SAFE          0

// ── Intensity-based design: CRITICAL = heavy continuous buzz,
//    AWARENESS = light, informational tap-tap pattern ────────────────────────
// Motor confirmed: 3V ERM coin motor, supplied from a 3V rail (not 5V).
// ERM motors need roughly ~2.3V (typ.) just to start spinning at all (stall
// threshold). On a 3V rail, that means duty must be roughly 195+/255 (~76%+)
// to actually start the motor — anything lower and it just hums/stalls
// silently instead of tapping. "Heavy" is simply full power (255), which
// is the motor's own rated voltage, no overdrive concern at 3V rail.
#define MOTOR_PWM_CRITICAL_DUTY    255   // full power, continuous — heavy buzz (~3.0V)
#define MOTOR_PWM_AWARENESS_DUTY  205   // ~2.4V — just clears stall threshold
                                         // on a 3V rail, still visibly weaker
                                         // than CRITICAL when pulsed
#define AWARENESS_PULSE_ON_MS     150   // short tap
#define AWARENESS_PULSE_OFF_MS    450   // clear gap between taps — reads as
                                         // "information", not alarm

// ── LED pins (unchanged) ────────────────────────────────────────────────────
#define LEFT_YELLOW_LED   25
#define LEFT_RED_LED      32
#define RIGHT_YELLOW_LED  33
#define RIGHT_RED_LED      4

// ── Pi 5 UART — moved to SoftwareSerial (see version notes above). Same
//    physical GPIO 26 RX wire as before. TX moved from GPIO 27 → GPIO 23,
//    since 27 is now confirmed as the RIGHT coin motor's pin (was a direct
//    conflict). Baud dropped to 9600 for reliability under SoftwareSerial +
//    WiFi + web server load. IMPORTANT: update ESP32_BAUD to 9600 in the
//    Pi's camera script too, and rewire the Pi's connection to GPIO 23. ──
#define PI_RX_PIN   26
#define PI_TX_PIN   23
#define PI_BAUD     9600
SoftwareSerial piSerial(PI_RX_PIN, PI_TX_PIN);

// ── LEFT radar — hardware Serial2, confirmed working via dual_radar_test.
//    Module's real data pin is OT1 (OT2 is a separate presence-trigger
//    pin, not UART data — do not use it). ──────────────────────────────
#define RADAR_L_RX_PIN  18
#define RADAR_L_TX_PIN  19
#define RADAR_BAUD      115200

// ── RIGHT radar — hardware Serial1 (confirmed working; SoftwareSerial
//    could not reliably decode this radar's binary protocol). RX/TX fixed
//    to match confirmed wiring: ESP32 RX=22 ← radar OT1, ESP32 TX=21 →
//    radar RX. ──────────────────────────────────────────────────────────
#define RADAR_R_RX_PIN  22
#define RADAR_R_TX_PIN  21

s3km1110 radarLeft;
s3km1110 radarRight;

#define RADAR_TIMEOUT_MS  1500   // no valid frame in this long = treat as disconnected

// ── SD card module — SCK/MISO moved off GPIO 6/7. On classic ESP32 (your
//    confirmed board type, not S3), GPIO 6/7 are internally wired to the
//    flash chip and aren't safe to use for anything else — not just
//    "won't work," touching them can destabilize the whole board. Parked
//    for now per your testing, but fixed here in case you revisit it. ──
#define SD_SCK_PIN   14
#define SD_MISO_PIN  34
#define SD_MOSI_PIN  15
#define SD_CS_PIN    5
bool sdReady = false;

// (Old fixed CRITICAL_CM/NEAR_CM=50/100 removed — superseded by the
// speed-adaptive CRITICAL_CM_CITY/HIGHWAY + NEAR_CM_CITY/HIGHWAY below.)

// =============================================================================
// NEW (v6): SPEED-ADAPTIVE RADAR THRESHOLD + TURN INTENT GATING
// NO NEW HARDWARE — both speed and turn intent are set from the web
// dashboard (buttons/slider), not physical sensors. Nothing below uses a
// GPIO pin. All existing wiring/pins/baud remain completely untouched.
// =============================================================================

// ── Speed-adaptive distance threshold: 2m (city) / 6m (highway) ────────────
// Spec requires 2m/6m, replacing the old fixed 50cm/100cm. Ratio between
// CRITICAL/NEAR kept similar in spirit to the original (NEAR = wider buffer
// than CRITICAL), just scaled to the new distances.
#define CRITICAL_CM_CITY      100   // 1.0 m
#define NEAR_CM_CITY          200   // buffer zone above critical
#define CRITICAL_CM_HIGHWAY   150   // 1.5 m
#define NEAR_CM_HIGHWAY       280   // wider buffer at highway speed

// Speed thresholds for the web-set speed value.
#define SPEED_HIGHWAY_KMH     40    // at/above this speed → highway thresholds
#define SPEED_MIN_KMH         5     // below this → treat as slow/stopped
                                     // traffic, suppress alerts entirely
                                     // (spec: "avoids alerts in slow traffic")

// ── Turn intent — web-controlled, no blinker switch or steering sensor ─────
// "Only alerts when driver is actually about to turn into danger" — an
// alert on a given side only fires if the dashboard's turn-signal buttons
// indicate that side (or BOTH).


#define BLINK_INTERVAL  250
#define SENSOR_TIMEOUT  2000

String alertMode    = "SAFE";      // SAFE / AWARENESS / CRITICAL
String alertSide    = "NONE";      // LEFT / RIGHT / NONE
String selectedSide = "NONE";      // manual monitoring-side picker (dashboard)

long radarLeftCM  = 999;
long radarRightCM = 999;

String piSide   = "NONE";          // last side reported by Pi vision (LEFT/RIGHT/BOTH)
String piClass  = "none";

bool redState = false;

unsigned long lastBlink        = 0;
unsigned long lastRadarLTime   = 0;
unsigned long lastRadarRTime   = 0;
unsigned long lastPiTime       = 0;
unsigned long lastMotorToggle  = 0;
bool motorAwarenessOn          = false;

// ── v6: speed + turn intent state — both set from the web dashboard ────────
unsigned long lastSpeedCalcTime = 0;
#define SPEED_CALC_INTERVAL_MS 500
float vehicleSpeedKMH = 0.0;

int   currentCriticalCM = CRITICAL_CM_CITY;   // updated each loop from speed
int   currentNearCM     = NEAR_CM_CITY;
bool  highwayModeActive = false;

String turnIntentSideNow = "NONE";            // LEFT / RIGHT / BOTH / NONE

// =============================================================================
// LED HELPERS (unchanged logic, same pins)
// =============================================================================

void allLEDsOff() {
  digitalWrite(LEFT_YELLOW_LED, LOW);
  digitalWrite(LEFT_RED_LED, LOW);
  digitalWrite(RIGHT_YELLOW_LED, LOW);
  digitalWrite(RIGHT_RED_LED, LOW);
}

void updateDirectionalLEDs() {
  unsigned long now = millis();

  if (alertMode == "SAFE") {
    allLEDsOff();
    redState = false;
    return;
  }

  if (alertMode == "AWARENESS" && alertSide == "LEFT") {
    digitalWrite(LEFT_YELLOW_LED, HIGH);
    digitalWrite(RIGHT_YELLOW_LED, LOW);
    digitalWrite(LEFT_RED_LED, LOW);
    digitalWrite(RIGHT_RED_LED, LOW);
    redState = false;
    return;
  }

  if (alertMode == "AWARENESS" && alertSide == "RIGHT") {
    digitalWrite(RIGHT_YELLOW_LED, HIGH);
    digitalWrite(LEFT_YELLOW_LED, LOW);
    digitalWrite(LEFT_RED_LED, LOW);
    digitalWrite(RIGHT_RED_LED, LOW);
    redState = false;
    return;
  }

  if (alertMode == "CRITICAL" && alertSide == "LEFT") {
    digitalWrite(LEFT_YELLOW_LED, LOW);
    digitalWrite(RIGHT_YELLOW_LED, LOW);
    digitalWrite(RIGHT_RED_LED, LOW);

    if (now - lastBlink >= BLINK_INTERVAL) {
      redState = !redState;
      digitalWrite(LEFT_RED_LED, redState ? HIGH : LOW);
      lastBlink = now;
    }
    return;
  }

  if (alertMode == "CRITICAL" && alertSide == "RIGHT") {
    digitalWrite(LEFT_YELLOW_LED, LOW);
    digitalWrite(RIGHT_YELLOW_LED, LOW);
    digitalWrite(LEFT_RED_LED, LOW);

    if (now - lastBlink >= BLINK_INTERVAL) {
      redState = !redState;
      digitalWrite(RIGHT_RED_LED, redState ? HIGH : LOW);
      lastBlink = now;
    }
    return;
  }
}

// =============================================================================
// COIN MOTOR CONTROL (NEW — replaces servo oscillation)
// =============================================================================

void motorsOff() {
  ledcWrite(MOTOR_LEFT_PIN,  MOTOR_PWM_SAFE);
  ledcWrite(MOTOR_RIGHT_PIN, MOTOR_PWM_SAFE);
}

void updateMotors() {
  unsigned long now = millis();

  if (alertMode == "SAFE" || alertSide == "NONE") {
    motorsOff();
    motorAwarenessOn = false;
    return;
  }

  int activePin = (alertSide == "LEFT") ? MOTOR_LEFT_PIN : MOTOR_RIGHT_PIN;
  int idlePin   = (alertSide == "LEFT") ? MOTOR_RIGHT_PIN : MOTOR_LEFT_PIN;
  ledcWrite(idlePin, MOTOR_PWM_SAFE);

  if (alertMode == "CRITICAL") {
    // Heavy buzz — full power, continuous, no gaps. This is the strongest,
    // most sustained sensation the motor can give — deliberately impossible
    // to mistake for anything except "urgent."
    ledcWrite(activePin, MOTOR_PWM_CRITICAL_DUTY);
    return;
  }

  if (alertMode == "AWARENESS") {
    // Light, informational tap — partial power, clear gaps between pulses.
    // Reads as "FYI, something's there" rather than an alarm.
    unsigned long interval = motorAwarenessOn ? AWARENESS_PULSE_ON_MS : AWARENESS_PULSE_OFF_MS;
    if (now - lastMotorToggle >= interval) {
      motorAwarenessOn = !motorAwarenessOn;
      ledcWrite(activePin, motorAwarenessOn ? MOTOR_PWM_AWARENESS_DUTY : MOTOR_PWM_SAFE);
      lastMotorToggle = now;
    }
  }
}

// =============================================================================
// SD CARD GHOST LOG (NEW)
// =============================================================================

void initSDCard() {
  SPI.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);

  if (!SD.begin(SD_CS_PIN, SPI)) {
    Serial.println("[SD] ⚠️  Card not found — Ghost Log will rely on Pi 5 backup only");
    sdReady = false;
    return;
  }

  sdReady = true;
  Serial.println("[SD] ✅ Card mounted");

  if (!SD.exists("/ghost_log.csv")) {
    File f = SD.open("/ghost_log.csv", FILE_WRITE);
    if (f) {
      f.println("millis,side,class,radar_left_cm,radar_right_cm,alert_type");
      f.close();
    }
  }
}

void writeGhostLog() {
  if (!sdReady) return;
  if (alertMode == "SAFE") return;   // only log actual events, not idle ticks

  File f = SD.open("/ghost_log.csv", FILE_APPEND);
  if (!f) return;

  f.print(millis());        f.print(",");
  f.print(alertSide);       f.print(",");
  f.print(piClass);         f.print(",");
  f.print(radarLeftCM);     f.print(",");
  f.print(radarRightCM);    f.print(",");
  f.println(alertMode);

  f.close();
}

// =============================================================================
// DASHBOARD HTML
// =============================================================================

String getHTML() {
  auto radarLabel = [](long cm) -> String {
    if (cm >= 900) return "No object";
    if (cm <= currentCriticalCM) return "CRITICAL &mdash; " + String(cm) + " cm";
    if (cm <= currentNearCM)     return "NEAR &mdash; " + String(cm) + " cm";
    return "FAR &mdash; " + String(cm) + " cm";
  };
  auto radarColorFor = [](long cm) -> String {
    if (cm >= 900) return "#555";
    if (cm <= currentCriticalCM) return "#ff2d2d";
    if (cm <= currentNearCM)     return "#ffaa00";
    return "#00e676";
  };

  String radarLText = radarLabel(radarLeftCM);
  String radarRText = radarLabel(radarRightCM);
  String radarLColor = radarColorFor(radarLeftCM);
  String radarRColor = radarColorFor(radarRightCM);

  String visionText = "No VRU detected";
  String visionColor = "#555";
  if (piSide == "LEFT") {
    visionText = "VRU LEFT &mdash; " + piClass;
    visionColor = "#ffaa00";
  } else if (piSide == "RIGHT") {
    visionText = "VRU RIGHT &mdash; " + piClass;
    visionColor = "#ffaa00";
  } else if (piSide == "BOTH") {
    visionText = "VRU BOTH SIDES &mdash; " + piClass;
    visionColor = "#ff2d2d";
  }

  String alertText = "SAFE";
  String alertColor = "#00e676";
  String alertBg = "#0a1f0a";
  if (alertMode == "CRITICAL") {
    alertText = "&#9888; CRITICAL &mdash; " + alertSide;
    alertColor = "#ff2d2d";
    alertBg = "#1f0a0a";
  } else if (alertMode == "AWARENESS") {
    alertText = "&#9679; AWARENESS &mdash; " + alertSide;
    alertColor = "#ffaa00";
    alertBg = "#1a1400";
  }

  unsigned long now = millis();
  String radarLHealth = (now - lastRadarLTime < SENSOR_TIMEOUT) ? "ONLINE" : "OFFLINE";
  String radarRHealth = (now - lastRadarRTime < SENSOR_TIMEOUT) ? "ONLINE" : "OFFLINE";
  String piHealth     = (now - lastPiTime     < SENSOR_TIMEOUT) ? "ONLINE" : "OFFLINE";
  String sdHealth      = sdReady ? "ONLINE" : "OFFLINE";

  String radarLHColor = (radarLHealth == "ONLINE") ? "#00e676" : "#ff2d2d";
  String radarRHColor = (radarRHealth == "ONLINE") ? "#00e676" : "#ff2d2d";
  String piHColor      = (piHealth == "ONLINE") ? "#00e676" : "#ff2d2d";
  String sdHColor       = sdReady ? "#00e676" : "#ff2d2d";

  String leftBg  = (selectedSide == "LEFT")  ? "#00e676" : "#1a1a1a";
  String leftFg  = (selectedSide == "LEFT")  ? "#000" : "#888";
  String rightBg = (selectedSide == "RIGHT") ? "#00e676" : "#1a1a1a";
  String rightFg = (selectedSide == "RIGHT") ? "#000" : "#888";

  // v6: turn signal + speed mode button states (same active/inactive pattern)
  String turnLBg = (turnIntentSideNow == "LEFT"  || turnIntentSideNow == "BOTH") ? "#ffaa00" : "#1a1a1a";
  String turnLFg = (turnIntentSideNow == "LEFT"  || turnIntentSideNow == "BOTH") ? "#000" : "#888";
  String turnRBg = (turnIntentSideNow == "RIGHT" || turnIntentSideNow == "BOTH") ? "#ffaa00" : "#1a1a1a";
  String turnRFg = (turnIntentSideNow == "RIGHT" || turnIntentSideNow == "BOTH") ? "#000" : "#888";
  String speedModeText = highwayModeActive ? "HIGHWAY MODE (&ge;40 km/h)" : "CITY MODE (<40 km/h)";
  String speedModeColor = highwayModeActive ? "#00e676" : "#ffaa00";

  String html = R"raw(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta http-equiv="refresh" content="1">
<title>GhostTrack</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:Arial,sans-serif;background:#080808;color:#ccc;min-height:100vh;display:flex;flex-direction:column;align-items:center;padding:20px 16px 40px}
.logo{color:#00e676;font-size:28px;letter-spacing:6px;margin-bottom:2px;margin-top:8px;font-weight:bold}
.sub{color:#333;font-size:11px;letter-spacing:3px;margin-bottom:24px}
.alert-banner{width:100%;max-width:400px;border-radius:12px;padding:18px 20px;margin-bottom:16px;text-align:center;border:1px solid #222}
.alert-label{font-size:10px;letter-spacing:3px;color:#444;margin-bottom:6px}
.alert-val{font-size:22px;font-weight:700}
.card{width:100%;max-width:400px;background:#0f0f0f;border:1px solid #1a1a1a;border-radius:10px;padding:14px 18px;margin-bottom:12px}
.card-label{font-size:10px;letter-spacing:2px;color:#444;margin-bottom:5px}
.card-val{font-size:17px;font-weight:600}
.dual-row{display:grid;grid-template-columns:1fr 1fr;gap:10px;width:100%;max-width:400px;margin-bottom:12px}
.health-row{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;width:100%;max-width:400px;margin-bottom:16px}
.health-card{background:#0f0f0f;border:1px solid #1a1a1a;border-radius:10px;padding:10px;text-align:center}
.health-lbl{font-size:8px;letter-spacing:1px;color:#444;margin-bottom:4px}
.health-val{font-size:12px;font-weight:700}
.sec-label{width:100%;max-width:400px;font-size:10px;letter-spacing:2px;color:#333;margin-bottom:10px}
.side-row{display:grid;grid-template-columns:1fr 1fr;gap:10px;width:100%;max-width:400px;margin-bottom:12px}
.btn-side{padding:20px 10px;border:1px solid #222;border-radius:10px;font-size:16px;font-weight:700;cursor:pointer}
.btn-clear{width:100%;max-width:400px;padding:18px;border:1px solid #ff2d2d44;border-radius:10px;background:#130000;color:#ff2d2d;font-size:16px;font-weight:700;letter-spacing:2px;cursor:pointer}
.divider{width:100%;max-width:400px;border:none;border-top:1px solid #181818;margin:8px 0 16px}
.mono{font-family:monospace;font-size:13px}
</style>
</head>
<body>
<div class="logo">GHOSTTRACK</div>
<div class="sub">DUAL RADAR + DUAL VISION FUSION</div>
)raw";

  html += "<div class=\"alert-banner\" style=\"background:" + alertBg + ";border-color:" + alertColor + "44\">";
  html += "<div class=\"alert-label\">FUSED ALERT STATUS</div>";
  html += "<div class=\"alert-val\" style=\"color:" + alertColor + "\">" + alertText + "</div></div>";

  html += "<div class=\"dual-row\">";
  html += "<div class=\"card\"><div class=\"card-label\">RADAR &mdash; LEFT</div>";
  html += "<div class=\"card-val mono\" style=\"color:" + radarLColor + "\">" + radarLText + "</div></div>";
  html += "<div class=\"card\"><div class=\"card-label\">RADAR &mdash; RIGHT</div>";
  html += "<div class=\"card-val mono\" style=\"color:" + radarRColor + "\">" + radarRText + "</div></div>";
  html += "</div>";

  html += "<div class=\"card\"><div class=\"card-label\">VISION AI &mdash; Pi 5 (dual camera)</div>";
  html += "<div class=\"card-val\" style=\"color:" + visionColor + "\">" + visionText + "</div></div>";

  html += "<hr class=\"divider\">";

  html += "<div class=\"sec-label\">SENSOR HEALTH</div>";
  html += "<div class=\"health-row\">";
  html += "<div class=\"health-card\"><div class=\"health-lbl\">PI 5</div>";
  html += "<div class=\"health-val\" style=\"color:" + piHColor + "\">" + piHealth + "</div></div>";
  html += "<div class=\"health-card\"><div class=\"health-lbl\">RADAR L/R</div>";
  html += "<div class=\"health-val\" style=\"color:" + radarLHColor + "\">" + radarLHealth + "/" + radarRHealth + "</div></div>";
  html += "<div class=\"health-card\"><div class=\"health-lbl\">SD CARD</div>";
  html += "<div class=\"health-val\" style=\"color:" + sdHColor + "\">" + sdHealth + "</div></div>";
  html += "</div>";

  html += "<hr class=\"divider\">";

  html += "<div class=\"sec-label\">SELECT MONITORING SIDE</div>";
  html += "<div class=\"side-row\">";
  html += "<button class=\"btn-side\" style=\"background:" + leftBg + ";color:" + leftFg + "\" onclick=\"go('/side?s=LEFT')\">LEFT SIDE</button>";
  html += "<button class=\"btn-side\" style=\"background:" + rightBg + ";color:" + rightFg + "\" onclick=\"go('/side?s=RIGHT')\">RIGHT SIDE</button>";
  html += "</div>";
  html += "<button class=\"btn-clear\" onclick=\"go('/side?s=NONE')\">SAFE &mdash; CLEAR ALL</button>";

  html += "<hr class=\"divider\">";

  // v6: turn signal — replaces a physical blinker switch, no hardware needed
  html += "<div class=\"sec-label\">TURN SIGNAL (gates alerts + reassigns haptic motor to turn side)</div>";
  html += "<div class=\"side-row\">";
  html += "<button class=\"btn-side\" style=\"background:" + turnLBg + ";color:" + turnLFg + "\" onclick=\"go('/turn?side=LEFT')\">&#8592; LEFT SIGNAL</button>";
  html += "<button class=\"btn-side\" style=\"background:" + turnRBg + ";color:" + turnRFg + "\" onclick=\"go('/turn?side=RIGHT')\">RIGHT SIGNAL &#8594;</button>";
  html += "</div>";
  html += "<button class=\"btn-clear\" onclick=\"go('/turn?side=NONE')\">SIGNAL OFF</button>";

  html += "<hr class=\"divider\">";

  // v6: speed — replaces a physical wheel-speed sensor, no hardware needed
  html += "<div class=\"sec-label\">VEHICLE SPEED (simulates speed sensor)</div>";
  html += "<div class=\"card\" style=\"width:100%;max-width:400px;margin-bottom:10px\">";
  html += "<div class=\"card-val mono\" style=\"color:" + speedModeColor + "\">" + String(vehicleSpeedKMH, 0) + " km/h &mdash; " + speedModeText + "</div>";
  html += "<input type=\"range\" min=\"0\" max=\"100\" value=\"" + String((int)vehicleSpeedKMH) + "\" style=\"width:100%;margin-top:10px\" "
          "onchange=\"go('/speed?kmh='+this.value)\">";
  html += "</div>";
  html += "<div class=\"side-row\">";
  html += "<button class=\"btn-side\" style=\"background:#1a1a1a;color:#888\" onclick=\"go('/speed?kmh=0')\">STOPPED</button>";
  html += "<button class=\"btn-side\" style=\"background:#1a1a1a;color:#888\" onclick=\"go('/speed?kmh=20')\">CITY (20)</button>";
  html += "<button class=\"btn-side\" style=\"background:#1a1a1a;color:#888\" onclick=\"go('/speed?kmh=60')\">HIGHWAY (60)</button>";
  html += "</div>";

  html += R"raw(
<script>
function go(url){
  fetch(url).then(()=>setTimeout(()=>location.reload(),200)).catch(()=>{});
}
</script>
</body>
</html>
)raw";

  return html;
}

void handleRoot() {
  server.send(200, "text/html", getHTML());
}

void handleSide() {
  if (server.hasArg("s")) {
    selectedSide = server.arg("s");
    // NONE is now AUTO mode (both sides monitored live) — not an "off"
    // state, so we no longer force everything to SAFE here. LEDs/motors
    // will simply follow whatever updateAlertLogic() determines next loop.
    Serial.print("[Web] Side: ");
    Serial.println(selectedSide == "NONE" ? "AUTO (both sides)" : selectedSide);
  }
  server.send(200, "text/plain", "OK");
}

// v6: turn signal — dashboard buttons for LEFT / RIGHT / NONE / BOTH,
// replacing a physical blinker switch + steering angle sensor entirely.
void handleTurn() {
  if (server.hasArg("side")) {
    turnIntentSideNow = server.arg("side");
    Serial.print("[Web] Turn signal: ");
    Serial.println(turnIntentSideNow);
  }
  server.send(200, "text/plain", "OK");
}

// v6: speed — dashboard slider/number input in km/h, replacing a physical
// wheel-speed pulse sensor entirely. Recomputes city/highway thresholds
// immediately (no need to wait for a polling interval, unlike a sensor).
void handleSpeed() {
  if (server.hasArg("kmh")) {
    vehicleSpeedKMH = server.arg("kmh").toFloat();
    applySpeedThresholds();
    Serial.print("[Web] Speed: ");
    Serial.print(vehicleSpeedKMH);
    Serial.println(highwayModeActive ? " km/h (HIGHWAY mode)" : " km/h (CITY mode)");
  }
  server.send(200, "text/plain", "OK");
}

// =============================================================================
// UART READERS
// =============================================================================

// Pi 5 protocol is UNCHANGED — "ALERT:SIDE:ZONE" or "SAFE" from the dual-camera
// script. Only the transport changed (SoftwareSerial instead of hardware
// Serial1); the message format and meaning are identical.
void readPiDetection() {
  if (!piSerial.available()) return;

  String data = piSerial.readStringUntil('\n');
  data.trim();
  if (data.length() == 0) return;

  lastPiTime = millis();
  Serial.print("[Pi5] ");
  Serial.println(data);

  if (data == "SAFE") {
    piSide = "NONE";
    piClass = "none";
    return;
  }

  if (data.startsWith("ALERT:")) {
    int p1 = data.indexOf(':');
    int p2 = data.indexOf(':', p1 + 1);
    if (p1 == -1 || p2 == -1) return;
    piSide  = data.substring(p1 + 1, p2);
    piClass = data.substring(p2 + 1);   // ZONE (CRITICAL/NEAR) — shown as "class" slot
    return;
  }
}

void readRadarLeft() {
  if (radarLeft.read()) {
    lastRadarLTime = millis();
    radarLeftCM = radarLeft.isTargetDetected ? radarLeft.distanceToTarget : 999;
    Serial.print("[Radar-L] ");
    if (radarLeft.isTargetDetected) {
      Serial.print(radarLeftCM);
      Serial.println(" cm");
    } else {
      Serial.println("No object");
    }
    return;
  }
  // No new frame this cycle — leave radarLeftCM as-is. The dashboard's
  // SENSOR_TIMEOUT health check already flags OFFLINE if this persists,
  // no need to duplicate that logic here.
}

void readRadarRight() {
  if (radarRight.read()) {
    lastRadarRTime = millis();
    radarRightCM = radarRight.isTargetDetected ? radarRight.distanceToTarget : 999;
    Serial.print("[Radar-R] ");
    if (radarRight.isTargetDetected) {
      Serial.print(radarRightCM);
      Serial.println(" cm");
    } else {
      Serial.println("No object");
    }
    return;
  }
}

// =============================================================================
// SPEED — now WEB-CONTROLLED, no physical sensor needed.
// Dashboard sends /speed?kmh=XX, we just store it directly. Thresholds
// still switch automatically based on this value (spec: 2m city / 6m
// highway), and slow-traffic suppression still works the same way.
// =============================================================================

void applySpeedThresholds() {
  highwayModeActive = (vehicleSpeedKMH >= SPEED_HIGHWAY_KMH);
  currentCriticalCM = highwayModeActive ? CRITICAL_CM_HIGHWAY : CRITICAL_CM_CITY;
  currentNearCM     = highwayModeActive ? NEAR_CM_HIGHWAY     : NEAR_CM_CITY;
}

// =============================================================================
// TURN INTENT — now WEB-CONTROLLED, no physical switch/sensor needed.
// Dashboard sends /turn?side=LEFT / RIGHT / NONE, we just store it directly.
// =============================================================================
// (turnIntentSideNow is set directly by handleTurn() below — nothing to
// poll here each loop, unlike the old sensor-based version.)

// =============================================================================
// FUSION LOGIC
// =============================================================================
//
// selectedSide == "NONE" (default, no button pressed) → AUTO mode: both
//   radars + vision are monitored simultaneously, and the system reacts to
//   whichever side actually triggers first. This is the always-on behavior.
// selectedSide == "LEFT"/"RIGHT" → manual override: only that one side is
//   monitored, exactly as before. The dashboard button stays for this case
//   (e.g. testing one side in isolation, or focusing on a known blind spot).
// =============================================================================

// Evaluate a single side's zone (SAFE/AWARENESS/CRITICAL) from ONLY its
// radar + vision inputs — no turn-intent gating, no speed suppression.
// Used as the raw input to AUTO mode's spatial reassignment logic below,
// since that logic needs to see BOTH sides' real danger level before
// deciding which motor should actually fire.
String evaluateZoneRaw(String side) {
  long radarCM   = (side == "LEFT") ? radarLeftCM : radarRightCM;
  bool visionVRU = (piSide == side || piSide == "BOTH");
  bool radarCritical = (radarCM <= currentCriticalCM);
  bool radarNear     = (radarCM <= currentNearCM);

  if (radarCritical && visionVRU)  return "CRITICAL";
  if (radarCritical && !visionVRU) return "AWARENESS";
  if (radarNear)                   return "AWARENESS";
  if (visionVRU)                   return "AWARENESS";
  return "SAFE";
}

// Same as evaluateZoneRaw, but with speed suppression + turn-intent gate
// applied. Used by MANUAL override mode (selectedSide == LEFT/RIGHT), where
// there's only one side to evaluate and no reassignment is meaningful.
String evaluateZone(String side) {
  if (vehicleSpeedKMH < SPEED_MIN_KMH) return "SAFE";
  bool turnIntentMatches = (turnIntentSideNow == side || turnIntentSideNow == "BOTH");
  if (!turnIntentMatches) return "SAFE";
  return evaluateZoneRaw(side);
}

int zoneRank(String zone) {
  if (zone == "CRITICAL")  return 2;
  if (zone == "AWARENESS") return 1;
  return 0;
}

void updateAlertLogic() {
  // ── Manual override: only the selected side is monitored ──────────────
  if (selectedSide == "LEFT" || selectedSide == "RIGHT") {
    String zone = evaluateZone(selectedSide);
    alertMode = zone;
    alertSide = (zone == "SAFE") ? "NONE" : selectedSide;
    return;
  }

  // ── AUTO mode (selectedSide == "NONE", the default) ─────────────────────
  // Slow/stopped traffic — suppress everything, same as before.
  if (vehicleSpeedKMH < SPEED_MIN_KMH) {
    alertMode = "SAFE";
    alertSide = "NONE";
    return;
  }

  // Evaluate BOTH sides' real danger level, ungated — we need to see
  // everything before deciding which motor should actually fire.
  String leftZone  = evaluateZoneRaw("LEFT");
  String rightZone = evaluateZoneRaw("RIGHT");
  int leftRank  = zoneRank(leftZone);
  int rightRank = zoneRank(rightZone);

  if (leftRank == 0 && rightRank == 0) {
    alertMode = "SAFE";
    alertSide = "NONE";
    return;
  }

  // Worst detected zone/side, before any turn-direction reassignment —
  // this is "what actually happened", independent of which way the wheel
  // is turned.
  String rawZone, rawSide;
  if (leftRank >= rightRank) { rawZone = leftZone;  rawSide = "LEFT";  }
  else                       { rawZone = rightZone; rawSide = "RIGHT"; }
  if (leftRank == rightRank && radarRightCM < radarLeftCM) {
    // Tie-break by closer radar distance, same as before
    rawZone = rightZone; rawSide = "RIGHT";
  }

  // ── Turn-intent gate + dynamic spatial reassignment ─────────────────────
  // No turn signaled at all → strict gate, per spec: no alert fires
  // regardless of what the sensors see.
  if (turnIntentSideNow == "NONE") {
    alertMode = "SAFE";
    alertSide = "NONE";
    return;
  }

  if (turnIntentSideNow == "BOTH") {
    // Both directions "active" (e.g. hazards/wide turn) — normal 1:1
    // mapping, no reassignment needed.
    alertMode = rawZone;
    alertSide = rawSide;
    return;
  }

  // turnIntentSideNow is LEFT or RIGHT — DYNAMIC SPATIAL REASSIGNMENT:
  // the haptic motor that fires is the one matching the turn direction,
  // regardless of which physical radar/camera actually detected the
  // danger. This mirrors the real "wide turn" hazard — as the vehicle
  // rotates into a turn, the side it's swinging toward is the side that
  // matters, even if the sensor that first saw the object was mounted on
  // the other side of the vehicle.
  alertMode = rawZone;             // severity = worst of either side
  alertSide = turnIntentSideNow;   // motor = the side you're turning toward
}

// =============================================================================
// SETUP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(LEFT_YELLOW_LED, OUTPUT);
  pinMode(LEFT_RED_LED, OUTPUT);
  pinMode(RIGHT_YELLOW_LED, OUTPUT);
  pinMode(RIGHT_RED_LED, OUTPUT);
  allLEDsOff();

  // v6: turn intent + speed are set from the web dashboard — no sensor
  // pins to initialize here.
  Serial.println("[OK] Turn intent     → web dashboard (/turn?side=)");
  Serial.println("[OK] Speed           → web dashboard (/speed?kmh=)");

  // Coin motor PWM — new pin-based ledc API (Arduino-ESP32 core v3.x), same
  // as confirmed working in dual_coin_motor_test.ino.
  ledcAttach(MOTOR_LEFT_PIN,  MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  ledcAttach(MOTOR_RIGHT_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  motorsOff();

  // Boot-stabilization delay — confirmed necessary via testing. Without
  // this, both radars could fail to initialize together on a cold power
  // cycle (inrush current settling on the shared 3.3V rail).
  delay(1000);

  piSerial.begin(PI_BAUD);
  Serial.println("[OK] Pi5 UART        → SoftwareSerial RX=D26 TX=D23 @9600");

  Serial2.begin(RADAR_BAUD, SERIAL_8N1, RADAR_L_RX_PIN, RADAR_L_TX_PIN);
  radarLeft.begin(Serial2, Serial);
  Serial.println("[OK] Radar LEFT UART → Serial2 RX=D18 TX=D19 (s3km1110)");

  Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_R_RX_PIN, RADAR_R_TX_PIN);
  radarRight.begin(Serial1, Serial);
  Serial.println("[OK] Radar RIGHT UART→ Serial1 RX=D22 TX=D21 (s3km1110)");

  initSDCard();

  for (int i = 0; i < 3; i++) {
    digitalWrite(LEFT_YELLOW_LED, HIGH);
    digitalWrite(LEFT_RED_LED, HIGH);
    digitalWrite(RIGHT_YELLOW_LED, HIGH);
    digitalWrite(RIGHT_RED_LED, HIGH);
    delay(150);
    allLEDsOff();
    delay(150);
  }

  WiFi.softAP(ssid, password);
  IPAddress ip = WiFi.softAPIP();

  Serial.println("\n=========================================");
  Serial.println("GHOSTTRACK v4.0 — DUAL RADAR / DUAL CAM");
  Serial.println("=========================================");
  Serial.print("SSID      : "); Serial.println(ssid);
  Serial.print("Dashboard : http://"); Serial.println(ip);
  Serial.println("-----------------------------------------");
  Serial.println("LEFT  motor → D13 (PWM)   RIGHT motor → D27 (PWM)");
  Serial.println("=========================================\n");

  server.on("/", handleRoot);
  server.on("/side", handleSide);
  server.on("/turn", handleTurn);
  server.on("/speed", handleSpeed);
  server.begin();
}

// =============================================================================
// MAIN LOOP
// =============================================================================

unsigned long lastLogWrite = 0;
#define LOG_INTERVAL 1000  // avoid hammering the SD card every loop tick

void loop() {
  server.handleClient();

  readPiDetection();
  readRadarLeft();
  readRadarRight();

  // v6: speed + turn intent come from the web dashboard (set instantly by
  // handleSpeed()/handleTurn() below) — nothing to poll here each loop.

  updateAlertLogic();

  updateDirectionalLEDs();
  updateMotors();

  unsigned long now = millis();
  if (now - lastLogWrite >= LOG_INTERVAL) {
    writeGhostLog();
    lastLogWrite = now;
  }
}
