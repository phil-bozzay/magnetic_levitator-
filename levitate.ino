#include "hardware/timer.h"

// ================= USER-ADJUSTABLE SETPOINT =================
volatile float Vref = 0.60f;

// ================= PINS =================
const int pwmPin    = 15;
const int pwmInvPin = 14;

// ================= CONTROL TIMING =================
const float Ts = 0.001f;   // 1 kHz

// ================= CONTROLLER PARAMETERS =================
volatile float G0 = 5.0f;
volatile float wz = 40.0f;     // rad/s
volatile float wp = 120.0f;    // rad/s

// ================= OUTPUT SCALING =================
volatile float du_scale = 50.0f;
volatile float u_bias   = 500.0f;

// ================= CONTROLLER STATE =================
volatile float e_k   = 0.0f;
volatile float e_k1  = 0.0f;
volatile float u_k   = 0.0f;

static float du_k  = 0.0f;
static float du_k1 = 0.0f;

// ================= COEFFICIENTS =================
volatile float b0, b1, a1;

// ================= SHARED SIGNAL =================
volatile float v_hall = 0.0f;

// ================= TIMER =================
repeating_timer_t control_timer;

// ============================================================
//   COEFFICIENT UPDATE
// ============================================================
void updateCoefficients() {
  float az = 2.0f / (wz * Ts);
  float ap = 2.0f / (wp * Ts);

  b0 = G0 * (1.0f + az) / (1.0f + ap);
  b1 = G0 * (1.0f - az) / (1.0f + ap);
  a1 = (1.0f - ap) / (1.0f + ap);
}

// ============================================================
//   1 kHz CONTROL LOOP
// ============================================================
bool controlISR(repeating_timer_t *t) {

  // --- ADC oversampling ---
  int sum = 0;
  for (int i = 0; i < 8; i++) {
    sum += analogRead(A0);
  }
  int adc = sum >> 3;

  v_hall = 2.0f * adc * (3.3f / 4095.0f);

  // --- Error (correct sign) ---
  e_k = v_hall - Vref;

  // --- Lead compensator (Δu) ---
  du_k = b0 * e_k + b1 * e_k1 - a1 * du_k1;

  // --- Apply scaling + bias ---
  u_k = u_bias + du_scale * du_k;

  // --- PWM ---
  analogWrite(pwmPin,    (int)u_k);
  analogWrite(pwmInvPin, 1023 - (int)u_k);

  // --- State update ---
  e_k1  = e_k;
  du_k1 = du_k;

  // --- DEBUG (temporary) ---
  Serial.print("| Vhall = ");
  Serial.print(v_hall, 4);
  Serial.print(" | u = ");
  Serial.print(u_k, 3);
  Serial.print(" | du = ");
  Serial.print(du_k, 5);
  Serial.print(" | e = ");
  Serial.print(e_k, 4);
  Serial.println();

  return true;
}

// ============================================================
//   SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(pwmPin, OUTPUT);
  pinMode(pwmInvPin, OUTPUT);

  analogWriteFreq(50000);
  analogWriteRange(1023);

  updateCoefficients();

  add_repeating_timer_us(-1000, controlISR, NULL, &control_timer);

  Serial.println("Maglev controller active.");
  Serial.println("Commands:");
  Serial.println(" G <val>  -> G0");
  Serial.println(" Z <val>  -> wz (rad/s)");
  Serial.println(" P <val>  -> wp (rad/s)");
  Serial.println(" R <val>  -> Vref (position)");
  Serial.println(" B <val>  -> u_bias");
  Serial.println(" S <val>  -> du_scale");
}

// ============================================================
//   SERIAL TUNING
// ============================================================
void loop() {

  if (Serial.available()) {
    char cmd = Serial.read();
    float val = Serial.parseFloat();

    if      (cmd == 'G') G0 = val;
    else if (cmd == 'Z') wz = val;
    else if (cmd == 'P') wp = val;
    else if (cmd == 'R') Vref = val;
    else if (cmd == 'B') u_bias = val;
    else if (cmd == 'S') du_scale = val;

    updateCoefficients();

    Serial.print("G0="); Serial.print(G0);
    Serial.print(" | wz="); Serial.print(wz);
    Serial.print(" | wp="); Serial.print(wp);
    Serial.print(" | Vref="); Serial.print(Vref,4);
    Serial.print(" | u_bias="); Serial.print(u_bias);
    Serial.print(" | du_scale="); Serial.println(du_scale);

    while (Serial.available()) Serial.read();
  }
}
