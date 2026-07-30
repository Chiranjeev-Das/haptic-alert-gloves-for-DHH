/*
 * ============================================================
 *  INERTIAL-GATED HAPTIC ALERT SYSTEM  —  v6.7
 *
 *  Changes from v6.5:
 *    - Wire.setClock(400000) added after Wire.begin() in setup():
 *        Runs I2C at 400kHz fast-mode so 32 IMU reads complete faster,
 *        keeping loop() tighter and reducing DMA backlog accumulation.
 *    - I2S DMA flush added in Phase 2 (audio read):
 *        After the blocking i2s_read, a zero-timeout flush loop drains
 *        any backlogged DMA frames so Phase 3 onward always sees the
 *        freshest 16ms window of audio. Prevents corrScore going erratic
 *        when the loop lags (heavy web traffic, WiFi packets, etc.).
 *        DC removal (Phase 3) is untouched — runs on the flushed data.
 *        All classification, gating, haptic, calib logic untouched.
 *
 *  Changes from v6.4:
 *    - selfNoise gate now ONLY applies when reportedMotion == MOVING.
 *        When STILL, corrScore gate is skipped entirely — "motion gated"
 *        can never appear while the hand is classified as still.
 *    - Moving haptic intensity raised: suppression 0.50 → 0.40 (60% intensity)
 *    - STILL_VAR_THR default widened: 0.0010 → 0.0015 — small gestures,
 *        slight hand shifts, and minor wrist rotations now stay STILL.
 *        Slider range also widened (max 0.015) to give more headroom.
 *    - Dashboard slider default updated to match new STILL_VAR_THR.
 *    - Version string bumped to v6.5 in title + h1.
 * ============================================================
 */

#include <driver/i2s.h>
#include <arduinoFFT.h>
#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_wifi.h"

// ─── WIFI ─────────────────────────────────────────────────────────────────
#define AP_SSID     "HapticSystem"
#define AP_PASSWORD "dsp12345"

// ─── PINS ─────────────────────────────────────────────────────────────────
#define I2S_WS        19
#define I2S_SCK       18
#define I2S_SD        17
#define I2C_SDA        4
#define I2C_SCL       16
#define VIB_PIN       33

// ─── AUDIO CONFIG ─────────────────────────────────────────────────────────
#define SAMPLE_RATE     16000
#define AUDIO_SAMPLES     256
#define AUDIO_BINS       (AUDIO_SAMPLES / 2)

// ─── IMU CONFIG ───────────────────────────────────────────────────────────
#define IMU_SAMPLES      32
#define IMU_SAMPLE_US  1000
#define IMU_SAMPLE_HZ  (1000000 / IMU_SAMPLE_US)
#define IMU_BINS        (IMU_SAMPLES / 2)

#define ACCEL_SCALE  16384.0f
#define GYRO_SCALE     131.0f

// ─── CORRELATION ──────────────────────────────────────────────────────────
#define CORR_POINTS 8

// ─── THRESHOLDS ───────────────────────────────────────────────────────────
float CORR_THRESHOLD  = 0.60f;
float FREQ_LOW        = 1200.0f;
float FREQ_HIGH       = 8000.0f;
float BAND_ENERGY_MIN = 2000.0f;

// STILL_VAR_THR: calibrated from worn-glove data
//   still p99=0.00099, walking min=0.00121 → original gap at 0.0010
//   Widened to 0.0015 so small gestures / slight wrist shifts stay STILL.
//   Tune upward further if brief hand shifts keep flipping to MOVING.
float STILL_VAR_THR = 0.0015f;

// ENERGY_OVERRIDE_MULT:
//   If band energy > BAND_ENERGY_MIN × this multiplier the signal is
//   treated as a confident external sound and the self-noise gate is
//   bypassed.  Taps / rustles cannot generate this much energy in the
//   alert band; loud alarms / doorbells will.
//   Tune live via the dashboard slider.  Start at 3.0×.
float ENERGY_OVERRIDE_MULT = 3.0f;

int DEBOUNCE_FRAMES = 10;  // ~500ms before state commits

// ─── HAPTIC ───────────────────────────────────────────────────────────────
#define HAPTIC_MIN_PWM   60
#define HAPTIC_MAX_PWM  210

// ─── MOTION TYPES ─────────────────────────────────────────────────────────
enum MotionType { STILL, MOVING };

const char* motionName(MotionType m) {
  return m == STILL ? "still" : "moving";
}

float suppressionWeight(MotionType m) {
  return m == STILL ? 0.00f : 0.40f;
  // STILL  → 0%  suppression → 100% haptic intensity
  // MOVING → 40% suppression →  60% haptic intensity
}

// ─── DSP BUFFERS ──────────────────────────────────────────────────────────
float   audioReal[AUDIO_SAMPLES];
float   audioImag[AUDIO_SAMPLES];
float   imuReal[IMU_SAMPLES];
float   imuImag[IMU_SAMPLES];
float   imuMagBuf[IMU_SAMPLES];
float   gyroMagBuf[IMU_SAMPLES];
int32_t i2sRaw[AUDIO_SAMPLES];

ArduinoFFT<float> audioFFT(audioReal, audioImag, AUDIO_SAMPLES, (float)SAMPLE_RATE);
ArduinoFFT<float> imuFFT  (imuReal,   imuImag,   IMU_SAMPLES,   (float)IMU_SAMPLE_HZ);

MPU6050 mpu;
WebServer server(80);

// ─── VARIANCE WINDOW ──────────────────────────────────────────────────────
#define VAR_WINDOW 20
float accelVarBuf[VAR_WINDOW];
int   varBufIdx  = 0;
bool  varBufFull = false;

float pushAndGetVariance(float newVal) {
  accelVarBuf[varBufIdx] = newVal;
  varBufIdx = (varBufIdx + 1) % VAR_WINDOW;
  if (varBufIdx == 0) varBufFull = true;
  int count = varBufFull ? VAR_WINDOW : varBufIdx;
  if (count < 2) return 9999.0f;
  float mean = 0;
  for (int i = 0; i < count; i++) mean += accelVarBuf[i];
  mean /= count;
  float var = 0;
  for (int i = 0; i < count; i++) { float d=accelVarBuf[i]-mean; var+=d*d; }
  return var / count;
}

// ─── JERK ─────────────────────────────────────────────────────────────────
float prevGyroRMS = 0.0f;
float computeJerk(float g) { float j=fabsf(g-prevGyroRMS); prevGyroRMS=g; return j; }

// ─── DEBOUNCE ─────────────────────────────────────────────────────────────
MotionType reportedMotion = STILL;
MotionType pendingMotion  = STILL;
int        pendingCount   = 0;

void updateMotionState(MotionType raw) {
  if (raw == pendingMotion) {
    if (++pendingCount >= DEBOUNCE_FRAMES) reportedMotion = pendingMotion;
  } else {
    pendingMotion = raw;
    pendingCount  = 1;
  }
}

// ─── CLASSIFIER — 2 states ────────────────────────────────────────────────
MotionType classifyRaw(float accelVariance) {
  return (accelVariance < STILL_VAR_THR) ? STILL : MOVING;
}

// ─── HELPERS ──────────────────────────────────────────────────────────────
float bandEnergy(float* s, int bins, float res, float lo, float hi) {
  int a=constrain((int)(lo/res),0,bins-1), b=constrain((int)(hi/res),0,bins-1);
  float e=0; for(int i=a;i<=b;i++) e+=s[i]; return e;
}

float pearsonCorr(float* a, float* b, int len) {
  float mA=0,mB=0;
  for(int i=0;i<len;i++){mA+=a[i];mB+=b[i];} mA/=len;mB/=len;
  float n=0,dA=0,dB=0;
  for(int i=0;i<len;i++){float da=a[i]-mA,db=b[i]-mB;n+=da*db;dA+=da*da;dB+=db*db;}
  if(dA<1e-9f||dB<1e-9f) return 0.0f;
  return n/sqrtf(dA*dB);
}

void setHaptic(int pwm) { ledcWrite(VIB_PIN, constrain(pwm,0,255)); }

String formatUptime(unsigned long ms) {
  unsigned long s=ms/1000,m=s/60; s%=60; unsigned long h=m/60; m%=60;
  char buf[16]; snprintf(buf,16,"%02lu:%02lu:%02lu",h,m,s); return String(buf);
}

// ─── TELEMETRY ────────────────────────────────────────────────────────────
volatile float t_dominantFreq=0, t_bandEnergy=0, t_imuRMS=0, t_gyroRMS=0;
volatile float t_accelVar=0, t_gyroJerk=0, t_corrScore=0;
volatile int   t_hapticPWM=0;
volatile char  t_decision[24]  = "booting";
volatile char  t_motionType[8] = "—";

// ─── ALERT LOG ────────────────────────────────────────────────────────────
#define LOG_SIZE 50
struct AlertEntry {
  unsigned long timestamp_ms;
  float dominantFreq, bandEnergy, imuRMS, gyroRMS, corrScore, suppress;
  char  motionType[8], type[20];
};
AlertEntry alertLog[LOG_SIZE];
int logHead=0, logCount=0;

void logAlert(float freq, float energy, float iRMS, float gRMS,
              float corr, float sup, const char* motion, const char* type) {
  alertLog[logHead].timestamp_ms = millis();
  alertLog[logHead].dominantFreq = freq;
  alertLog[logHead].bandEnergy   = energy;
  alertLog[logHead].imuRMS       = iRMS;
  alertLog[logHead].gyroRMS      = gRMS;
  alertLog[logHead].corrScore    = corr;
  alertLog[logHead].suppress     = sup;
  strncpy(alertLog[logHead].motionType, motion, 7);
  strncpy(alertLog[logHead].type,       type,   19);
  logHead = (logHead+1) % LOG_SIZE;
  if (logCount < LOG_SIZE) logCount++;
}

// ─── CALIBRATION LOGGER ───────────────────────────────────────────────────
// gyroJerk removed from struct — was causing 4112-byte DRAM overflow
#define CALIB_MAX 1800
struct CalibEntry {
  char  label[16];
  float accelRMS, gyroRMS, accelVar;
  unsigned long ms;
};
CalibEntry calibBuf[CALIB_MAX];
int  calibCount   = 0;
bool calibLogging = false;
char calibLabel[16] = "";

void calibRecord(float aRMS, float gRMS, float aVar) {
  if (!calibLogging || calibCount >= CALIB_MAX) return;
  strncpy(calibBuf[calibCount].label, calibLabel, 15);
  calibBuf[calibCount].accelRMS = aRMS;
  calibBuf[calibCount].gyroRMS  = gRMS;
  calibBuf[calibCount].accelVar = aVar;
  calibBuf[calibCount].ms       = millis();
  calibCount++;
}

// ═══════════════════════════════════════════════════════════════════════════
//  WEB HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

void handleData() {
  char json[650];
  snprintf(json, sizeof(json),
    "{\"domFreq\":%.1f,\"bandEnergy\":%.1f,\"imuRMS\":%.4f,\"gyroRMS\":%.2f,"
    "\"accelVar\":%.6f,\"gyroJerk\":%.2f,"
    "\"corrScore\":%.3f,\"hapticPWM\":%d,\"decision\":\"%s\",\"motionType\":\"%s\","
    "\"corrThresh\":%.2f,\"freqLow\":%.0f,\"freqHigh\":%.0f,\"bandMin\":%.0f,"
    "\"stillVar\":%.6f,\"debounce\":%d,\"energyOverrideMult\":%.2f,"
    "\"calibLogging\":%s,\"calibLabel\":\"%s\",\"calibCount\":%d,\"calibMax\":%d}",
    (float)t_dominantFreq, (float)t_bandEnergy,
    (float)t_imuRMS,       (float)t_gyroRMS,
    (float)t_accelVar,     (float)t_gyroJerk,
    (float)t_corrScore,    (int)t_hapticPWM,
    (const char*)t_decision, (const char*)t_motionType,
    CORR_THRESHOLD, FREQ_LOW, FREQ_HIGH, BAND_ENERGY_MIN,
    STILL_VAR_THR, DEBOUNCE_FRAMES, ENERGY_OVERRIDE_MULT,
    calibLogging ? "true" : "false",
    calibLabel, calibCount, CALIB_MAX);
  server.send(200, "application/json", json);
}

void handleSetThresh() {
  if (server.hasArg("corr"))           CORR_THRESHOLD      = server.arg("corr").toFloat();
  if (server.hasArg("freqLow"))        FREQ_LOW            = server.arg("freqLow").toFloat();
  if (server.hasArg("freqHigh"))       FREQ_HIGH           = server.arg("freqHigh").toFloat();
  if (server.hasArg("bandMin"))        BAND_ENERGY_MIN     = server.arg("bandMin").toFloat();
  if (server.hasArg("stillVar"))       STILL_VAR_THR       = server.arg("stillVar").toFloat();
  if (server.hasArg("debounce"))       DEBOUNCE_FRAMES     = server.arg("debounce").toInt();
  if (server.hasArg("energyOverride")) ENERGY_OVERRIDE_MULT = server.arg("energyOverride").toFloat();
  server.send(200, "text/plain", "ok");
}

void handleCalibStart() {
  if (!server.hasArg("label")) { server.send(400,"text/plain","missing label"); return; }
  strncpy(calibLabel, server.arg("label").c_str(), 15);
  calibLogging = true;
  Serial.printf("[CALIB] Recording: %s\n", calibLabel);
  server.send(200, "text/plain", "ok");
}
void handleCalibStop()  { calibLogging=false; Serial.printf("[CALIB] Stopped. %d entries\n",calibCount); server.send(200,"text/plain","ok"); }
void handleCalibClear() { calibLogging=false; calibCount=0; memset(calibLabel,0,sizeof(calibLabel)); Serial.println("[CALIB] Cleared."); server.send(200,"text/plain","ok"); }

void handleDownload() {
  if (calibCount==0) { server.send(200,"text/plain","No data."); return; }
  server.sendHeader("Content-Disposition","attachment; filename=calib_data.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200,"text/csv","");
  server.sendContent("label,accelRMS,gyroRMS,accelVariance,timestamp_ms\n");
  for (int i=0; i<calibCount; i++) {
    char row[72];
    snprintf(row, sizeof(row), "%s,%.5f,%.3f,%.7f,%lu\n",
      calibBuf[i].label, calibBuf[i].accelRMS,
      calibBuf[i].gyroRMS, calibBuf[i].accelVar, calibBuf[i].ms);
    server.sendContent(row);
  }
  server.sendContent("");
}

void handleLogData() {
  String json = "[";
  int start = (logCount < LOG_SIZE) ? 0 : logHead;
  for (int i=0; i<logCount; i++) {
    int idx = (start+i) % LOG_SIZE;
    if (i>0) json += ",";
    char e[200];
    snprintf(e, sizeof(e),
      "{\"t\":\"%s\",\"freq\":%.0f,\"energy\":%.0f,\"imu\":%.4f,"
      "\"gyro\":%.2f,\"corr\":%.3f,\"suppress\":%.2f,\"motion\":\"%s\",\"type\":\"%s\"}",
      formatUptime(alertLog[idx].timestamp_ms).c_str(),
      alertLog[idx].dominantFreq, alertLog[idx].bandEnergy,
      alertLog[idx].imuRMS,       alertLog[idx].gyroRMS,
      alertLog[idx].corrScore,    alertLog[idx].suppress,
      alertLog[idx].motionType,   alertLog[idx].type);
    json += e;
  }
  json += "]";
  server.send(200, "application/json", json);
}

void handleClearLog() { logHead=0; logCount=0; server.send(200,"text/plain","ok"); }

void handleRoot() {
  String html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>Haptic System v6.7</title>
<style>
  :root{
    --bg:#0d0d0f;--card:#16161a;--border:#2a2a32;
    --green:#00e5a0;--yellow:#f5c542;--red:#ff4f5e;
    --blue:#4fa3ff;--purple:#b57bff;--text:#e8e8f0;--muted:#6b6b80;
    --orange:#ff8c42;
  }
  *{box-sizing:border-box;margin:0;padding:0;}
  body{background:var(--bg);color:var(--text);font-family:'Courier New',monospace;padding:16px;}
  h1{font-size:1rem;letter-spacing:.15em;color:var(--green);margin-bottom:16px;text-transform:uppercase;}
  .tabs{display:flex;gap:8px;margin-bottom:16px;flex-wrap:wrap;}
  .tab{padding:8px 20px;background:var(--card);border:1px solid var(--border);border-radius:4px;
       cursor:pointer;font-family:inherit;font-size:.8rem;letter-spacing:.1em;color:var(--muted);text-transform:uppercase;}
  .tab.active{border-color:var(--green);color:var(--green);}
  .panel{display:none;}.panel.active{display:block;}
  .grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:10px;margin-bottom:16px;}
  .card{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:14px;}
  .card .label{font-size:.65rem;letter-spacing:.12em;color:var(--muted);text-transform:uppercase;margin-bottom:6px;}
  .card .value{font-size:1.2rem;font-weight:bold;}
  .card .unit{font-size:.7rem;color:var(--muted);margin-left:4px;}
  #decisionCard{grid-column:span 3;border-color:var(--green);}
  #decisionCard .value{font-size:1rem;color:var(--green);}
  #motionCard .value{color:var(--purple);}
  .freq-bar-wrap{margin-top:8px;position:relative;height:4px;background:var(--border);border-radius:2px;}
  .freq-bar-fill{position:absolute;height:100%;background:var(--blue);border-radius:2px;transition:left .3s,width .3s;}
  .freq-bar-marker{position:absolute;top:-3px;width:2px;height:10px;background:var(--yellow);border-radius:1px;transition:left .3s;}
  .var-bar-wrap{margin-top:6px;position:relative;height:3px;background:var(--border);border-radius:2px;}
  .var-bar-fill{position:absolute;height:100%;border-radius:2px;max-width:100%;transition:width .3s,background .3s;}
  .state-legend{display:flex;gap:12px;flex-wrap:wrap;margin-bottom:16px;
                background:var(--card);border:1px solid var(--border);border-radius:6px;padding:12px;}
  .state-item{display:flex;align-items:center;gap:8px;font-size:.72rem;}
  .state-dot{width:10px;height:10px;border-radius:50%;}
  .state-suppress{color:var(--muted);font-size:.65rem;margin-left:2px;}
  .chart-wrap{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:14px;margin-bottom:16px;}
  .chart-wrap .label{font-size:.65rem;letter-spacing:.12em;color:var(--muted);text-transform:uppercase;margin-bottom:10px;}
  canvas{width:100%!important;height:120px!important;}
  .legend{display:flex;gap:16px;margin-bottom:8px;flex-wrap:wrap;}
  .legend-item{display:flex;align-items:center;gap:6px;font-size:.7rem;color:var(--muted);}
  .legend-dot{width:8px;height:8px;border-radius:50%;}
  .sliders{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:14px;margin-bottom:16px;}
  .sliders h2{font-size:.7rem;letter-spacing:.12em;color:var(--muted);text-transform:uppercase;margin-bottom:12px;}
  .slider-section{border-top:1px solid var(--border);padding-top:12px;margin-top:4px;}
  .slider-section-label{font-size:.6rem;color:var(--muted);letter-spacing:.1em;text-transform:uppercase;margin-bottom:10px;}
  .slider-row{margin-bottom:14px;}
  .slider-row .row-top{display:flex;justify-content:space-between;font-size:.75rem;margin-bottom:6px;}
  .slider-row .row-top .name{color:var(--text);}
  .slider-row .row-top .val{color:var(--yellow);}
  input[type=range]{width:100%;-webkit-appearance:none;height:3px;background:var(--border);border-radius:2px;outline:none;}
  input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:14px;height:14px;background:var(--green);border-radius:50%;cursor:pointer;}
  .calib-box{background:var(--card);border:1px solid var(--border);border-radius:6px;padding:16px;}
  .calib-box h2{font-size:.7rem;letter-spacing:.12em;color:var(--orange);text-transform:uppercase;margin-bottom:14px;}
  .calib-status{background:#0a0a0c;border:1px solid var(--border);border-radius:4px;padding:10px 14px;margin-bottom:14px;font-size:.8rem;}
  .calib-status .cs-label{color:var(--muted);font-size:.65rem;letter-spacing:.1em;text-transform:uppercase;margin-bottom:4px;}
  .calib-status .cs-val{color:var(--orange);}
  .calib-status.recording{border-color:var(--red);}
  .calib-status.recording .cs-val{color:var(--red);}
  .btn-row{display:flex;gap:8px;margin-bottom:12px;flex-wrap:wrap;}
  .btn{padding:9px 16px;border-radius:4px;font-family:inherit;font-size:.75rem;letter-spacing:.1em;cursor:pointer;text-transform:uppercase;border:1px solid;}
  .btn-still  {border-color:var(--green);  color:var(--green);  background:transparent;}
  .btn-moving {border-color:var(--orange); color:var(--orange); background:transparent;}
  .btn-stop   {border-color:var(--muted);  color:var(--muted);  background:transparent;}
  .btn-dl     {border-color:var(--blue);   color:var(--blue);   background:transparent;}
  .btn-clr    {border-color:var(--muted);  color:var(--muted);  background:transparent;}
  .btn.active-btn{color:var(--bg)!important;}
  .btn-still.active-btn  {background:var(--green);}
  .btn-moving.active-btn {background:var(--orange);}
  .progress-wrap{height:6px;background:var(--border);border-radius:3px;overflow:hidden;margin-bottom:12px;}
  .progress-fill{height:100%;background:var(--orange);border-radius:3px;transition:width .5s;}
  .calib-hint{font-size:.72rem;color:var(--muted);line-height:1.8;}
  .calib-hint span{color:var(--text);}
  #logTable{width:100%;border-collapse:collapse;font-size:.7rem;}
  #logTable th{text-align:left;padding:8px;color:var(--muted);border-bottom:1px solid var(--border);font-weight:normal;letter-spacing:.1em;text-transform:uppercase;}
  #logTable td{padding:7px;border-bottom:1px solid var(--border);}
  .type-alert{color:var(--red);}.type-alert-low{color:var(--yellow);}
  .clear-btn{margin-bottom:12px;padding:8px 18px;background:transparent;border:1px solid var(--red);color:var(--red);border-radius:4px;font-family:inherit;font-size:.75rem;cursor:pointer;letter-spacing:.1em;}
  .clear-btn:hover{background:var(--red);color:var(--bg);}
  .empty-msg{color:var(--muted);font-size:.8rem;padding:20px 0;text-align:center;}
</style>
</head>
<body>
<h1>&#9670; Haptic Alert System v6.7</h1>

<div class="tabs">
  <button class="tab active" onclick="switchTab('monitor')">Monitor</button>
  <button class="tab"        onclick="switchTab('calib')">Calibration</button>
  <button class="tab"        onclick="switchTab('log')">Alert Log</button>
</div>

<!-- ═══ MONITOR ═══ -->
<div class="panel active" id="panel-monitor">

  <div class="state-legend">
    <div class="state-item"><div class="state-dot" style="background:#00e5a0"></div><span>Still</span><span class="state-suppress">100% haptic intensity</span></div>
    <div class="state-item"><div class="state-dot" style="background:#ff8c42"></div><span>Moving</span><span class="state-suppress">60% haptic intensity</span></div>
  </div>

  <div class="grid">
    <div class="card" id="freqCard">
      <div class="label">Dominant Freq</div>
      <div class="value" id="val-freq">—<span class="unit">Hz</span></div>
      <div class="freq-bar-wrap">
        <div class="freq-bar-fill" id="freq-bar-fill"></div>
        <div class="freq-bar-marker" id="freq-bar-marker"></div>
      </div>
    </div>
    <div class="card">
      <div class="label">Accel Variance</div>
      <div class="value" id="val-var">—</div>
      <div class="var-bar-wrap"><div class="var-bar-fill" id="var-bar"></div></div>
    </div>
    <div class="card" id="motionCard">
      <div class="label">Motion State</div>
      <div class="value" id="val-motion">—</div>
    </div>
    <div class="card">
      <div class="label">Corr Score</div>
      <div class="value" id="val-corr">—</div>
    </div>
    <div class="card">
      <div class="label">Gyro Jerk</div>
      <div class="value" id="val-jerk">—<span class="unit">°/s/f</span></div>
    </div>
    <div class="card">
      <div class="label">Haptic PWM</div>
      <div class="value" id="val-pwm">—<span class="unit">/255</span></div>
    </div>
    <div class="card" id="decisionCard">
      <div class="label">Decision</div>
      <div class="value" id="val-decision">—</div>
    </div>
  </div>

  <div class="chart-wrap">
    <div class="label">Live signals — last 60s</div>
    <div class="legend">
      <div class="legend-item"><div class="legend-dot" style="background:#4fa3ff"></div>corrScore</div>
      <div class="legend-item"><div class="legend-dot" style="background:#ff4f5e"></div>corr threshold</div>
      <div class="legend-item"><div class="legend-dot" style="background:#00e5a0"></div>accel var (norm)</div>
    </div>
    <canvas id="corrChart"></canvas>
  </div>

  <div class="sliders">
    <h2>Threshold Tuning</h2>
    <div class="slider-section">
      <div class="slider-section-label">Alert Band — only affects what triggers an alert</div>
      <div class="slider-row">
        <div class="row-top"><span class="name">Freq Low</span><span class="val" id="disp-freqLow">1200 Hz</span></div>
        <input type="range" id="sl-freqLow" min="100" max="7000" step="50" value="1200" oninput="updateSlider('freqLow',this.value)">
      </div>
      <div class="slider-row">
        <div class="row-top"><span class="name">Freq High</span><span class="val" id="disp-freqHigh">8000 Hz</span></div>
        <input type="range" id="sl-freqHigh" min="100" max="15000" step="50" value="8000" oninput="updateSlider('freqHigh',this.value)">
      </div>
      <div class="slider-row">
        <div class="row-top"><span class="name">Band Energy Min</span><span class="val" id="disp-bandMin">2000</span></div>
        <input type="range" id="sl-bandMin" min="500" max="200000" step="500" value="2000" oninput="updateSlider('bandMin',this.value)">
      </div>
    </div>
    <div class="slider-section" style="margin-top:12px;">
      <div class="slider-section-label">Self-noise Gate — full spectrum, independent of alert band</div>
      <div class="slider-row">
        <div class="row-top"><span class="name">Correlation Threshold</span><span class="val" id="disp-corr">0.60</span></div>
        <input type="range" id="sl-corr" min="-0.95" max="0.95" step="0.01" value="0.60" oninput="updateSlider('corr',this.value)">
      </div>
    </div>
    <div class="slider-section" style="margin-top:12px;">
      <div class="slider-section-label">Motion Classifier</div>
      <div class="slider-row">
        <div class="row-top"><span class="name">Still Variance Threshold</span><span class="val" id="disp-stillVar">0.001000</span></div>
        <input type="range" id="sl-stillVar" min="0.00005" max="0.015" step="0.00005" value="0.0015" oninput="updateSlider('stillVar',this.value)">
      </div>
      <div class="slider-row">
        <div class="row-top"><span class="name">Debounce Frames (~50ms each)</span><span class="val" id="disp-debounce">10</span></div>
        <input type="range" id="sl-debounce" min="2" max="20" step="1" value="10" oninput="updateSlider('debounce',this.value)">
      </div>
      <div class="slider-row">
        <div class="row-top">
          <span class="name">Energy Override (× Band Min)</span>
          <span class="val" id="disp-energyOverride">3.00×</span>
        </div>
        <!-- If band energy clears this bar, the self-noise gate is bypassed.      -->
        <!-- Taps / rustles stay below it. Loud alarms / doorbells clear it.       -->
        <!-- Lower = override fires more easily. Raise if taps still break through. -->
        <input type="range" id="sl-energyOverride" min="1.5" max="10" step="0.25" value="3" oninput="updateSlider('energyOverride',this.value)">
      </div>
    </div>
  </div>
</div>

<!-- ═══ CALIBRATION ═══ -->
<div class="panel" id="panel-calib">
  <div class="calib-box">
    <h2>&#9650; IMU Calibration Logger</h2>
    <div class="calib-status" id="calibStatus">
      <div class="cs-label">Status</div>
      <div class="cs-val" id="cs-val">Idle — press a position to start</div>
      <div style="margin-top:6px;font-size:.7rem;color:var(--muted);">Entries: <span id="cs-count">0</span> / 1800</div>
    </div>
    <div class="progress-wrap"><div class="progress-fill" id="calibProgress" style="width:0%"></div></div>
    <div class="btn-row">
      <button class="btn btn-still"  onclick="calibStart('still')">&#9679; Still</button>
      <button class="btn btn-moving" onclick="calibStart('moving')">&#9679; Moving</button>
      <button class="btn btn-stop"   onclick="calibStop()">&#9646; Stop</button>
    </div>
    <div class="btn-row">
      <button class="btn btn-dl"  onclick="downloadCSV()">&#8595; Download CSV</button>
      <button class="btn btn-clr" onclick="calibClear()">&#10005; Clear All</button>
    </div>
    <div class="calib-hint">
      <p>1. <span>Still</span> → hand relaxed at side, change angles mid-pass → 30s → <span>Stop</span></p>
      <p>2. <span>Moving</span> → walk, shake, move naturally → 30s → <span>Stop</span></p>
      <p>CSV has accelVariance column — tune <span>Still Variance Threshold</span> from that data.</p>
    </div>
  </div>
</div>

<!-- ═══ LOG ═══ -->
<div class="panel" id="panel-log">
  <button class="clear-btn" onclick="clearLog()">&#10005; CLEAR LOG</button>
  <table id="logTable">
    <thead><tr><th>Time</th><th>Type</th><th>Motion</th><th>Freq</th><th>BandE</th><th>Gyro</th><th>Suppress</th></tr></thead>
    <tbody id="logBody"><tr><td colspan="7" class="empty-msg">No alerts yet.</td></tr></tbody>
  </table>
</div>

<script>
function switchTab(name) {
  const names=['monitor','calib','log'];
  document.querySelectorAll('.tab').forEach((t,i)=>t.classList.toggle('active',names[i]===name));
  document.querySelectorAll('.panel').forEach((p,i)=>p.classList.toggle('active','panel-'+names[i]==='panel-'+name));
  if(name==='log') fetchLog();
}

const MAX_POINTS=60, FREQ_MAX=15000;
let corrHistory=[], varHistory=[], threshLine=0.60;

function drawChart() {
  const canvas=document.getElementById('corrChart');
  const W=canvas.offsetWidth, H=120;
  canvas.width=W; canvas.height=H;
  const ctx=canvas.getContext('2d');
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='#2a2a32'; ctx.lineWidth=1;
  [.25,.5,.75,1].forEach(y=>{const py=H-y*H;ctx.beginPath();ctx.moveTo(0,py);ctx.lineTo(W,py);ctx.stroke();});
  ctx.strokeStyle='#ff4f5e'; ctx.lineWidth=1; ctx.setLineDash([4,4]);
  const ty=H-Math.max(0,Math.min(1,(threshLine+1)/2))*H;
  ctx.beginPath();ctx.moveTo(0,ty);ctx.lineTo(W,ty);ctx.stroke();ctx.setLineDash([]);
  const drawLine=(h,color,norm,width)=>{
    if(h.length<2) return;
    ctx.strokeStyle=color; ctx.lineWidth=width; ctx.beginPath();
    h.forEach((v,i)=>{const x=(i/(MAX_POINTS-1))*W,y=H-Math.max(0,Math.min(1,v/norm))*H;i===0?ctx.moveTo(x,y):ctx.lineTo(x,y);});
    ctx.stroke();
  };
  drawLine(varHistory,  '#00e5a0', 0.10, 1.5);
  drawLine(corrHistory, '#4fa3ff', 2.0,  2.0);
}

const motionColor={'still':'#00e5a0','moving':'#ff8c42'};

async function fetchData() {
  try {
    const r=await fetch('/data'), d=await r.json();
    document.getElementById('val-freq').innerHTML=d.domFreq.toFixed(0)+'<span class="unit">Hz</span>';
    document.getElementById('val-var').textContent=d.accelVar.toFixed(6);
    document.getElementById('val-motion').textContent=d.motionType;
    document.getElementById('val-corr').textContent=d.corrScore.toFixed(3);
    document.getElementById('val-jerk').innerHTML=d.gyroJerk.toFixed(1)+'<span class="unit">°/s/f</span>';
    document.getElementById('val-pwm').innerHTML=d.hapticPWM+'<span class="unit">/255</span>';
    document.getElementById('val-decision').textContent=d.decision;

    const vb=document.getElementById('var-bar');
    vb.style.width=Math.min(d.accelVar/0.10*100,100)+'%';
    vb.style.background=d.accelVar<d.stillVar?'#00e5a0':'#ff8c42';

    const mc=document.getElementById('motionCard'), mv=document.getElementById('val-motion');
    const col=motionColor[d.motionType]||'#6b6b80';
    mc.style.borderColor=col; mv.style.color=col;

    const fill=document.getElementById('freq-bar-fill'), marker=document.getElementById('freq-bar-marker');
    fill.style.left=(d.freqLow/FREQ_MAX*100)+'%';
    fill.style.width=((d.freqHigh-d.freqLow)/FREQ_MAX*100)+'%';
    marker.style.left=Math.min(d.domFreq/FREQ_MAX*100,99)+'%';
    const inRange=d.domFreq>=d.freqLow&&d.domFreq<=d.freqHigh;
    document.getElementById('freqCard').style.borderColor=inRange?'#00e5a0':'#2a2a32';
    document.getElementById('val-freq').style.color=inRange?'#00e5a0':'#4fa3ff';

    const card=document.getElementById('decisionCard'), v=document.getElementById('val-decision');
    if(d.decision.includes('ALERT')){card.style.borderColor='#ff4f5e';v.style.color='#ff4f5e';}
    else if(d.decision.includes('gated')){card.style.borderColor='#f5c542';v.style.color='#f5c542';}
    else{card.style.borderColor='#00e5a0';v.style.color='#00e5a0';}

    if(!window._ss){
      const s=id=>document.getElementById(id);
      s('sl-corr').value=d.corrThresh;               s('disp-corr').textContent=parseFloat(d.corrThresh).toFixed(2);
      s('sl-freqLow').value=d.freqLow;               s('disp-freqLow').textContent=parseFloat(d.freqLow).toFixed(0)+' Hz';
      s('sl-freqHigh').value=d.freqHigh;             s('disp-freqHigh').textContent=parseFloat(d.freqHigh).toFixed(0)+' Hz';
      s('sl-bandMin').value=d.bandMin;               s('disp-bandMin').textContent=parseFloat(d.bandMin).toFixed(0);
      s('sl-stillVar').value=d.stillVar;             s('disp-stillVar').textContent=parseFloat(d.stillVar).toFixed(6);
      s('sl-debounce').value=d.debounce;             s('disp-debounce').textContent=d.debounce;
      s('sl-energyOverride').value=d.energyOverrideMult;
      s('disp-energyOverride').textContent=parseFloat(d.energyOverrideMult).toFixed(2)+'×';
      threshLine=d.corrThresh;
      window._ss=true;
    }

    document.getElementById('cs-count').textContent=d.calibCount;
    document.getElementById('calibProgress').style.width=(d.calibCount/d.calibMax*100)+'%';
    const sb=document.getElementById('calibStatus'), cs=document.getElementById('cs-val');
    if(d.calibLogging){sb.classList.add('recording');cs.textContent='Recording: '+d.calibLabel+' ('+d.calibCount+')';}
    else{sb.classList.remove('recording');cs.textContent=d.calibCount===0?'Idle — press a position to start':'Stopped — '+d.calibCount+' entries saved';}

    corrHistory.push(d.corrScore);
    varHistory.push(d.accelVar);
    if(corrHistory.length>MAX_POINTS){corrHistory.shift();varHistory.shift();}
    drawChart();
  } catch(e) {}
}
setInterval(fetchData, 1000); fetchData();

let _t=null;
function updateSlider(key, val) {
  const fval=parseFloat(val);
  const map={
    corr:           ['disp-corr',           v=>v.toFixed(2)],
    freqLow:        ['disp-freqLow',        v=>v.toFixed(0)+' Hz'],
    freqHigh:       ['disp-freqHigh',       v=>v.toFixed(0)+' Hz'],
    bandMin:        ['disp-bandMin',        v=>v.toFixed(0)],
    stillVar:       ['disp-stillVar',       v=>v.toFixed(6)],
    debounce:       ['disp-debounce',       v=>v.toFixed(0)],
    energyOverride: ['disp-energyOverride', v=>v.toFixed(2)+'×']
  };
  if(map[key]) document.getElementById(map[key][0]).textContent=map[key][1](fval);
  if(key==='corr') threshLine=fval;
  clearTimeout(_t);
  _t=setTimeout(()=>{
    const b=[
      'corr='+document.getElementById('sl-corr').value,
      'freqLow='+document.getElementById('sl-freqLow').value,
      'freqHigh='+document.getElementById('sl-freqHigh').value,
      'bandMin='+document.getElementById('sl-bandMin').value,
      'stillVar='+document.getElementById('sl-stillVar').value,
      'debounce='+document.getElementById('sl-debounce').value,
      'energyOverride='+document.getElementById('sl-energyOverride').value
    ].join('&');
    fetch('/setthresh',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:b});
  }, 300);
}

function calibStart(label) {
  document.querySelectorAll('.btn-still,.btn-moving').forEach(b=>b.classList.remove('active-btn'));
  const map={'still':'.btn-still','moving':'.btn-moving'};
  if(map[label]) document.querySelector(map[label]).classList.add('active-btn');
  fetch('/calibstart?label='+label, {method:'POST'});
}
function calibStop()  { document.querySelectorAll('.btn-still,.btn-moving').forEach(b=>b.classList.remove('active-btn')); fetch('/calibstop',{method:'POST'}); }
function calibClear() { if(!confirm('Clear all?')) return; document.querySelectorAll('.btn-still,.btn-moving').forEach(b=>b.classList.remove('active-btn')); fetch('/calibclear',{method:'POST'}); }
function downloadCSV() { window.location.href='/download'; }

async function fetchLog() {
  try {
    const r=await fetch('/logdata'), data=await r.json();
    const body=document.getElementById('logBody');
    if(!data.length){body.innerHTML='<tr><td colspan="7" class="empty-msg">No alerts yet.</td></tr>';return;}
    body.innerHTML=[...data].reverse().map(e=>{
      const cls=e.type.includes('low')?'type-alert-low':'type-alert';
      return`<tr><td>${e.t}</td><td class="${cls}">${e.type}</td><td>${e.motion}</td>
        <td>${parseFloat(e.freq).toFixed(0)}</td><td>${parseFloat(e.energy).toFixed(0)}</td>
        <td>${parseFloat(e.gyro).toFixed(1)}</td><td>${parseFloat(e.suppress).toFixed(2)}</td></tr>`;
    }).join('');
  } catch(e) {}
}
async function clearLog() { await fetch('/clearlog',{method:'POST'}); fetchLog(); }
</script>
</body>
</html>
)rawhtml";
  server.send(200, "text/html", html);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== INERTIAL-GATED HAPTIC v6.7 ===");

  ledcAttach(VIB_PIN, 1000, 8);
  setHaptic(0);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // I2C fast-mode: 32 IMU reads complete ~2× faster,
                          // keeps loop() tight and reduces DMA backlog build-up
  delay(100);
  mpu.initialize();
  delay(50);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  Serial.println("[OK]   MPU6050 online");

  i2s_config_t i2s_cfg = {
    .mode=(i2s_mode_t)(I2S_MODE_MASTER|I2S_MODE_RX),
    .sample_rate=SAMPLE_RATE, .bits_per_sample=I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format=I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format=I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags=ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count=8, .dma_buf_len=64, .use_apll=false
  };
  i2s_pin_config_t pin_cfg = {
    .bck_io_num=I2S_SCK, .ws_io_num=I2S_WS,
    .data_out_num=I2S_PIN_NO_CHANGE, .data_in_num=I2S_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_cfg, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_cfg);
  i2s_start(I2S_NUM_0);
  Serial.println("[OK]   I2S mic online");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[OK]   WiFi AP → "); Serial.print(AP_SSID);
  Serial.print("  IP: "); Serial.println(WiFi.softAPIP());

  server.on("/",           HTTP_GET,  handleRoot);
  server.on("/data",       HTTP_GET,  handleData);
  server.on("/setthresh",  HTTP_POST, handleSetThresh);
  server.on("/logdata",    HTTP_GET,  handleLogData);
  server.on("/clearlog",   HTTP_POST, handleClearLog);
  server.on("/calibstart", HTTP_POST, handleCalibStart);
  server.on("/calibstop",  HTTP_POST, handleCalibStop);
  server.on("/calibclear", HTTP_POST, handleCalibClear);
  server.on("/download",   HTTP_GET,  handleDownload);
  server.begin();
  Serial.println("[OK]   Web server → http://192.168.4.1");
  Serial.println("=== SYSTEM READY ===\n");
  Serial.println("  Freq(Hz) | BandE   | AccVar    | GyroJerk | Motion | Corr   | PWM | Decision");
  Serial.println("-----------+---------+-----------+----------+--------+--------+-----+---------");
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
  server.handleClient();

  // PHASE 1: IMU — 32 samples at 1kHz
  for (int i = 0; i < IMU_SAMPLES; i++) {
    unsigned long t0 = micros();
    int16_t ax,ay,az,gx,gy,gz;
    mpu.getAcceleration(&ax,&ay,&az);
    mpu.getRotation(&gx,&gy,&gz);
    float fax=ax/ACCEL_SCALE, fay=ay/ACCEL_SCALE, faz=(az/ACCEL_SCALE)-1.0f;
    float fgx=gx/GYRO_SCALE,  fgy=gy/GYRO_SCALE,  fgz=gz/GYRO_SCALE;
    imuMagBuf[i]  = sqrtf(fax*fax+fay*fay+faz*faz);
    gyroMagBuf[i] = sqrtf(fgx*fgx+fgy*fgy+fgz*fgz);
    imuReal[i]    = imuMagBuf[i];
    imuImag[i]    = 0.0f;
    while ((micros()-t0) < IMU_SAMPLE_US);
  }

  // PHASE 2: Audio — zero-latency DMA flush
  // Block until at least one full buffer is ready, then drain any backlogged
  // frames so Phase 3 always operates on the freshest 16ms audio window.
  // Without this, a slow loop iteration (web traffic, WiFi) lets the DMA
  // queue build up; eventually a misaligned 32-bit frame corrupts i2sRaw and
  // corrScore goes erratic. The flush costs ~zero time when caught up.
  size_t bytes_read = 0;
  i2s_read(I2S_NUM_0, i2sRaw, sizeof(i2sRaw), &bytes_read, portMAX_DELAY);
  {
    size_t flushed = 0;
    while (true) {
      // 0-tick timeout: returns immediately if no more data in the queue
      i2s_read(I2S_NUM_0, i2sRaw, sizeof(i2sRaw), &flushed, 0);
      if (flushed == 0) break;  // queue empty — i2sRaw holds the freshest frame
    }
  }
  if (bytes_read == 0) return;

  // PHASE 3: DC removal
  float dcMean = 0;
  for (int i=0;i<AUDIO_SAMPLES;i++) dcMean += (float)(i2sRaw[i]>>14);
  dcMean /= AUDIO_SAMPLES;
  for (int i=0;i<AUDIO_SAMPLES;i++) { audioReal[i]=(float)(i2sRaw[i]>>14)-dcMean; audioImag[i]=0; }

  // PHASE 4: FFT audio
  audioFFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  audioFFT.compute(FFT_FORWARD);
  audioFFT.complexToMagnitude();

  // PHASE 5: FFT IMU
  imuFFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  imuFFT.compute(FFT_FORWARD);
  imuFFT.complexToMagnitude();

  // PHASE 6: Dominant freq + band energy
  float audioFreqRes = (float)SAMPLE_RATE / AUDIO_SAMPLES;
  float imuFreqRes   = (float)IMU_SAMPLE_HZ / IMU_SAMPLES;
  float peakMag = 0; int peakBin = 1;
  for (int i=1;i<AUDIO_BINS;i++) if(audioReal[i]>peakMag){peakMag=audioReal[i];peakBin=i;}
  float dominantFreq     = peakBin * audioFreqRes;
  float targetBandEnergy = bandEnergy(audioReal, AUDIO_BINS, audioFreqRes, FREQ_LOW, FREQ_HIGH);

  // PHASE 7: Spectral correlation (8 bin-pairs, 62.5–500Hz)
  float audioSpec[CORR_POINTS], imuSpec[CORR_POINTS];
  for (int i=0;i<CORR_POINTS;i++) {
    audioSpec[i] = audioReal[i+1];
    int lo = constrain((int)(((i+1)*audioFreqRes)/imuFreqRes),   0, IMU_BINS-1);
    int hi = constrain((int)(((i+1)*audioFreqRes)/imuFreqRes)+1, 0, IMU_BINS-1);
    imuSpec[i] = (imuReal[lo] + imuReal[hi]) * 0.5f;
  }

  // PHASE 8: Pearson correlation
  float corrScore = pearsonCorr(audioSpec, imuSpec, CORR_POINTS);

  // PHASE 9: Accel RMS + Gyro RMS
  float accelRMS=0, gyroRMS=0;
  for (int i=0;i<IMU_SAMPLES;i++) {
    accelRMS += imuMagBuf[i]  * imuMagBuf[i];
    gyroRMS  += gyroMagBuf[i] * gyroMagBuf[i];
  }
  accelRMS = sqrtf(accelRMS / IMU_SAMPLES);
  gyroRMS  = sqrtf(gyroRMS  / IMU_SAMPLES);

  // PHASE 10: Variance + Jerk
  float accelVariance = pushAndGetVariance(accelRMS);
  float gyroJerk      = computeJerk(gyroRMS);

  // PHASE 11: Classify → debounce (untouched)
  MotionType rawMotion = classifyRaw(accelVariance);
  updateMotionState(rawMotion);
  float       suppress = suppressionWeight(reportedMotion);
  const char* mName    = motionName(reportedMotion);

  calibRecord(accelRMS, gyroRMS, accelVariance);

  // PHASE 12: Gate decision
  //
  // selfNoise: full-spectrum corrScore gate — only active when MOVING.
  //   When STILL, this gate is skipped entirely. "motion gated" can never
  //   appear while the hand is classified as still — every alert candidate
  //   passes through as a full ALERT.
  //
  // alertCandidate: freq + energy gate — dashboard sliders ONLY affect this.
  //
  // strongExternal: energy override — bypasses selfNoise gate when band
  //   energy is >> BAND_ENERGY_MIN. Loud alarms / doorbells clear this bar;
  //   taps and rustles do not.
  //
  // Priority: still → no gate. moving → strongExternal beats selfNoise.

  bool selfNoise      = (corrScore > CORR_THRESHOLD)
                        && (reportedMotion == MOVING);  // never gate when STILL
  bool alertCandidate = (dominantFreq >= FREQ_LOW && dominantFreq <= FREQ_HIGH)
                        && (targetBandEnergy > BAND_ENERGY_MIN);
  bool strongExternal = alertCandidate
                        && (targetBandEnergy > BAND_ENERGY_MIN * ENERGY_OVERRIDE_MULT);

  int hapticPWM = 0;
  const char* dec = "silent";

  if (selfNoise && !strongExternal) {
    // Self-generated noise detected AND no dominant external signal —
    // suppress motor output entirely.
    hapticPWM = 0;
    dec = "motion gated";
  } else if (alertCandidate) {
    // Either no self-noise, OR strongExternal overrode the gate.
    // Fire haptic scaled by motion suppression weight.
    int rawPWM = (int)map((long)targetBandEnergy,(long)BAND_ENERGY_MIN,200000L,HAPTIC_MIN_PWM,HAPTIC_MAX_PWM);
    rawPWM    = constrain(rawPWM, HAPTIC_MIN_PWM, HAPTIC_MAX_PWM);
    hapticPWM = constrain((int)(rawPWM*(1.0f-suppress)), 0, HAPTIC_MAX_PWM);
    if (reportedMotion == STILL) {
      dec = "ALERT";
      logAlert(dominantFreq,targetBandEnergy,accelRMS,gyroRMS,corrScore,suppress,mName,"ALERT");
    } else {
      dec = "ALERT (low conf)";
      logAlert(dominantFreq,targetBandEnergy,accelRMS,gyroRMS,corrScore,suppress,mName,"ALERT (low conf)");
    }
  } else {
    // No self-noise, no alert candidate
    hapticPWM = 0;
    if      (dominantFreq < FREQ_LOW || dominantFreq > FREQ_HIGH) dec = "out of range";
    else if (targetBandEnergy <= BAND_ENERGY_MIN)                  dec = "below energy";
    else                                                            dec = "silent";
  }

  setHaptic(hapticPWM);

  t_dominantFreq = dominantFreq; t_bandEnergy = targetBandEnergy;
  t_imuRMS       = accelRMS;     t_gyroRMS    = gyroRMS;
  t_accelVar     = accelVariance; t_gyroJerk   = gyroJerk;
  t_corrScore    = corrScore;     t_hapticPWM  = hapticPWM;
  strncpy((char*)t_decision,   dec,   sizeof(t_decision)-1);
  strncpy((char*)t_motionType, mName, sizeof(t_motionType)-1);

  Serial.printf("%7.1f Hz | %7.0f | %.7f | %8.2f | %-6s | %6.3f | %3d | [%s]%s\n",
                dominantFreq, targetBandEnergy, accelVariance, gyroJerk,
                mName, corrScore, hapticPWM, dec,
                calibLogging ? " [REC]" : "");
}
