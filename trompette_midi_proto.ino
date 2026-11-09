/**
 * ============================================================================
 * PROJET : TROMPETTE MIDI AUGMENTÉE (Phase 1 - Prototype Monolithique)
 * CIBLE  : Seeed Studio XIAO ESP32-S3 Sense (Core Arduino ESP32 v3.x / IDF v5)
 * VERSION: v1.5.1-proto (Avec lissage anti-artefacts)
 * ============================================================================
 */

#include <driver/i2s_pdm.h>
#include <math.h>
#include <USB.h>
#include <USBMIDI.h>
#include "config.h"
#include "soc/rtc_cntl_reg.h"

#if ARDUINO_USB_MODE
#error "USB Mode doit être 'USB-OTG (TinyUSB)' (Outils > USB Mode). 'Hardware CDC and JTAG' désactive l'USB MIDI."
#endif

#if ARDUINO_USB_CDC_ON_BOOT
#define Console Serial
#else
USBCDC Console;
#endif

// --- VARIABLES PARTAGÉES INTER-CŒURS (Cœur 0 -> Cœur 1) ---
volatile float shared_raw_freq = 0.0f;
volatile float shared_peak_amp = 0.0f;
volatile float shared_rms_amp = 0.0f;
volatile float shared_yin_quality = 1.0f;
volatile bool isBassMode = true;
volatile uint32_t shared_frame_id = 0;

// --- RESSOURCES CŒUR 0 (DSP) ---
TaskHandle_t AudioTask;
int16_t samples[BUFFER_SIZE];
i2s_chan_handle_t rx_handle;

// --- RESSOURCES CŒUR 1 (Logique Musicale & MIDI) ---
USBMIDI USBMidi;

int currentMidiNote = 0;
bool noteIsPlaying = false;
int newNoteCounter = 0;
int targetNewNote = 0;

float last_peak = 0.0f;
float last_yin_quality = 1.0f;
unsigned int attackCounter = 0;
unsigned int lastSentAttack = 0;
bool isSlur = false;
bool isMidiSent = false;
int lastSentNote = 0;
int lastSentPitchBend = 8192;

// --- NOUVEAU : LISSAGE & FILTRAGE ARTEFACTS (CŒUR 1) ---
float freq_history[3] = {0.0f, 0.0f, 0.0f};
int freq_idx = 0;
float last_good_freq = 0.0f;
float smoothed_pb = 8192.0f;

// --- VÉLOCITÉ & EXPRESSION ---
int currentVelocity = 100;
bool velocityPending = false;
float attackPeakMax = 0.0f;
uint32_t attackStartFrame = 0;
float expressionEnv = 0.0f;
int lastSentExpression = -1;
uint32_t lastExprUpdateUs = 0;
uint32_t lastExprSendMs = 0;
bool velocityDebug = false;
float debugPeakMax = 0.0f;
float debugRmsMax = 0.0f;

// --- RESSOURCES BOUTON 1 & TRACK AND HOLD ---
enum Button1Mode {
  BTN_MODE_TRIGGER_C0,
  BTN_MODE_TRACK_HOLD
};

Button1Mode currentBtn1Mode = BTN_MODE_TRIGGER_C0;
int btn1Integrator = 0;
bool btn1StablePressed = false;
bool btn1NoteActive = false;
int heldBassNote = MIDI_NOTE_C0;

// --- FONCTIONS UTILITAIRES ---
float midiToFreq(int midiNote) {
  return 440.0f * powf(2.0f, (midiNote - 69.0f) / 12.0f);
}

int peakToMidi(float peak, int outMin, int outMax) {
  if (peak <= NOISE_GATE_PEAK) return outMin;
  float db = 20.0f * log10f(peak / NOISE_GATE_PEAK);
  float dbRange = 20.0f * log10f(VELOCITY_PEAK_MAX / NOISE_GATE_PEAK);
  float norm = db / dbRange;
  if (norm > 1.0f) norm = 1.0f;
  if (norm < 0.0f) norm = 0.0f;
  norm = powf(norm, VELOCITY_CURVE);
  return outMin + (int)lroundf(norm * (outMax - outMin));
}

void beginVelocityCapture(float peak) {
  velocityPending = true;
  attackPeakMax = peak;
  attackStartFrame = shared_frame_id;
}

float rmsToExpression(float rms) {
  if (rms <= EXPRESSION_RMS_FLOOR) return 0.0f;
  float norm = log10f(rms / EXPRESSION_RMS_FLOOR) / log10f(EXPRESSION_RMS_MAX / EXPRESSION_RMS_FLOOR);
  if (norm > 1.0f) norm = 1.0f;
  norm = powf(norm, EXPRESSION_CURVE);
  return norm * 127.0f;
}

void sendExpression(float env) {
#if EXPRESSION_14BIT
  int v14 = (int)lroundf(env / 127.0f * 16383.0f);
  if (v14 < 0) v14 = 0;
  if (v14 > 16383) v14 = 16383;
  USBMidi.controlChange(EXPRESSION_CC, v14 >> 7, MIDI_CH_PITCH);
  USBMidi.controlChange(EXPRESSION_CC + 32, v14 & 0x7F, MIDI_CH_PITCH);
  lastSentExpression = v14;
#else
  int v = (int)lroundf(env);
  if (v < 0) v = 0;
  if (v > 127) v = 127;
  USBMidi.controlChange(EXPRESSION_CC, v, MIDI_CH_PITCH);
  lastSentExpression = v;
#endif
  lastExprSendMs = millis();
}

int expressionToSendUnits(float env) {
#if EXPRESSION_14BIT
  return (int)lroundf(env / 127.0f * 16383.0f);
#else
  return (int)lroundf(env);
#endif
}

// ============================================================================
// PRODUCTEUR : TÂCHE CŒUR 0 (Acquisition I2S PDM & Algorithme YIN)
// ============================================================================
void audio_pitch_task(void *pvParameters) {
  size_t bytesIn = 0;
  const int min_period = SAMPLE_RATE / MAX_FREQ_HZ;

  while (true) {
    esp_err_t result = i2s_channel_read(rx_handle, &samples, sizeof(samples), &bytesIn, portMAX_DELAY);

    if (result == ESP_OK && bytesIn > 0) {
      int numSamples = bytesIn / sizeof(int16_t);

      int activeSamples = isBassMode ? numSamples : (numSamples / 2);
      int startIndex = numSamples - activeSamples;

      int min_freq = isBassMode ? MIN_FREQ_BASS_HZ : MIN_FREQ_LEAD_HZ;
      int max_period = SAMPLE_RATE / min_freq;
      int window_size = activeSamples - max_period;

      if (window_size <= 0) {
        vTaskDelay(1);
        continue;
      }

      float mean = 0.0f;
      for (int i = startIndex; i < numSamples; i++) {
        mean += samples[i];
      }
      mean /= activeSamples;

      float peak = 0.0f;
      float sum_sq = 0.0f;
      for (int i = startIndex; i < numSamples; i++) {
        float ac = (float)samples[i] - mean;
        float ac_val = fabsf(ac);
        if (ac_val > peak) peak = ac_val;
        sum_sq += ac * ac;
      }
      shared_peak_amp = peak;
      shared_rms_amp = sqrtf(sum_sq / activeSamples);

      if (peak > NOISE_GATE_PEAK) {
        float yin_buffer[max_period + 1];
        yin_buffer[0] = 1.0f;

        for (int tau = 1; tau <= max_period; tau++) {
          float diff_sum = 0.0f;
          for (int i = 0; i < window_size; i++) {
            int actual_i = startIndex + i;
            float delta = ((float)samples[actual_i] - mean) - ((float)samples[actual_i + tau] - mean);
            diff_sum += (delta * delta);
          }
          yin_buffer[tau] = diff_sum / 100000000.0f;
        }

        float running_sum = 0.0f;
        for (int tau = 1; tau <= max_period; tau++) {
          running_sum += yin_buffer[tau];
          if (running_sum == 0.0f) {
            yin_buffer[tau] = 1.0f;
          } else {
            yin_buffer[tau] = yin_buffer[tau] * tau / running_sum;
          }
        }

        int best_tau = -1;
        float current_yin_quality = 1.0f;

        for (int tau = min_period; tau <= max_period; tau++) {
          if (yin_buffer[tau] < YIN_THRESHOLD) {
            current_yin_quality = yin_buffer[tau];
            while (tau + 1 <= max_period && yin_buffer[tau + 1] < yin_buffer[tau]) {
              tau++;
            }
            best_tau = tau;
            break;
          }
        }

        if (best_tau == -1) {
          float min_val = 1.0f;
          for (int tau = min_period; tau <= max_period; tau++) {
            if (yin_buffer[tau] < min_val) {
              min_val = yin_buffer[tau];
              best_tau = tau;
            }
          }
          current_yin_quality = min_val;
          if (min_val > YIN_FALLBACK_MAX) best_tau = -1;
        }

        shared_yin_quality = current_yin_quality;

        if (best_tau > 0) {
          float refined_period = (float)best_tau;
          if (best_tau > min_period && best_tau < max_period) {
            float s0 = yin_buffer[best_tau - 1];
            float s1 = yin_buffer[best_tau];
            float s2 = yin_buffer[best_tau + 1];
            float denom = (s0 - 2.0f * s1 + s2);
            if (fabsf(denom) > 0.00001f) {
              float shift = 0.5f * (s0 - s2) / denom;
              refined_period = best_tau + shift;
            }
          }
          shared_raw_freq = ((float)SAMPLE_RATE / refined_period) * PITCH_CALIBRATION;
        } else {
          shared_raw_freq = 0.0f;
        }
      } else {
        shared_raw_freq = 0.0f;
        shared_yin_quality = 1.0f;
      }
      shared_frame_id++;
    }
  }
}

// ============================================================================
// INITIALISATION (SETUP)
// ============================================================================
void setup() {
  pinMode(BUTTON_1_PIN, INPUT_PULLUP);
  pinMode(BUTTON_2_PIN, INPUT_PULLUP);

  USBMidi.begin();
  Console.begin(115200); 
  USB.begin();

  delay(2500);
  Console.println("====================================================");
  Console.printf("TROMPETTE MIDI_ - %s (XIAO ESP32-S3 Sense)\n", FW_VERSION);
  Console.println("Commandes Série : 'B'=Mode BASS | 'L'=Mode LEAD | 'T'=Toggle Bouton | 'V'=Debug vélocité | 'U'=Mode flash");
  Console.println("====================================================");

  i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
  chan_cfg.dma_frame_num = DMA_FRAME_NUM;

  esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  if (err != ESP_OK) {
    Console.println("[ERREUR] Échec de création du canal I2S");
  }

  i2s_pdm_rx_config_t pdm_rx_cfg = {
    .clk_cfg = I2S_PDM_RX_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
    .slot_cfg = I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .clk = (gpio_num_t)I2S_CLK_PIN,
      .din = (gpio_num_t)I2S_DAT_PIN,
      .invert_flags = { .clk_inv = false },
    },
  };

  i2s_channel_init_pdm_rx_mode(rx_handle, &pdm_rx_cfg);
  err = i2s_channel_enable(rx_handle);
  if (err == ESP_OK) {
    Console.println("[OK] Microphone PDM I2S initialisé.");
  }

  xTaskCreatePinnedToCore(
    audio_pitch_task,
    "AudioPitchTask",
    16384,
    NULL,
    1,
    &AudioTask,
    0
  );
}

// ============================================================================
// CONSOMMATEUR : CŒUR 1 (Logique Musicale & MIDI)
// ============================================================================
void loop() {
  // 1. Commandes de contrôle via le Moniteur Série
  if (Console.available() > 0) {
    char cmd = Console.read();
    if (cmd == 'b' || cmd == 'B') {
      isBassMode = true;
      Console.println("[MODE] BASS activé");
    } else if (cmd == 'l' || cmd == 'L') {
      isBassMode = false;
      Console.println("[MODE] LEAD activé");
    } else if (cmd == 't' || cmd == 'T') {
      if (currentBtn1Mode == BTN_MODE_TRIGGER_C0) {
        currentBtn1Mode = BTN_MODE_TRACK_HOLD;
        Console.println("[BOUTON 1] Mode TRACK & HOLD");
      } else {
        currentBtn1Mode = BTN_MODE_TRIGGER_C0;
        Console.println("[BOUTON 1] Mode TRIGGER C0");
      }
    } else if (cmd == 'u' || cmd == 'U') {
      Console.println("[BOOT] Redemarrage en mode telechargement...");
      Console.flush();
      delay(100);
      REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
      esp_restart();
    } else if (cmd == 'v' || cmd == 'V') {
      velocityDebug = !velocityDebug;
      debugPeakMax = 0.0f;
      debugRmsMax = 0.0f;
      Console.printf("[VELOCITE] Debug %s\n", velocityDebug ? "ON" : "OFF");
    }
  }

  // 2. Lecture atomique des variables produites par le Cœur 0
  float raw_f = shared_raw_freq;
  float peak = shared_peak_amp;
  float rms = shared_rms_amp;
  float yin_qual = shared_yin_quality;
  uint32_t frame = shared_frame_id;

  if (velocityDebug && peak > debugPeakMax) {
    debugPeakMax = peak;
    Console.printf("[VELOCITE] Nouvelle crête max = %.0f\n", debugPeakMax);
  }
  if (velocityDebug && rms > debugRmsMax) {
    debugRmsMax = rms;
    Console.printf("[EXPRESSION] Nouveau RMS max = %.0f\n", debugRmsMax);
  }

  // 3. Détection des transitoires d'attaque vs Liaison
  bool hard_attack = false;
  if (peak > NOISE_GATE_PEAK && peak > (last_peak * 1.5f)) {
    hard_attack = true;
  }
  last_peak = peak;

  bool periodic_transient = false;
  float legato_threshold = isBassMode ? 0.35f : 0.25f;
  if (yin_qual > legato_threshold && last_yin_quality < YIN_THRESHOLD) {
    periodic_transient = true;
  }
  last_yin_quality = yin_qual;

  // --- 3bis. LISSAGE DE LA FRÉQUENCE ET REJET DES ARTEFACTS ---
  float freq = 0.0f;
  if (raw_f > 0.0f) {
    // Filtre médian sur 3 échantillons (Rejette les pics isolés)
    freq_history[freq_idx] = raw_f;
    freq_idx = (freq_idx + 1) % 3;
    float f1 = freq_history[0], f2 = freq_history[1], f3 = freq_history[2];
    float median_freq = max(min(f1,f2), min(max(f1,f2),f3));

    // Gate de confiance YIN : Ignore la médiane si le son est trop instable
    // (sauf lors d'une attaque franche où la turbulence est normale)
    if (yin_qual > 0.25f && !hard_attack) {
      freq = last_good_freq;
    } else {
      freq = median_freq;
      last_good_freq = freq;
    }
  } else {
    // Réinitialisation du buffer au silence pour éviter les artefacts d'anciennes notes
    freq_history[0] = 0.0f; freq_history[1] = 0.0f; freq_history[2] = 0.0f;
    last_good_freq = 0.0f;
  }

  // 4. Calcul de la Note MIDI, Hystérésis & Pitch Bend
  int pb_val = 8192;

  if (freq > 0.0f && peak > NOISE_GATE_PEAK) {
    float exact_midi_float = 69.0f + 12.0f * log2f(freq / 440.0f);
    int proposedMidiNote = round(exact_midi_float);

    if (noteIsPlaying) {
      float theoretical_freq = midiToFreq(currentMidiNote);
      float deviation_semitones = 12.0f * log2f(freq / theoretical_freq);

      int target_pb = 8192 + (int)((deviation_semitones / PITCHBEND_RANGE) * 8192.0f);
      if (target_pb > 16383) target_pb = 16383;
      if (target_pb < 0) target_pb = 0;

      // Lissage exponentiel du Pitch Bend pour des transitions vocales/organiques
      smoothed_pb += 0.2f * ((float)target_pb - smoothed_pb);
      pb_val = (int)smoothed_pb;

      float diff = fabsf(exact_midi_float - (float)currentMidiNote);
      float hysteresis_limit = isBassMode ? 1.0f : 0.7f;

      if (hard_attack && diff <= hysteresis_limit) {
        isSlur = false;
        attackCounter++;
        beginVelocityCapture(peak);
      }

      if (diff > hysteresis_limit) {
        if (proposedMidiNote == targetNewNote) {
          newNoteCounter++;
        } else {
          targetNewNote = proposedMidiNote;
          newNoteCounter = 1;
        }

        int debounce_limit = isBassMode ? 3 : 2;
        if (newNoteCounter >= debounce_limit) {
          currentMidiNote = targetNewNote;
          newNoteCounter = 0;

          if (periodic_transient && !hard_attack) {
            isSlur = true;
          } else {
            isSlur = false;
            beginVelocityCapture(peak);
          }
          attackCounter++;
          
          pb_val = 8192;
          smoothed_pb = 8192.0f; // Reset au changement de note
        }
      } else {
        newNoteCounter = 0;
      }
    } else {
      // Première attaque
      currentMidiNote = proposedMidiNote;
      noteIsPlaying = true;
      newNoteCounter = 0;
      isSlur = false;
      attackCounter++;
      beginVelocityCapture(peak);
      
      pb_val = 8192;
      smoothed_pb = 8192.0f; // Reset
    }
  } else {
    if (noteIsPlaying) {
      noteIsPlaying = false;
    }
  }

  // 5. Envoi des événements MIDI (Canal 1)
  bool play = noteIsPlaying;
  int note = currentMidiNote;

  if (velocityPending) {
    if (!play) {
      velocityPending = false;
    } else {
      if (peak > attackPeakMax) attackPeakMax = peak;
      if ((uint32_t)(frame - attackStartFrame) >= VELOCITY_ATTACK_FRAMES) {
        currentVelocity = peakToMidi(attackPeakMax, VELOCITY_MIN, VELOCITY_MAX);
        velocityPending = false;
        if (velocityDebug) {
          Console.printf("[VELOCITE] Note %d | crete %.0f -> velocite %d\n", note, attackPeakMax, currentVelocity);
        }
      }
    }
  }

  if (play && !velocityPending && (!isMidiSent || note != lastSentNote || attackCounter != lastSentAttack)) {
    if (isMidiSent) {
      if (isSlur && note != lastSentNote) {
        USBMidi.noteOn(note, currentVelocity, MIDI_CH_PITCH);
        USBMidi.noteOff(lastSentNote, 0, MIDI_CH_PITCH);
      } else {
        USBMidi.noteOff(lastSentNote, 0, MIDI_CH_PITCH);
        USBMidi.noteOn(note, currentVelocity, MIDI_CH_PITCH);
      }
    } else {
#if EXPRESSION_ENABLED
      expressionEnv = rmsToExpression(rms);
      sendExpression(expressionEnv);
#endif
      USBMidi.noteOn(note, currentVelocity, MIDI_CH_PITCH);
    }

    lastSentNote = note;
    lastSentAttack = attackCounter;
    isMidiSent = true;
  } else if (!play && isMidiSent) {
    USBMidi.noteOff(lastSentNote, 0, MIDI_CH_PITCH);
    USBMidi.pitchBend((uint16_t)8192, MIDI_CH_PITCH);
    lastSentPitchBend = 8192;
    smoothed_pb = 8192.0f; // Remise à plat du lissage en fin de note
    isMidiSent = false;
  }

#if EXPRESSION_ENABLED
  {
    uint32_t nowUs = micros();
    float dt_ms = (nowUs - lastExprUpdateUs) / 1000.0f;
    lastExprUpdateUs = nowUs;
    if (dt_ms > 50.0f) dt_ms = 50.0f;

    if (play && isMidiSent) {
      float target = rmsToExpression(rms);
      float tau = (target > expressionEnv) ? EXPRESSION_ATTACK_MS : EXPRESSION_RELEASE_MS;
      expressionEnv += (1.0f - expf(-dt_ms / tau)) * (target - expressionEnv);

      int units = expressionToSendUnits(expressionEnv);
      if (abs(units - lastSentExpression) >= EXPRESSION_DEADBAND &&
          (millis() - lastExprSendMs) >= EXPRESSION_MIN_INTERVAL_MS) {
        sendExpression(expressionEnv);
      }
    }
  }
#endif

  // Envoi du Pitch Bend continu
  if (play && isMidiSent) {
    if (abs(pb_val - lastSentPitchBend) > PITCHBEND_DEADBAND) {
      USBMidi.pitchBend((uint16_t)pb_val, MIDI_CH_PITCH);
      lastSentPitchBend = pb_val;
    }
  }

  // 6. Lecture du Bouton 1 avec Intégrateur
  int rawBtn1 = digitalRead(BUTTON_1_PIN);
  if (rawBtn1 == LOW) {
    if (btn1Integrator < EMI_INTEGRATOR_MAX) {
      btn1Integrator++;
    }
  } else {
    if (btn1Integrator > 0) {
      btn1Integrator--;
    }
  }

  if (btn1Integrator >= EMI_INTEGRATOR_MAX && !btn1StablePressed) {
    btn1StablePressed = true;

    if (currentBtn1Mode == BTN_MODE_TRACK_HOLD && noteIsPlaying) {
      heldBassNote = currentMidiNote;
    } else {
      heldBassNote = MIDI_NOTE_C0;
    }

    int btnVelocity = (currentBtn1Mode == BTN_MODE_TRACK_HOLD && noteIsPlaying) ? currentVelocity : 100;
    USBMidi.noteOn(heldBassNote, btnVelocity, MIDI_CH_BUTTON_1);
    btn1NoteActive = true;
    Console.printf("[CH %d] Bouton ON -> Note %d (vel %d)\n", MIDI_CH_BUTTON_1, heldBassNote, btnVelocity);
  } else if (btn1Integrator == 0 && btn1StablePressed) {
    btn1StablePressed = false;

    if (btn1NoteActive) {
      USBMidi.noteOff(heldBassNote, 0, MIDI_CH_BUTTON_1);
      btn1NoteActive = false;
      Console.printf("[CH %d] Bouton OFF -> Note %d\n", MIDI_CH_BUTTON_1, heldBassNote);
    }
  }

  vTaskDelay(2);
}
