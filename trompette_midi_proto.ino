/**
 * ============================================================================
 * PROJET : TROMPETTE MIDI AUGMENTÉE (Phase 1 - Prototype Monolithique)
 * CIBLE  : Seeed Studio XIAO ESP32-S3 Sense (Core Arduino ESP32 v3.x / IDF v5)
 * VERSION: v1.5.1-proto
 * ============================================================================
 * ARCHITECTURE INTER-CŒURS (Producteur - Consommateur) :
 * - CŒUR 0 (Producteur DSP) : Acquisition I2S PDM via DMA + Algorithme YIN
 *   100% en virgule flottante (float) + Fenêtrage dynamique (Mode Lead/Bass).
 * - CŒUR 1 (Consommateur Musical) : Machine à états MIDI, Hystérésis,
 *   Détection Legato par Transitoire de Périodicité, Pitch Bend continu (Ch 1),
 *   Boutons avec intégrateur numérique anti-EMI (Ch 2) et Mode Track & Hold.
 *   Vélocité dynamique (dB de la crête d'attaque) + Expression continue CC11
 *   (enveloppe RMS lissée à ~500 Hz, attaque/relâchement séparés).
 *
 * CONFIGURATION ARDUINO IDE OBLIGATOIRE :
 * - Board            : "Seeed Studio XIAO ESP32S3"
 * - USB CDC On Boot  : "Enabled"
 * - USB Mode         : "USB-OTG (TinyUSB)"
 * ============================================================================
 */

#include <driver/i2s_pdm.h>
#include <math.h>
#include <USB.h>
#include <USBMIDI.h>
#include "config.h"
#include "soc/rtc_cntl_reg.h"  // Commande 'U' : reboot en mode téléchargement

#if ARDUINO_USB_MODE
#error "USB Mode doit être 'USB-OTG (TinyUSB)' (Outils > USB Mode). 'Hardware CDC and JTAG' désactive l'USB MIDI."
#endif

// --- CONSOLE SÉRIE USB (indépendante du réglage "USB CDC On Boot") ---
// Si "USB CDC On Boot" est désactivé dans l'IDE, `Serial` pointe vers l'UART0
// (broche TX) et aucun port /dev/ttyACM n'apparaît. On crée alors nous-mêmes
// l'interface CDC : son constructeur global l'enregistre AVANT USB.begin().
#if ARDUINO_USB_CDC_ON_BOOT
#define Console Serial
#else
USBCDC Console;
#endif

// --- VARIABLES PARTAGÉES INTER-CŒURS (Cœur 0 -> Cœur 1) ---
volatile float shared_raw_freq = 0.0f;
volatile float shared_peak_amp = 0.0f;
volatile float shared_rms_amp = 0.0f;   // Niveau efficace (RMS) : bien plus stable que la crête
volatile float shared_yin_quality = 1.0f;
volatile bool isBassMode = true; // Actif par défaut pour garantir les sous-harmoniques
volatile uint32_t shared_frame_id = 0; // Incrémenté à chaque trame DSP analysée (32 ms)

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

// --- VÉLOCITÉ & EXPRESSION ---
int currentVelocity = 100;       // Vélocité de la dernière attaque (réutilisée en legato)
bool velocityPending = false;    // Attaque détectée, NoteOn retenu le temps de capter le pic
float attackPeakMax = 0.0f;      // Crête max observée pendant la fenêtre d'attaque
uint32_t attackStartFrame = 0;
float expressionEnv = 0.0f;      // Enveloppe de souffle lissée (0..127, flottant)
int lastSentExpression = -1;     // Dernière valeur envoyée (7 bits, ou 14 bits si activé)
uint32_t lastExprUpdateUs = 0;
uint32_t lastExprSendMs = 0;
bool velocityDebug = false;      // Commande série 'V' : log de calibration
float debugPeakMax = 0.0f;       // Plus forte crête vue depuis l'activation du debug
float debugRmsMax = 0.0f;        // Plus fort RMS vu depuis l'activation du debug

// --- RESSOURCES BOUTON 1 (D0 / GPIO 1) & TRACK AND HOLD ---
enum Button1Mode {
  BTN_MODE_TRIGGER_C0,   // Envoie la note C0 sur le Canal 2
  BTN_MODE_TRACK_HOLD    // Verrouille la note actuelle de la trompette en drone sur le Canal 2
};

Button1Mode currentBtn1Mode = BTN_MODE_TRIGGER_C0;
int btn1Integrator = 0;       // Compteur d'intégration numérique anti-EMI (0 à EMI_INTEGRATOR_MAX)
bool btn1StablePressed = false;
bool btn1NoteActive = false;
int heldBassNote = MIDI_NOTE_C0;

// --- FONCTIONS UTILITAIRES ---
float midiToFreq(int midiNote) {
  return 440.0f * powf(2.0f, (midiNote - 69.0f) / 12.0f);
}

// Convertit une amplitude crête en valeur MIDI [outMin..outMax] sur une échelle
// logarithmique : l'oreille (et le souffle) perçoivent les nuances en dB.
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

// Ouvre la fenêtre de capture de vélocité (coup de langue / attaque depuis le silence)
void beginVelocityCapture(float peak) {
  velocityPending = true;
  attackPeakMax = peak;
  attackStartFrame = shared_frame_id;
}

// Convertit le niveau RMS en expression continue 0..127 (flottant, échelle dB)
float rmsToExpression(float rms) {
  if (rms <= EXPRESSION_RMS_FLOOR) return 0.0f;
  float norm = log10f(rms / EXPRESSION_RMS_FLOOR) / log10f(EXPRESSION_RMS_MAX / EXPRESSION_RMS_FLOOR);
  if (norm > 1.0f) norm = 1.0f;
  norm = powf(norm, EXPRESSION_CURVE);
  return norm * 127.0f;
}

// Envoie l'expression (7 bits, ou 14 bits MSB CC11 + LSB CC43)
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

      // Fenêtrage logiciel dynamique selon le mode (BASS = 100%, LEAD = 50% plus récents)
      int activeSamples = isBassMode ? numSamples : (numSamples / 2);
      int startIndex = numSamples - activeSamples;

      int min_freq = isBassMode ? MIN_FREQ_BASS_HZ : MIN_FREQ_LEAD_HZ;
      int max_period = SAMPLE_RATE / min_freq;
      int window_size = activeSamples - max_period;

      if (window_size <= 0) {
        vTaskDelay(1);
        continue;
      }

      // 1. Calcul de l'Offset DC (Moyenne)
      float mean = 0.0f;
      for (int i = startIndex; i < numSamples; i++) {
        mean += samples[i];
      }
      mean /= activeSamples;

      // 2. Mesure de l'amplitude crête (Peak) et du niveau efficace (RMS)
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

      // 3. Algorithme YIN (Uniquement si le signal dépasse le Noise Gate)
      if (peak > NOISE_GATE_PEAK) {
        float yin_buffer[max_period + 1];
        yin_buffer[0] = 1.0f;

        // Étape 3a : Fonction de différence en virgule flottante (zéro risque d'overflow)
        for (int tau = 1; tau <= max_period; tau++) {
          float diff_sum = 0.0f;
          for (int i = 0; i < window_size; i++) {
            int actual_i = startIndex + i;
            float delta = ((float)samples[actual_i] - mean) - ((float)samples[actual_i + tau] - mean);
            diff_sum += (delta * delta);
          }
          yin_buffer[tau] = diff_sum / 100000000.0f;
        }

        // Étape 3b : Différence cumulée normalisée (CMND)
        float running_sum = 0.0f;
        for (int tau = 1; tau <= max_period; tau++) {
          running_sum += yin_buffer[tau];
          if (running_sum == 0.0f) {
            yin_buffer[tau] = 1.0f;
          } else {
            yin_buffer[tau] = yin_buffer[tau] * tau / running_sum;
          }
        }

        // Étape 3c : Seuil absolu (Premier creux sous YIN_THRESHOLD)
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

        // Recherche du minimum global de secours si aucun creux n'est sous 0.15
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

        // Étape 3d : Interpolation parabolique
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
  Console.begin(115200);  // Avant USB.begin() : l'interface CDC fait partie du descripteur
  USB.begin();

  delay(2500); // Laisse le temps à Fedora de monter le port /dev/ttyACM
  Console.println("====================================================");
  Console.printf("TROMPETTE MIDI_ - %s (XIAO ESP32-S3 Sense)\n", FW_VERSION);
  Console.println("Commandes Série : 'B'=Mode BASS | 'L'=Mode LEAD | 'T'=Toggle Bouton (C0 / Track&Hold) | 'V'=Debug vélocité | 'U'=Mode flash");
  Console.println("====================================================");

  // Configuration I2S PDM (API ESP-IDF v5)
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

  // Lancement du moteur DSP sur le Cœur 0
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
// CONSOMMATEUR : CŒUR 1 (Logique Musicale, Boutons Anti-EMI & USB MIDI)
// ============================================================================
void loop() {
  // 1. Commandes de contrôle via le Moniteur Série
  if (Console.available() > 0) {
    char cmd = Console.read();
    if (cmd == 'b' || cmd == 'B') {
      isBassMode = true;
      Console.println("[MODE] BASS activé (Fenêtre 512, Min 60Hz)");
    } else if (cmd == 'l' || cmd == 'L') {
      isBassMode = false;
      Console.println("[MODE] LEAD activé (Fenêtre 256, Min 120Hz)");
    } else if (cmd == 't' || cmd == 'T') {
      if (currentBtn1Mode == BTN_MODE_TRIGGER_C0) {
        currentBtn1Mode = BTN_MODE_TRACK_HOLD;
        Console.println("[BOUTON 1] Mode TRACK & HOLD (Verrouille la note jouée sur Canal 2)");
      } else {
        currentBtn1Mode = BTN_MODE_TRIGGER_C0;
        Console.println("[BOUTON 1] Mode TRIGGER C0 (Joue C0 sur Canal 2)");
      }
    } else if (cmd == 'u' || cmd == 'U') {
      // Redémarre directement en mode téléchargement (équivalent bouton BOOT)
      Console.println("[BOOT] Redemarrage en mode telechargement... (lancer l'upload)");
      Console.flush();
      delay(100);
      REG_WRITE(RTC_CNTL_OPTION1_REG, RTC_CNTL_FORCE_DOWNLOAD_BOOT);
      esp_restart();
    } else if (cmd == 'v' || cmd == 'V') {
      velocityDebug = !velocityDebug;
      debugPeakMax = 0.0f;
      debugRmsMax = 0.0f;
      Console.printf("[VELOCITE] Debug %s (jouer pp puis ff pour calibrer VELOCITY_PEAK_MAX)\n",
                    velocityDebug ? "ON" : "OFF");
    }
  }

  // 2. Lecture atomique des variables produites par le Cœur 0
  float freq = shared_raw_freq;
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

  // 3. Détection des transitoires d'attaque (Coup de langue) vs Liaison (Legato)
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

  // 4. Calcul de la Note MIDI, Hystérésis & Pitch Bend
  int pb_val = 8192;

  if (freq > 0.0f && peak > NOISE_GATE_PEAK) {
    float exact_midi_float = 69.0f + 12.0f * log2f(freq / 440.0f);
    int proposedMidiNote = round(exact_midi_float);

    if (noteIsPlaying) {
      float theoretical_freq = midiToFreq(currentMidiNote);
      float deviation_semitones = 12.0f * log2f(freq / theoretical_freq);

      pb_val = 8192 + (int)((deviation_semitones / PITCHBEND_RANGE) * 8192.0f);
      if (pb_val > 16383) pb_val = 16383;
      if (pb_val < 0) pb_val = 0;

      float diff = fabsf(exact_midi_float - (float)currentMidiNote);
      float hysteresis_limit = isBassMode ? 1.0f : 0.7f;

      // Coup de langue répété sur la même note
      if (hard_attack && diff <= hysteresis_limit) {
        isSlur = false;
        attackCounter++;
        beginVelocityCapture(peak);
      }

      // Changement de note au-delà du seuil d'hystérésis
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
            isSlur = true;  // Legato : on conserve la vélocité courante (pas de nouvelle attaque)
          } else {
            isSlur = false;
            beginVelocityCapture(peak);
          }
          attackCounter++;
          pb_val = 8192;
        }
      } else {
        newNoteCounter = 0;
      }
    } else {
      // Première attaque depuis le silence
      currentMidiNote = proposedMidiNote;
      noteIsPlaying = true;
      newNoteCounter = 0;
      isSlur = false;
      attackCounter++;
      beginVelocityCapture(peak);
      pb_val = 8192;
    }
  } else {
    if (noteIsPlaying) {
      noteIsPlaying = false;
    }
  }

  // 5. Envoi des événements MIDI de la Trompette (Canal 1)
  bool play = noteIsPlaying;
  int note = currentMidiNote;

  // 4b. Résolution de la vélocité : on suit la crête pendant VELOCITY_ATTACK_FRAMES
  //     trames pour capter le vrai sommet de l'attaque (compromis latence / précision).
  if (velocityPending) {
    if (!play) {
      velocityPending = false;  // Attaque avortée (souffle trop court)
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
        // Mode LEGATO : Superposition (Nouveau NoteOn AVANT l'ancien NoteOff)
        USBMidi.noteOn(note, currentVelocity, MIDI_CH_PITCH);
        USBMidi.noteOff(lastSentNote, 0, MIDI_CH_PITCH);
      } else {
        // Mode STACCATO : Séparation franche
        USBMidi.noteOff(lastSentNote, 0, MIDI_CH_PITCH);
        USBMidi.noteOn(note, currentVelocity, MIDI_CH_PITCH);
      }
    } else {
#if EXPRESSION_ENABLED
      // Pré-positionne l'expression AVANT le NoteOn pour que le son démarre au bon niveau
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
    isMidiSent = false;
  }

#if EXPRESSION_ENABLED
  // Expression continue (CC11) : enveloppe calculée à CHAQUE loop (~2 ms) et non
  // plus une fois par trame DSP (32 ms). Le filtre à un pôle glisse vers la cible
  // avec une constante de temps d'attaque (montée) et de relâchement (descente),
  // ce qui transforme les marches d'escalier en rampes continues.
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

  // Envoi du Pitch Bend continu (avec filtre de zone morte pour ne pas saturer l'USB)
  if (play && isMidiSent) {
    if (abs(pb_val - lastSentPitchBend) > PITCHBEND_DEADBAND) {
      USBMidi.pitchBend((uint16_t)pb_val, MIDI_CH_PITCH);
      lastSentPitchBend = pb_val;
    }
  }

  // 6. Lecture du Bouton 1 (D0) avec Intégrateur Numérique Anti-EMI (Canal 2)
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

  // Verrouillage d'état par hystérésis sur l'intégrateur
  if (btn1Integrator >= EMI_INTEGRATOR_MAX && !btn1StablePressed) {
    btn1StablePressed = true;

    if (currentBtn1Mode == BTN_MODE_TRACK_HOLD && noteIsPlaying) {
      heldBassNote = currentMidiNote; // Échantillonne la note actuelle de la trompette
    } else {
      heldBassNote = MIDI_NOTE_C0;
    }

    // En Track & Hold, la basse hérite de la dynamique de la trompette
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

  vTaskDelay(2); // Boucle temps réel cadencée à 2 ms (500 Hz)
}
