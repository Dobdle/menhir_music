#ifndef CONFIG_H
#define CONFIG_H

// ============================================================================
// VERSION DU FIRMWARE
// ============================================================================
#define FW_VERSION "v1.5.1-proto"

// ============================================================================
// BROCHES MATÉRIELLES (Seeed Studio XIAO ESP32-S3 Sense)
// ============================================================================
#define I2S_CLK_PIN       42    // Horloge PDM interne du micro XIAO Sense
#define I2S_DAT_PIN       41    // Données PDM internes du micro XIAO Sense

#define BUTTON_1_PIN      1     // Broche D0 (GPIO 1) - Juste en face de GND
#define BUTTON_2_PIN      2     // Broche D1 (GPIO 2) - Prêt pour le second bouton

// ============================================================================
// PARAMÈTRES AUDIO & DSP (CŒUR 0)
// ============================================================================
#define SAMPLE_RATE       16000
#define DMA_FRAME_NUM     512   // Taille matérielle DMA (512 ech = 32ms @ 16kHz)
#define BUFFER_SIZE       1024

#define PITCH_CALIBRATION 1.0f
#define NOISE_GATE_PEAK   1000.0f
#define YIN_THRESHOLD     0.15f // Seuil absolu de périodicité YIN
#define YIN_FALLBACK_MAX  0.40f // Seuil max toléré en recherche de secours

// Fréquences limites selon le mode (Lead vs Bass)
#define MAX_FREQ_HZ       1200
#define MIN_FREQ_BASS_HZ  60    // Sous-harmoniques graves (Mode BASS)
#define MIN_FREQ_LEAD_HZ  120   // Jeu standard rapide (Mode LEAD)

// ============================================================================
// PARAMÈTRES LOGIQUE MUSICALE & MIDI (CŒUR 1)
// ============================================================================
#define MIDI_CH_PITCH     1     // Canal 1 : Note trompette + Pitch Bend
#define MIDI_CH_BUTTON_1  2     // Canal 2 : Bouton 1 / Basse Track & Hold
#define MIDI_CH_ARP       3     // Canal 3 : Arpégiateur (Backlog)
#define MIDI_CH_CC_MOD    4     // Canal 4 : Modulations accéléromètre & Looper (Backlog)

#define PITCHBEND_RANGE   1.0f  // Plage du Pitch Bend en demi-tons (+/- 2)
#define PITCHBEND_DEADBAND 15   // Zone morte autour de 8192 pour éviter le spam MIDI

#define MIDI_NOTE_C0      12    // Note par défaut envoyée par le Bouton 1 en mode Trigger

// ============================================================================
// VÉLOCITÉ DYNAMIQUE & EXPRESSION (voir EXP-06 dans second_brain)
// ============================================================================
// La vélocité est dérivée de l'amplitude crête au moment de l'attaque, sur une
// échelle logarithmique (dB) entre NOISE_GATE_PEAK (pianissimo) et
// VELOCITY_PEAK_MAX (fortissimo). À calibrer avec la commande série 'V'.
#define VELOCITY_PEAK_MAX   50000.0f // Crête mesurée sur un fortissimo (à calibrer)
#define VELOCITY_MIN        20       // Vélocité plancher (pp juste au-dessus du gate)
#define VELOCITY_MAX        127
#define VELOCITY_CURVE      1.0f     // <1 = plus sensible aux nuances douces, >1 = plus dur
#define VELOCITY_ATTACK_FRAMES 1     // Nb de trames DSP (32 ms) supplémentaires observées
                                     // après l'attaque pour capter le vrai pic (0 = immédiat)

// Expression continue (CC11) sur le Canal 1 : suit le souffle pendant la note
// (crescendo / decrescendo) sans redéclencher l'enveloppe du synthé.
// Basée sur le niveau RMS (stable) et non sur la crête (très instable).
#define EXPRESSION_ENABLED  1
#define EXPRESSION_CC       11
#define EXPRESSION_RMS_FLOOR 2500.0f  // RMS d'un souffle très doux -> expression 0
#define EXPRESSION_RMS_MAX  50000.0f  // RMS d'un ff -> expression 127 (à calibrer avec 'V')
#define EXPRESSION_CURVE    1.0f     // <1 = plus de résolution dans les nuances douces
#define EXPRESSION_ATTACK_MS  40.0f  // Constante de temps à la montée (réactivité crescendo)
#define EXPRESSION_RELEASE_MS 120.0f // Constante de temps à la descente (évite les trous en legato)
#define EXPRESSION_DEADBAND 1        // Variation mini avant envoi (1 = résolution maximale)
#define EXPRESSION_MIN_INTERVAL_MS 6 // Débit max ~160 msg/s, ne sature pas l'USB MIDI
#define EXPRESSION_14BIT    1        // 1 = CC11 (MSB) + CC43 (LSB) : 16384 pas au lieu de 128
                                     //     (activer "14-bit" dans le module MIDI-CC de Cardinal)

// ============================================================================
// FILTRE ANTI-PARASITES (EMI / INTÉGRATEUR NUMÉRIQUE BOUTONS)
// ============================================================================
// À une cadence de loop() de 2ms, un seuil de 10 exige 20ms d'état LOW continu
// ininterrompu, éliminant 100% des impulsions électromagnétiques (ex: frigo).
#define EMI_INTEGRATOR_MAX 10

#endif // CONFIG_H
