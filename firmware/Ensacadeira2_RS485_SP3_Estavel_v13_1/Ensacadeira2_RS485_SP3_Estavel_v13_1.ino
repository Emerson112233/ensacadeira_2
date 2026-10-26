// ============================================================================
// ENSACADEIRA 2 - TURAMIX - MONITOR DE ENSAQUE
// Versao: E2-RS485-SP3EST-v13.1 (26/10/2026)
// v13.1: saturacao 99999 -> 00500 estimado; pares finais e retomada apos lacuna.
// v13: um evento por descarga confirmada (2 pesos abaixo do limite); SP3 pode
// oscilar sem criar sacos. Ultimo patamar consistente/estavel, nunca maximo
// automatico. ID persistente, journal do ciclo e fila com CRC antes do envio.
// A v12 original permanece no pacote de manutencao. Nao gravar automaticamente.
// Ajustes da v13: ESCALA_LIMITE13, AJUSTES_DESCARGA13 e E2V13::defaultConfig().
//  v12: atualizacao pelo Wi-Fi SOB DEMANDA: comando Telnet "ota" libera a
//       gravacao remota por 10 min (fecha a conexao do Firebase e liga o
//       mDNS so nesse periodo). Fora disso nao gasta memoria.
//  v11: visor "opcao A": produto bem grande (ate tamanho 12), codigo pequeno
//       em cima, barra de status com bolinha 2x maior, ONLINE/SEM FIREBASE/
//       SEM INTERNET, hora, fila e IP. Sem peso ao vivo nem ultimo saco.
//       Tela inteira so e redesenhada ao trocar de produto; na barra, cada
//       campo e redesenhado sozinho e so quando muda (impacto minimo).
//  v10: limite de registros guardados sem internet (fila offline) com
//       margem de seguranca; aviso no display a partir de 80% e alarme
//       vermelho quando cheia; contador de sacos nao gravados (persistente).
//  v9: confirmado pelo .elf: o crash do login e falta de memoria dentro da
//      biblioteca do Firebase. Textos de log/tela passaram para a flash
//      (F()), liberando ~4,5 KB de RAM.
//  v8: crash no login do Firebase = FALTA DE MEMORIA no pico do login.
//      Libera memoria: OTA/mDNS desligados (USAR_OTA=false; grava pelo
//      cabo USB), buffer da serial 2048 -> 1024 bytes. Registra a memoria
//      livre logo antes do login (aparece no motivo do reinicio).
//  v7: Firebase iniciado IGUAL a Ensacadeira 1 (sem ajustar tempos de
//      espera nem tamanhos de buffer antes do Firebase.begin).
//  v6: lista de produtos lida do Firebase SOMENTE pelo comando Telnet
//      "atualizar" (nao le mais ao ligar nem a cada 5 min).
//  v5: marcacao mais fina das partes do codigo (RTC ler/gravar, inicio do
//      Firebase, entrada da rede) para achar o crash (excecao 29).
//  v4: registra o MOTIVO de cada reinicio (energia, watchdog, crash, saude)
//      e em que parte do codigo estava; comando Telnet "reinicios";
//      motivo publicado em /ips_wemos/ensacadeira2/ultimo_reset.
//      Reinicio por memoria baixa so em situacao realmente critica.
//  v3: CORRIGE leitura da fila (todo registro novo era descartado como
//      "corrompido"); linha rejeitada vai para /fila_rejeitada.txt em vez
//      de sumir; lista de produtos inexistente no Firebase nao e mais erro.
//  v2: lista de produtos no Firebase (/config_ensacadeira_2/produtos)
//      e IP publicado em /ips_wemos/ensacadeira2 logo apos conectar.
// Placa: LOLIN(WeMos) D1 R1 (ESP8266, formato Arduino UNO)
//
// O QUE ESTA VERSAO FAZ
//  - Le o indicador Alfa pela RS485 (Serial 3 do Alfa, protocolo T02 STD,
//    115200 8N1, pacote de 15 bytes) por um MAX485 em modo so-recepcao.
//  - Valida cada pacote (STX 02, ETX 03 e BCC = XOR dos bytes 1 a 14).
//  - SP3 0->1 abre ciclo; SP3=0 acima do limite nao encerra nem rearma.
//  - Encerra somente apos 2 pesos validos consecutivos abaixo do limite.
//  - Seleciona ultima estabilidade consistente ou mediana do patamar final;
//    leituras de descarga e picos nao viram automaticamente peso oficial.
//  - Cada registro leva tambem dados de diagnostico do ciclo (peso quando o
//    SP3 ligou, maximo, primeiro e ultimo estavel, tara, tempos) para
//    validar a logica com o ensaque real.
//  - Fila offline em LittleFS, envio ao Firebase Realtime Database,
//    display ILI9488 3,5" com produto + peso ao vivo + ultimo saco,
//    2 botoes, RTC DS3231, OTA e monitor Telnet (PuTTY, porta 23).
//
// LIGACOES (resumo - detalhes no guia que acompanha este arquivo)
//  Alfa RS485 A   -> MAX485 A        Alfa RS485 B -> MAX485 B
//  Alfa GND com.  -> MAX485 GND      MAX485 GND   -> WeMos GND
//  MAX485 VCC     -> WeMos 3V3  (EXPERIMENTAL; ideal: MAX3485/SP3485)
//  MAX485 RO      -> WeMos RX/D0 (GPIO3)
//  MAX485 /RE     -> GND             MAX485 DE    -> GND
//  MAX485 DI      -> sem ligacao
//  DISPLAY: CS=D4(GPIO4) DC=D3(GPIO5) RST=D2(GPIO16) MOSI=D7(GPIO13)
//           SCK=D5(GPIO14) VCC=3V3 LED=3V3 GND=GND  MISO nao ligar
//  RTC DS3231: SDA=D8(GPIO0) SCL=D9(GPIO2) VCC=3V3 GND=GND
//  BOTAO AVANCAR : D6(GPIO12) <-> GND
//  BOTAO RETORNAR: D10(GPIO15) <-> resistor 1k <-> 3V3 (NAO apertar no boot)
//
// ATENCAO NO UPLOAD PELO CABO USB: desconecte o fio RO (MAX485 -> RX/D0)
// antes de gravar e religue depois. O MAX485 ocupa o mesmo pino RX que o
// conversor USB usa para gravar. Pelo Wi-Fi (OTA) nao precisa.
//
// Bibliotecas (Gerenciador de Bibliotecas da IDE):
//  - Firebase Arduino Client Library for ESP8266 and ESP32 (Mobizt) 4.4.17
//  - GFX Library for Arduino (Moon On Our Nation) 1.6.9
//  - RTClib (Adafruit) 2.1.4  + Adafruit BusIO 1.17.4
// Placa: esp8266 core 3.1.2 -> "LOLIN(WeMos) D1 R1"
// Flash Size: 4MB (FS:2MB OTA:~1019KB)  (manter o mesmo para nao perder a fila)
// ============================================================================

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Firebase_ESP_Client.h>
#include <LittleFS.h>
#include <Wire.h>
#include <RTClib.h>
#include <time.h>
#include <Arduino_GFX_Library.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <stddef.h>

// BEGIN V13 ENGINE
#ifndef ENSACADEIRA2_V13_CYCLE_INC
#define ENSACADEIRA2_V13_CYCLE_INC

// Portable cycle detector. No Arduino, allocation, IO, product IDs or persistence.
// Config, State and Event are plain, trivially-copyable values. Persist State
// with an external version/CRC and restore it before feeding more packets.
#include <stdint.h>
#include <string.h>
#include <type_traits>

namespace E2V13 {

enum class Quality : uint8_t { Unconfirmed = 0, Median = 1, Stable = 2 };
enum class Step : uint8_t { Ignored, Started, Updated, Ready, Pending, Invalid };

struct Config {
  int32_t lowLimitRaw;
  int32_t consistentToleranceRaw;
  int32_t stableToleranceRaw;
  uint8_t stableSamples;
  uint8_t plateauSamples;
};

inline Config defaultConfig() {
  Config c = {};
  c.lowLimitRaw = 100;
  c.consistentToleranceRaw = 3;
  c.stableToleranceRaw = 1;
  c.stableSamples = 2;
  c.plateauSamples = 3;
  return c;
}

struct Reading {
  int32_t raw;
  uint32_t ms;
  uint8_t sp3;
  uint8_t unstable;
};

struct Event {
  int32_t raw;
  int32_t startRaw;
  int32_t maximumRaw;
  int32_t firstStableRaw;
  int32_t lastStableRaw;
  uint32_t readings;
  uint32_t sp3Readings;
  uint32_t packetsToStable;
  uint32_t startedMs;
  uint32_t completedMs;
  uint32_t durationMs;
  uint32_t cycleNumber;
  Quality quality;
  uint8_t stable;
  uint8_t ambiguous;
  uint8_t gap;
  uint8_t durationApproximate;
};

struct Candidate {
  int32_t raw;
  int32_t firstStableRaw;
  int32_t lastStableRaw;
  uint32_t firstStablePacket;
  Quality quality;
};

struct State {
  uint32_t magic;
  uint32_t version;
  Config config;
  Event event;  // Immutable while pending; feed() never discards it.
  Reading history[15];
  int32_t window[3];
  Candidate candidate;
  Candidate lowerAnchor;
  int32_t lowerFirstRaw;
  int32_t stableMin;
  int32_t stableMax;
  int32_t startRaw;
  int32_t maximumRaw;
  uint32_t startedMs;
  uint32_t lastSampleMs;
  uint32_t durationMs;
  uint32_t readings;
  uint32_t sp3Readings;
  uint32_t cycleNumber;
  uint32_t logicalRevision;
  uint8_t active;
  uint8_t loadSeen;
  uint8_t pending;
  uint8_t sp3Known;
  uint8_t previousSp3;
  uint8_t primed;
  uint8_t primingLowCount;
  uint8_t historyHead;  // Next write position; oldest when historyCount == 15.
  uint8_t historyCount;
  uint8_t windowCount;
  uint8_t stableCount;
  uint8_t lowCount;
  uint8_t emptyLowCount;
  uint8_t lowerPending;
  uint8_t discharge;
  uint8_t ambiguous;
  uint8_t gap;
  uint8_t clockKnown;
  uint8_t durationApproximate;
};

static_assert(std::is_trivially_copyable<State>::value, "State must be serializable");
static_assert(std::is_standard_layout<State>::value, "State must have fixed layout");

class Engine {
 public:
  Engine() { clearState(defaultConfig()); }

  // Changing product/threshold never erases an active cycle or pending event.
  bool begin(const Config& c) { return configure(c); }
  bool begin(int32_t lowLimitRaw) {
    Config c = defaultConfig();
    c.lowLimitRaw = lowLimitRaw;
    return configure(c);
  }
  bool configure(const Config& c) {
    if (s_.active || s_.pending || !validConfig(c)) return false;
    s_.config = c;
    changed();
    return true;
  }
  bool reset() {
    if (s_.active || s_.pending) return false;
    Config c = s_.config;
    clearState(c);
    return true;
  }

  const State& state() const { return s_; }
  const State& snapshot() const { return s_; }
  bool pending() const { return s_.pending != 0; }
  bool ready() const { return pending(); }
  const Event& result() const { return s_.event; }

  bool restore(const State& saved) {
    if (s_.active || s_.pending || !validState(saved)) return false;
    s_ = saved;
    return true;
  }

  // Call after a reboot/restoration. Excludes unobserved downtime from duration;
  // durationApproximate tells downstream consumers this is a lower bound.
  void rebaseClock(uint32_t now) {
    if (!s_.active || s_.pending) return;
    s_.lastSampleMs = now;
    s_.clockKnown = 1;
    s_.durationApproximate = 1;
    changed();
  }

  // Any invalid packet or known communication gap breaks consecutiveness.
  // The active cycle, prior priming and a trusted weight survive the gap.
  void invalidate() {
    if (s_.pending) return;
    if (!s_.active) {
      const bool changedIdle = s_.sp3Known || s_.primingLowCount || !s_.gap;
      // A known empty scale must not require another empty-scale observation
      // while the next bag is already filling. Relearn SP3 without creating an
      // artificial edge; only a later observed 0->1 can start that next cycle.
      s_.sp3Known = s_.primingLowCount = 0;
      s_.gap = s_.ambiguous = s_.durationApproximate = 1;
      if (changedIdle) {
        changed();
      }
      return;
    }
    const bool wasContinuous = !s_.gap || s_.lowCount || s_.windowCount ||
                               s_.stableCount || s_.clockKnown;
    s_.gap = 1;
    s_.ambiguous = 1;
    s_.durationApproximate = 1;
    s_.clockKnown = 0;
    s_.lowCount = 0;
    s_.emptyLowCount = 0;
    clearGroup();
    if (wasContinuous) changed();
  }

  // Only call once the external durable queue/journal has committed result().
  // Returns false rather than erasing an active cycle or nonexistent event.
  bool acknowledge() {
    if (!s_.pending) return false;
    s_.pending = 0;
    // The committed event retains its flags. A future event inherits a gap
    // only if another invalidation is observed after this acknowledgement.
    s_.gap = s_.ambiguous = s_.durationApproximate = 0;
    changed();
    return true;
  }

  // Call once per VALID raw packet. Negative values are invalid, not low.
  // First packet at boot learns SP3; only an observed 0->1 starts a new cycle.
  // SP3 chatter and elapsed time cannot terminate or restart an active cycle.
  Step feed(int32_t raw, bool sp3, bool unstable, uint32_t now) {
    if (s_.pending) return Step::Pending;
    if (raw < 0) { invalidate(); return Step::Invalid; }
    if (!s_.active && !s_.primed) {
      if (raw < s_.config.lowLimitRaw) {
        if (s_.primingLowCount < 2) ++s_.primingLowCount;
      } else s_.primingLowCount = 0;
      s_.sp3Known = 1;
      s_.previousSp3 = sp3;
      if (s_.primingLowCount == 2) s_.primed = 1;
      changed();
      return Step::Ignored;
    }
    if (!s_.sp3Known) {
      s_.sp3Known = 1;
      s_.previousSp3 = sp3;
      if (!s_.active) { changed(); return Step::Ignored; }
    }
    const bool rising = sp3 && !s_.previousSp3;
    s_.previousSp3 = sp3;
    bool started = false;
    if (!s_.active) {
      if (!rising) return Step::Ignored;
      start(raw, now);
      started = true;
    }
    appendHistory(raw, sp3, unstable, now);
    saturatingIncrement(s_.readings);
    if (sp3) saturatingIncrement(s_.sp3Readings);
    if (raw > s_.maximumRaw) s_.maximumRaw = raw;
    if (!s_.loadSeen && raw > s_.config.lowLimitRaw) {
      s_.loadSeen = 1;
      changed();
    }
    if (s_.clockKnown) {
      const uint32_t delta = now - s_.lastSampleMs;
      if (UINT32_MAX - s_.durationMs < delta) {
        s_.durationMs = UINT32_MAX;
        s_.durationApproximate = 1;
      } else s_.durationMs += delta;
    }
    s_.lastSampleMs = now;
    s_.clockKnown = 1;

    if (raw < s_.config.lowLimitRaw) {
      if (!s_.loadSeen) {
        // SP3 may assert before filling begins; empty readings are not a bag.
        clearGroup();
        s_.lowCount = 0;
        if (!sp3) {
          if (s_.emptyLowCount < 2) ++s_.emptyLowCount;
        } else s_.emptyLowCount = 0;
        if (s_.emptyLowCount == 2) {
          // A spurious empty pulse can release network work without creating
          // an event. The observed low SP3 still requires a fresh rising edge.
          s_.active = 0;
          changed();
        }
        return started ? Step::Started : Step::Updated;
      }
      if (s_.lowCount < 2) ++s_.lowCount;
      // Retain only the last load window for final insufficient-sample
      // estimation. Stable continuity is broken immediately; a return above
      // the limit clears the retained window before processing that reading.
      s_.stableCount = 0;
      changed();  // Checkpoint the first low; the next low produces the event.
      if (s_.lowCount == 2) { finish(now); return Step::Ready; }
      return started ? Step::Started : Step::Updated;
    }
    s_.emptyLowCount = 0;
    if (s_.lowCount) { s_.lowCount = 0; clearGroup(); changed(); }
    consider(raw, unstable);
    return started ? Step::Started : Step::Updated;
  }

 private:
  State s_;
  static constexpr uint32_t kMagic = 0x45325613UL;
  static constexpr uint32_t kVersion = 1;

  static bool validConfig(const Config& c) {
    return c.lowLimitRaw > 0 && c.consistentToleranceRaw >= 0 &&
           c.stableToleranceRaw >= 0 &&
           c.stableToleranceRaw <= c.consistentToleranceRaw &&
           c.stableSamples >= 2 && c.stableSamples <= 3 &&
           c.plateauSamples == 3;
  }
  static bool validState(const State& s) {
    return s.magic == kMagic && s.version == kVersion && validConfig(s.config) &&
           !(s.active && s.pending) && s.active <= 1 && s.pending <= 1 &&
           s.historyHead < 15 && s.historyCount <= 15 && s.windowCount <= 3 &&
           s.stableCount <= 3 && s.lowCount <= 2 && s.emptyLowCount <= 2 &&
           s.primingLowCount <= 2 &&
           static_cast<uint8_t>(s.candidate.quality) <= 2 &&
           static_cast<uint8_t>(s.lowerAnchor.quality) <= 2 &&
           static_cast<uint8_t>(s.event.quality) <= 2;
  }
  static void saturatingIncrement(uint32_t& value) {
    if (value != UINT32_MAX) ++value;
  }
  static int32_t minRaw(int32_t a, int32_t b) { return a < b ? a : b; }
  static int32_t maxRaw(int32_t a, int32_t b) { return a > b ? a : b; }
  static int32_t median3(int32_t a, int32_t b, int32_t c) {
    if (a > b) { const int32_t t = a; a = b; b = t; }
    if (b > c) { const int32_t t = b; b = c; c = t; }
    return a > b ? a : b;
  }
  static bool belowByMore(int32_t raw, int32_t anchor, int32_t tolerance) {
    return static_cast<int64_t>(anchor) - raw > tolerance;
  }
  void changed() { ++s_.logicalRevision; }
  void clearState(const Config& c) {
    memset(&s_, 0, sizeof(s_));
    s_.magic = kMagic;
    s_.version = kVersion;
    s_.config = c;
    s_.candidate.raw = -1;
    s_.candidate.firstStableRaw = -1;
    s_.candidate.lastStableRaw = -1;
  }
  void clearGroup() {
    s_.windowCount = 0;
    s_.stableCount = 0;
  }
  void start(int32_t raw, uint32_t now) {
    const uint8_t inheritedGap = s_.gap;
    s_.active = 1;
    s_.loadSeen = 0;
    s_.historyHead = s_.historyCount = 0;
    s_.lowCount = s_.lowerPending = s_.discharge = 0;
    s_.emptyLowCount = 0;
    s_.ambiguous = s_.gap = s_.durationApproximate = inheritedGap;
    s_.clockKnown = 1;
    s_.startedMs = s_.lastSampleMs = now;
    s_.durationMs = s_.readings = s_.sp3Readings = 0;
    s_.startRaw = s_.maximumRaw = raw;
    s_.candidate = Candidate{};
    s_.candidate.raw = s_.candidate.firstStableRaw = s_.candidate.lastStableRaw = -1;
    clearGroup();
    saturatingIncrement(s_.cycleNumber);
    changed();
  }
  void appendHistory(int32_t raw, bool sp3, bool unstable, uint32_t now) {
    Reading& r = s_.history[s_.historyHead];
    r = Reading{};
    r.raw = raw;
    r.ms = now;
    r.sp3 = sp3;
    r.unstable = unstable;
    s_.historyHead = static_cast<uint8_t>((s_.historyHead + 1) % 15);
    if (s_.historyCount < 15) ++s_.historyCount;
  }
  void pushGroup(int32_t raw, bool unstable) {
    if (s_.windowCount < 3) s_.window[s_.windowCount++] = raw;
    else {
      s_.window[0] = s_.window[1];
      s_.window[1] = s_.window[2];
      s_.window[2] = raw;
    }
    if (unstable) { s_.stableCount = 0; return; }
    if (!s_.stableCount) {
      s_.stableMin = s_.stableMax = raw;
      s_.stableCount = 1;
      return;
    }
    const int32_t mn = minRaw(s_.stableMin, raw);
    const int32_t mx = maxRaw(s_.stableMax, raw);
    if (static_cast<int64_t>(mx) - mn <= s_.config.stableToleranceRaw) {
      s_.stableMin = mn;
      s_.stableMax = mx;
      if (s_.stableCount < 3) ++s_.stableCount;
    } else {
      s_.stableMin = s_.stableMax = raw;
      s_.stableCount = 1;
    }
  }
  bool consistentWindow() const {
    if (s_.windowCount != 3) return false;
    const int32_t mn = minRaw(s_.window[0], minRaw(s_.window[1], s_.window[2]));
    const int32_t mx = maxRaw(s_.window[0], maxRaw(s_.window[1], s_.window[2]));
    return static_cast<int64_t>(mx) - mn <= s_.config.consistentToleranceRaw;
  }
  void select(int32_t raw, Quality quality) {
    Candidate next = s_.candidate;
    next.raw = raw;
    next.quality = quality;
    if (quality == Quality::Stable) {
      if (next.firstStableRaw < 0) {
        next.firstStableRaw = raw;
        next.firstStablePacket = s_.readings;
      }
      next.lastStableRaw = raw;
    }
    if (next.raw != s_.candidate.raw || next.quality != s_.candidate.quality ||
        next.firstStableRaw != s_.candidate.firstStableRaw ||
        next.lastStableRaw != s_.candidate.lastStableRaw) {
      s_.candidate = next;
      changed();
    }
  }
  void consider(int32_t raw, bool unstable) {
    bool groupAlreadyPushed = false;
    if (s_.discharge) {
      if (s_.candidate.raw < 0 || belowByMore(raw, s_.candidate.raw,
                                            s_.config.consistentToleranceRaw)) {
        clearGroup();
        return;
      }
      // A rebound to the previous plateau or above disproves an irreversible
      // unload assumption only after three fresh consistent samples. Keep the
      // ambiguity flag so a real recovery can become the final plateau.
      pushGroup(raw, unstable);
      if (!consistentWindow()) return;
      groupAlreadyPushed = true;
      s_.discharge = s_.lowerPending = 0;
      s_.ambiguous = 1;
      changed();
    }
    // Equality is not a low confirmation, but is not a bag plateau either.
    if (raw == s_.config.lowLimitRaw) { clearGroup(); return; }
    const int32_t tolerance = s_.config.consistentToleranceRaw;
    if (s_.lowerPending) {
      if (!belowByMore(raw, s_.lowerAnchor.raw, tolerance)) {
        // A downward spike recovered: retain the previously trusted plateau.
        s_.candidate = s_.lowerAnchor;
        s_.lowerPending = 0;
        clearGroup();
        changed();
      } else if (belowByMore(raw, s_.lowerFirstRaw, tolerance)) {
        // Two significant downward stages above the low limit identify unload.
        // Roll back a provisional lower plateau; terminal low samples themselves
        // never invalidate a legitimate three-sample lower plateau.
        s_.candidate = s_.lowerAnchor;
        s_.discharge = 1;
        clearGroup();
        changed();
        return;
      }
    } else if (s_.candidate.raw >= 0 &&
               belowByMore(raw, s_.candidate.raw, tolerance)) {
      s_.lowerPending = 1;
      s_.lowerAnchor = s_.candidate;
      s_.lowerFirstRaw = raw;
      clearGroup();
      changed();
    }
    if (!groupAlreadyPushed) pushGroup(raw, unstable);
    const bool consistent = consistentWindow();
    if (s_.lowerPending && !consistent) return;
    if (s_.stableCount >= s_.config.stableSamples) {
      select(raw, Quality::Stable);
      if (s_.lowerPending) confirmLowerPlateau();
    } else if (consistent) {
      const int32_t median = median3(s_.window[0], s_.window[1], s_.window[2]);
      // A stable value during filling is not a trustworthy final plateau when
      // a later consistent group is clearly elsewhere. Preserve stability only
      // while it agrees with the final group within the configured tolerance.
      if (s_.lowerPending || s_.candidate.quality != Quality::Stable ||
          belowByMore(median, s_.candidate.raw, tolerance) ||
          belowByMore(s_.candidate.raw, median, tolerance)) {
        select(median, Quality::Median);
        if (s_.lowerPending) confirmLowerPlateau();
      }
    } else if (!s_.lowerPending && s_.candidate.quality == Quality::Unconfirmed) {
      // With too few samples retain an explicitly uncertain median of available
      // plausible samples. Never substitute the cycle maximum for the weight.
      int32_t fallback = raw;
      if (s_.windowCount == 2) {
        fallback = static_cast<int32_t>((static_cast<int64_t>(s_.window[0]) +
                                        s_.window[1]) / 2);
      } else if (s_.windowCount == 3) {
        fallback = median3(s_.window[0], s_.window[1], s_.window[2]);
      }
      select(fallback, Quality::Unconfirmed);
    }
  }
  void confirmLowerPlateau() {
    // Three consistent samples make this a real candidate, not a solitary
    // unload reading. Subsequent discharge must preserve this new plateau.
    const bool replacedTrusted = s_.lowerAnchor.quality != Quality::Unconfirmed;
    s_.lowerAnchor = s_.candidate;
    s_.lowerPending = 0;
    if (replacedTrusted) s_.ambiguous = 1;
    changed();
  }
  void rejectStaleFillingCandidate() {
    if (s_.discharge || s_.candidate.quality == Quality::Unconfirmed) return;
    bool supported = false;
    uint8_t higher = 0;
    uint8_t available = 0;
    int32_t recent[3] = {};
    for (uint8_t i = 0; i < s_.historyCount; ++i) {
      const uint8_t index = static_cast<uint8_t>((s_.historyHead + 29 - i) % 15);
      const int32_t raw = s_.history[index].raw;
      if (raw <= s_.config.lowLimitRaw) continue;
      if (available < 3) recent[available++] = raw;
      if (belowByMore(s_.candidate.raw, raw, s_.config.consistentToleranceRaw)) ++higher;
      else if (!belowByMore(raw, s_.candidate.raw, s_.config.consistentToleranceRaw)) supported = true;
    }
    // An early stable filling value without any recent support is not final
    // stability. An irregular higher plateau has insufficient confidence even
    // if it never supplied a tight three-sample group.
    if (!supported && higher >= 3 && available == 3) {
      select(median3(recent[0], recent[1], recent[2]), Quality::Unconfirmed);
      s_.ambiguous = 1;
    }
  }
  void estimateFinalPair() {
    // The confidence rules remain explicit: Stable needs stableSamples (2)
    // within stableToleranceRaw (1); Median needs plateauSamples (3) within
    // consistentToleranceRaw (3). A final pair is only an insufficient-sample
    // estimate, and must never override Stable or a confirmed unload anchor.
    if (s_.candidate.quality == Quality::Stable || s_.discharge ||
        s_.windowCount < 2) return;
    int32_t recent[3] = {};
    uint8_t count = 0;
    for (uint8_t i = 0; i < s_.historyCount && count < 3; ++i) {
      const uint8_t index = static_cast<uint8_t>((s_.historyHead + 29 - i) % 15);
      const int32_t raw = s_.history[index].raw;
      if (raw < s_.config.lowLimitRaw && count == 0) continue;
      // Do not skip an intervening zero/threshold reading to assemble a pair.
      if (raw <= s_.config.lowLimitRaw) break;
      recent[count++] = raw;
    }
    if (count < 2) return;
    const int32_t pairMin = minRaw(recent[0], recent[1]);
    const int32_t pairMax = maxRaw(recent[0], recent[1]);
    if (static_cast<int64_t>(pairMax) - pairMin > s_.config.consistentToleranceRaw) return;
    if (s_.windowCount >= s_.config.plateauSamples && count == 3) {
      const int32_t mn = minRaw(pairMin, recent[2]);
      const int32_t mx = maxRaw(pairMax, recent[2]);
      if (static_cast<int64_t>(mx) - mn <= s_.config.consistentToleranceRaw) return;
    }
    const int32_t estimate = static_cast<int32_t>(
        (static_cast<int64_t>(recent[0]) + recent[1]) / 2);
    select(estimate, Quality::Unconfirmed);
    s_.ambiguous = 1;
  }
  void finish(uint32_t now) {
    rejectStaleFillingCandidate();
    estimateFinalPair();
    Event e = {};
    e.raw = s_.candidate.raw < 0 ? 0 : s_.candidate.raw;
    e.startRaw = s_.startRaw;
    e.maximumRaw = s_.maximumRaw;
    e.firstStableRaw = s_.candidate.firstStableRaw;
    e.lastStableRaw = s_.candidate.lastStableRaw;
    e.readings = s_.readings;
    e.sp3Readings = s_.sp3Readings;
    e.packetsToStable = s_.candidate.firstStablePacket;
    e.startedMs = s_.startedMs;
    e.completedMs = now;
    e.durationMs = s_.durationMs;
    e.cycleNumber = s_.cycleNumber;
    e.quality = s_.candidate.quality;
    e.stable = s_.candidate.quality == Quality::Stable;
    e.ambiguous = s_.ambiguous || s_.candidate.quality == Quality::Unconfirmed;
    e.gap = s_.gap;
    e.durationApproximate = s_.durationApproximate;
    s_.event = e;
    s_.active = 0;
    s_.pending = 1;
    changed();
  }
};

}  // namespace E2V13
#endif

// END V13 ENGINE

// ============================================================================
// 1) CONFIGURACAO QUE VOCE PRECISA PREENCHER
// ============================================================================
const char* WIFI_SSID  = "PREENCHER_SSID";
const char* WIFI_SENHA = "PREENCHER_SENHA_WIFI";

#define API_KEY      "PREENCHER_API_KEY"
#define DATABASE_URL "PREENCHER_URL_DO_RTDB"   // ex.: https://xxxx-default-rtdb.firebaseio.com/
const char* FIREBASE_EMAIL = "PREENCHER_EMAIL";
const char* FIREBASE_SENHA = "PREENCHER_SENHA_FIREBASE";

// Atualizacao pelo Wi-Fi (OTA) SOB DEMANDA: no PuTTY digite "ota" e a
// WeMos aparece no Arduino IDE como porta de rede por OTA_MINUTOS_LIBERADA.
// Ela nao fica ligada direto porque o mDNS ocupa a memoria que o login do
// Firebase precisa. A senha abaixo e pedida pelo IDE na hora de carregar.
const bool  USAR_OTA     = false;   // true = sempre ligada (NAO recomendado)
const unsigned long OTA_MINUTOS_LIBERADA = 10;
const char* OTA_HOSTNAME = "ensacadeira2";
const char* OTA_SENHA    = "PREENCHER_SENHA_OTA";

const char* FIREBASE_PATH_PESAGENS  = "/pesagens_2";            // producao (dashboard)
const char* FIREBASE_PATH_TESTES    = "/testes_ensacadeira_2";  // simulacao 99999
const char* FIREBASE_PATH_IPS_WEMOS = "/ips_wemos";
const char* FIREBASE_ID_DISPOSITIVO = "ensacadeira2";
const char* FIREBASE_PATH_CONFIG     = "/config_ensacadeira_2";  // lista de produtos em .../produtos
const char* FW_VERSAO = "E2-RS485-SP3EST-v13.1";

// ============================================================================
// 2) AJUSTES DA LOGICA DE PESAGEM (os tempos sao contados em PACOTES:
//    o Alfa mandou ~4 pacotes/s no teste de 23/10, ou seja ~250 ms cada)
// ============================================================================
const uint8_t  BIT_SP3_STS2      = 2;     // STS2 (pacote[2]) bit 2 = SP3
const uint8_t  BIT_INSTAVEL_STS1 = 4;     // STS1 (pacote[1]) bit 4 = 1 instavel
const bool     VALIDAR_BCC       = true;  // descarta pacote com ETX/BCC errado

// V13: parametros de consistencia ficam em E2V13::defaultConfig(), abaixo.

// Casas decimais do peso no Alfa. -1 = AINDA NAO CONFIRMADO (nao envia peso_kg).
// Quando confirmar no indicador (ex.: 25,0 kg aparece como 00250 -> 1 casa), ajuste aqui.
const int8_t   CASAS_DECIMAIS = -1;

// ============================================================================
// 3) PINOS - WeMos D1 R1
// ============================================================================
const int  PINO_AVANCAR  = 12;   // D6 (GPIO12) -> GND
const int  PINO_RETORNAR = 15;   // D10 (GPIO15) -> 1k -> 3V3 (ativo em HIGH)
const int  PINO_RTC_SDA  = 0;    // D8 (GPIO0)
const int  PINO_RTC_SCL  = 2;    // D9 (GPIO2)
#define TFT_CS   4               // D4
#define TFT_DC   5               // D3
#define TFT_RST 16               // D2
const uint32_t TFT_SPI_HZ = 4000000UL;   // 4 MHz testado; fios curtos: pode testar 16000000

// ============================================================================
// 4) TEMPOS DE REDE E MANUTENCAO
// ============================================================================
const unsigned long INTERVALO_REDE_PRODUCAO  = 1000UL;   // 1 operacao Firebase/s com maquina rodando
const unsigned long INTERVALO_REDE_PARADA    = 300UL;
const unsigned long TEMPO_PARADA_MS          = 60000UL;  // 1 min sem ciclo = parada
const unsigned long ESPERA_NTP_MS            = 120000UL;
const unsigned long INTERVALO_RECONEXAO_WIFI = 30000UL;
const unsigned long TIMEOUT_WIFI_SEM_CONEXAO = 300000UL;
const unsigned long REINICIO_SEM_ENVIO_MS    = 1800000UL;
const unsigned long INTERVALO_AJUSTE_RTC     = 21600000UL;
const unsigned long TIMEOUT_RESULTADO_TESTE  = 30000UL;
const uint16_t      TELNET_PORTA             = 23;

// ============================================================================
// 5) MEMORIA (LittleFS)
// ============================================================================
// Fila offline (sacos guardados sem internet). O LittleFS tem ~2 MB; cada
// saco ocupa ~110-130 bytes. Ficam DUAS travas de seguranca:
//  1) no maximo LIMITE_REGISTROS_PENDENTES sacos esperando envio;
//  2) nunca usar os ultimos RESERVA_FS bytes do sistema de arquivos.
// O que vier antes vale. Com 10.000 sacos usa ~1,3 MB (cerca de 65%).
const uint32_t LIMITE_REGISTROS_PENDENTES = 10000;
const uint8_t  PERCENT_AVISO_MEMORIA      = 80;    // aviso amarelo no display
const size_t   BYTES_POR_REGISTRO_ESTIM   = 384;   // estimativa para o calculo
const size_t RESERVA_FS    = 128 * 1024;
const size_t TAM_MAX_LINHA = 768;
const char* ARQUIVO_FILA         = "/offline.jsonl";
const char* ARQUIVO_POSICAO      = "/offline_pos.txt";
const char* ARQUIVO_POSICAO_TEMP = "/offline_pos.tmp";
const char* ARQUIVO_ULTIMA       = "/ultima_pesagem.json";
const char* ARQUIVO_PRODUTO      = "/produto_ativo.txt";
const char* ARQUIVO_PRODUTOS     = "/produtos.txt";     // copia local da lista vinda do Firebase
const char* ARQUIVO_BOOT         = "/boot.txt";
const char* ARQUIVO_REJEITADOS   = "/fila_rejeitada.txt"; // linhas que nao puderam ser lidas
const char* ARQUIVO_REINICIOS    = "/reinicios.txt";      // historico dos reinicios
const char* ARQUIVO_PERDIDAS     = "/pesagens_perdidas.txt"; // sacos que nao couberam
const uint32_t EPOCH_MIN = 1704067200UL;
const uint32_t EPOCH_MAX = 4000000000UL;

// ============================================================================
// 6) PRODUTOS
// A lista oficial fica no Firebase em /config_ensacadeira_2/produtos.
// O WeMos SO le essa lista quando voce digita "atualizar" no Telnet.
// A ultima lista lida fica guardada no LittleFS (vale depois de reiniciar
// e sem internet). A lista abaixo so e usada se nunca houve "atualizar".
// ============================================================================
const int MAX_PRODUTOS = 20;
const int TAM_MAX_PRODUTO = 40;
const int TOTAL_PRODUTOS_PADRAO = 6;
const char* const PRODUTOS_PADRAO[TOTAL_PRODUTOS_PADRAO] = {
  "598 - MILHO QUEBRADO 25KG",
  "121 - MILHO INTEIRO 25KG",
  "19 - MILHO INTEIRO 50KG",
  "5270 - DIETA TOTAL 15% 50KG",
  "5375 - CASCA GROSSA 25KG",
  "17800 - BANDINHA 40KG"
};
String produtos[MAX_PRODUTOS];
int totalProdutos = 0;
String origemProdutos = "codigo";
int indiceAtual = 0;
bool produtosLidosNesteBoot = false;
bool pedidoAtualizarProdutos = false;
unsigned long ultimaBuscaProdutos = 0;

// ============================================================================
// TIPOS
// ============================================================================
struct Registro {
  String id, chaveFirebase;
  bool v13 = false, confiavel = false, estimado = false, lacuna = false, recuperado = false;
  uint8_t amostras = 0;
  char qualidade = 'I';
  long limite = -1;
  uint32_t duracao = 0;
  uint32_t epoch = 0;
  uint32_t boot = 0;
  uint32_t ms = 0;
  bool teste = false;
  String peso;              // 5 digitos, como vem do Alfa
  String produto;
  bool temDiag = false;     // false = linha de versao antiga (sem diagnostico)
  bool estavel = false;
  char criterio = '?';      // D = descarga confirmada; E/T permanecem legados
  long pesoSp3 = -1;
  long pesoMax = -1;
  long pesoEst1 = -1;
  long pesoEstU = -1;
  long tara = -1;
  uint32_t pacAteEst = 0;
  uint32_t pacSp3 = 0;
  uint32_t msSp3 = 0;
};

struct Ciclo {
  bool ativo = false;
  bool simulado = false;
  uint32_t numero = 0;
  uint32_t pacotes = 0;       // pacotes desde que o SP3 ligou
  uint32_t pacotesSp3 = 0;    // pacotes com SP3 = 1
  unsigned long inicioMs = 0;
  long pesoSp3 = 0;
  long pesoMax = 0;
  long tara = 0;
  uint8_t sp3OffSeq = 0;
  uint8_t seqEstavel = 0;
  long seqMin = 0;
  long seqMax = 0;
  bool achouEstavel = false;
  long pesoEst1 = -1;
  long pesoEstU = -1;
  uint32_t pacAteEst = 0;
};

enum ResultadoEnvio { ENVIO_NADA, ENVIO_OK, ENVIO_FALHOU, ENVIO_ESPERAR };

// ============================================================================
// OBJETOS
// ============================================================================
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
RTC_DS3231 rtc;
Arduino_DataBus *displayBus = new Arduino_ESP8266SPI(TFT_DC, TFT_CS);
Arduino_GFX *tft = new Arduino_ILI9488_18bit(displayBus, TFT_RST, 1, false);
WiFiServer telnetServer(TELNET_PORTA);
WiFiClient telnetClient;

// ============================================================================
// ESTADO GLOBAL
// ============================================================================
bool displayOK = false;
bool littleFsOK = false;
bool firebaseIniciado = false;
bool firebaseProntoCache = false;
bool rtcOK = false;
bool rtcConfiavel = false;
bool memoriaCheia = false;
uint32_t fsLivreCache = 0;           // bytes livres no LittleFS (atualizado periodicamente)
uint32_t fsTotalCache = 0;
unsigned long ultimaLeituraFs = 0;
bool servicosRedeIniciados = false;
bool otaTelaTrocada = false;
bool otaAtiva = false;               // OTA liberada (sempre ou pelo comando "ota")
unsigned long otaLiberadaEm = 0;
int  otaCodigoErro = -1;

bool ipFirebasePendente = false;
bool ipFirebasePublicado = false;
IPAddress ultimoIpDetectado(0, 0, 0, 0);
unsigned long ultimaTentativaIpFirebase = 0;

uint32_t bootId = 0;
uint32_t ultimoEpochGravado = 0;
uint32_t totalRegistrados = 0;
uint32_t sacosEstaveis = 0;
uint32_t sacosSemEstavel = 0;
uint32_t pesagensPerdidas = 0;
uint32_t registrosCorrompidos = 0;
uint32_t overrunsSerial = 0;
uint32_t telnetLinhasPerdidas = 0;

unsigned long ultimoTempoConectado = 0;
unsigned long ultimaTentativaWiFi = 0;
unsigned long momentoWifiConectou = 0;
unsigned long ultimaOperacaoRede = 0;
unsigned long ultimoEnvioOk = 0;
unsigned long ultimaPesagemMillis = 0;
unsigned long proximaTentativaFirebase = 0;
uint8_t falhasSeguidas = 0;

unsigned long posicaoFila = 0;
int pendenciasCache = 0;

// Recepcao RS485 / T02
uint8_t rxBuf[15];
uint8_t rxPos = 0;
uint32_t bytesRecebidos = 0;
uint32_t bytesUltimoSegundo = 0;
unsigned long ultimoByteMs = 0;
uint32_t pacotesValidos = 0;
uint32_t pacotesInvalidos = 0;      // ETX ou BCC errado
uint32_t pacotesPesoInvalido = 0;   // peso nao numerico
uint8_t ultimoPacote[15];
unsigned long ultimoPacoteMs = 0;

// Estado lido da balanca
bool  sp3Inicializado = false;
bool  sp3Anterior = false;
bool  aguardarSp3Desligar = false;
uint8_t sp3OffSeqEspera = 0;
uint8_t sts1Anterior = 0xFF;
uint8_t sts2Anterior = 0xFF;
long  pesoAtual = -1;
bool  instavelAtual = true;
bool  sp3Atual = false;
Ciclo ciclo;
uint32_t contadorCiclos = 0;

// Ultima pesagem (tela)
String ultimaPesagemPeso = "";
String ultimaPesagemHora = "";
String ultimaPesagemData = "";
bool   ultimaPesagemEstavel = false;
bool   ultimaPesagemTeste = false;

// Simulacao (2 botoes 3 s ou comando "teste")
bool simulacaoAtiva = false;
uint8_t simulacaoPasso = 0;
unsigned long simulacaoUltimo = 0;
bool aguardandoResultadoTeste = false;
unsigned long inicioEsperaTeste = 0;

// Telnet
bool monitorEventos = true;     // liga sozinho a cada conexao
bool monitorPacotes = false;
unsigned long debugBytesFim = 0;
String linhaCrua;
char telnetBuf[28];
uint8_t telnetLen = 0;
uint8_t telnetIac = 0;

// Display
bool telaPrincipalAtiva = false;
bool mensagemTemporariaAtiva = false;
unsigned long mensagemTemporariaFim = 0;
unsigned long ultimaChecagemStatus = 0;
// O que esta desenhado agora na barra de status (para so redesenhar o que mudou)
int      barraNivel = -1;
int      barraConexao = -1;
String   barraHora = "";
long     barraFila = -1;
String   barraIp = "";
String   barraAviso = "#";
String   barraAlerta = "";

// ============================================================================
// DIAGNOSTICO DE REINICIO
// Guardado na memoria RTC do ESP8266: sobrevive a reinicio por watchdog,
// crash e ESP.restart(), mas SOME quando falta energia. Por isso, se o
// motivo vier como "energia" sem ninguem ter desligado, o problema e
// alimentacao (fonte fraca, cabo, queda de tensao).
// ============================================================================
enum Etapa : uint32_t {
  ET_SETUP = 1, ET_OTA, ET_TELNET, ET_WIFI, ET_BOTOES, ET_BALANCA, ET_SIMULACAO, ET_RTC,
  ET_FB_READY, ET_FB_IP, ET_FB_PRODUTOS, ET_FB_ENVIO, ET_DISPLAY, ET_SAUDE, ET_OCIOSO,
  ET_RTC_PROCURAR, ET_RTC_LER, ET_RTC_GRAVAR, ET_REDE, ET_FB_BEGIN, ET_FB_DEPOIS_BEGIN
};
// Nomes das partes do codigo (na flash, nao ocupam RAM)
const __FlashStringHelper* nomeEtapa(uint32_t e) {
  switch (e) {
    case 0: return F("?");
    case 1: return F("setup (inicializacao)");
    case 2: return F("OTA");
    case 3: return F("Telnet");
    case 4: return F("Wi-Fi");
    case 5: return F("botoes");
    case 6: return F("leitura balanca");
    case 7: return F("simulacao");
    case 8: return F("RTC");
    case 9: return F("Firebase.ready (login/token)");
    case 10: return F("Firebase: publicar IP");
    case 11: return F("Firebase: ler produtos");
    case 12: return F("Firebase: enviar pesagem");
    case 13: return F("display");
    case 14: return F("verificacao de saude");
    case 15: return F("fim do loop");
    case 16: return F("RTC: procurar modulo");
    case 17: return F("RTC: ler hora");
    case 18: return F("RTC: gravar hora do NTP");
    case 19: return F("rede (inicio)");
    case 20: return F("Firebase.begin (iniciar)");
    case 21: return F("Firebase: logo apos iniciar");
    default: return F("?");
  }
}
struct DiagRtc {
  uint32_t magic;
  uint32_t boot;
  uint32_t uptimeS;
  uint32_t heapMin;
  uint32_t blocoMin;
  uint32_t etapa;
  char motivo[40];
  uint32_t heapAntesLogin;    // memoria livre logo antes do Firebase.begin
  uint32_t blocoAntesLogin;
};
const uint32_t DIAG_MAGIC = 0xE2D1A608;
const uint32_t DIAG_OFFSET = 64;           // blocos de 4 bytes; os primeiros sao usados pela OTA
const uint32_t DIAG_OFFSET_ETAPA = DIAG_OFFSET + 5;
DiagRtc diagRtc;
String ultimoResetTexto = "";

inline void marcarEtapa(uint32_t e) {
  if (diagRtc.etapa == e) return;
  diagRtc.etapa = e;
  ESP.rtcUserMemoryWrite(DIAG_OFFSET_ETAPA, &diagRtc.etapa, sizeof(uint32_t));
}

// ============================================================================
// PROTOTIPOS
// ============================================================================
bool telnetPronto();
void logln(const String &s);
String prefixoLog();
bool epochValido(uint32_t t);
bool internetConectada();
bool maquinaParada();
bool extrairStringJson(const String &json, const String &campo, String &valor);
bool extrairBoolJson(const String &json, const String &campo, bool &valor);
bool extrairNumeroJson(const String &json, const String &campo, unsigned long &valor);
String lerArquivoTexto(const char* caminho, size_t maximo);
void inicializarNTP();
uint32_t lerRTC();
uint32_t obterEpochAtual();
bool relogioDisponivel();
void manterRTC();
String formatarData(uint32_t epoch);
String formatarHora(uint32_t epoch);
String pesoTexto(long p);
String hex2(uint8_t b);
void salvarProdutoAtivo();
void carregarProdutoAtivo();
void carregarListaProdutos();
bool salvarListaProdutos();
int acharProduto(const String &texto);
bool lerListaDoPayload(const String &json, String *saida, int &qtd);
void buscarProdutosFirebase();
void logProdutos();
void carregarBootId();
String limparTextoFila(String s);
String montarLinha(const Registro &r);
bool lerRegistro(const String &linha, Registro &r);
bool lerRegistroLegado(const String &linha, Registro &r);
void alimentarMotor13(long peso, long tara, bool sp3, bool instavel, bool simulado);
void lacunaMotor13();
void carregarCiclo13();
void manterPersistencia13();
bool espacoLivreSuficiente();
unsigned long carregarPosicaoFila();
bool salvarPosicaoFila(unsigned long novaPosicao);
unsigned long tamanhoFila();
void garantirFimDeLinha();
void recalcularPendencias();
void prepararFila();
void limparFilaSeFinalizada();
void avancarFila(unsigned long novaPosicao);
int lerProximaLinha(String &linha, unsigned long &fim);
void salvarUltimaPesagem(const String &linha);
void carregarUltimaPesagem();
void atualizarEspacoFs(bool forcar);
uint32_t capacidadeRegistros();
int nivelMemoria();
void registrarPerda();
void carregarPerdidas();
void salvarPerdidas();
bool registrarPesagem(Registro &r);
uint32_t resolverEpoch(const Registro &r);
const char* nomeCriterio(const Registro &r);
ResultadoEnvio enviarProximo();
void iniciarFirebase();
bool publicarIpWemos();
void processarRede();
void gerenciarConexao();
void verificarSaude();
bool lerNumeroAscii(const uint8_t *p, long &valor);
void receberByte(uint8_t b);
void processarPacote(const uint8_t *p, bool simulado);
void resetarLeitura();
void lerBalanca();
bool botaoRetornarPressionado();
void lerBotoes();
void iniciarSimulacao();
void processarSimulacao();
void verificarTimeoutTeste();
String textoSemAcento(String s);
void desenharProduto();
void desenharTelaPrincipal();
void invalidarBarra();
void atualizarIndicadorStatus(bool forcar);
void mostrarMensagem(const String &titulo, const String &linha1, const String &linha2, uint16_t cor);
void mostrarMensagemTemporaria(const String &titulo, const String &linha1, const String &linha2, uint16_t cor, unsigned long ms);
void verificarMensagemTemporaria();
const char* nomeErroOta(ota_error_t e);
void iniciarServicosRede();
void configurarOta();
void liberarOtaSobDemanda();
void logStatus();
void logLeitura();
void logHeartbeat();
void logAjuda();
void telnetComando(String cmd);
bool telnetFeed(int c);
void telnetAceitar();
void telnetProcessar();
void diagIniciar();
void diagAtualizar();
void diagRegistrarBoot();
void reiniciarComMotivo(const String &motivo);
void logReinicios();

// Monta o texto do log so quando ha alguem conectado no Telnet
#define TLOG(...) do { if (telnetPronto()) logln(__VA_ARGS__); } while (0)

// ============================================================================
// UTILITARIOS
// ============================================================================
bool epochValido(uint32_t t) { return t >= EPOCH_MIN && t < EPOCH_MAX; }
bool internetConectada() { return WiFi.status() == WL_CONNECTED; }

bool maquinaParada() {
  if (ciclo.ativo || simulacaoAtiva) return false;
  if (ultimaPesagemMillis == 0) return millis() > TEMPO_PARADA_MS;
  return millis() - ultimaPesagemMillis >= TEMPO_PARADA_MS;
}

String pesoTexto(long p) {
  if (p < 0) return String("-----");
  char b[12];
  snprintf(b, sizeof(b), "%05ld", p);
  return String(b);
}

String hex2(uint8_t b) {
  char s[3];
  snprintf(s, sizeof(s), "%02X", b);
  return String(s);
}

bool extrairStringJson(const String &json, const String &campo, String &valor) {
  String chave = "\"" + campo + "\":";
  int p = json.indexOf(chave);
  if (p < 0) return false;
  p += chave.length();
  while (p < (int)json.length() && json[p] == ' ') p++;
  if (p >= (int)json.length() || json[p] != '"') return false;
  p++;
  valor = "";
  bool escape = false;
  while (p < (int)json.length()) {
    char c = json[p++];
    if (escape) { valor += c; escape = false; continue; }
    if (c == '\\') { escape = true; continue; }
    if (c == '"') return true;
    valor += c;
  }
  return false;
}

bool extrairBoolJson(const String &json, const String &campo, bool &valor) {
  String chave = "\"" + campo + "\":";
  int p = json.indexOf(chave);
  if (p < 0) return false;
  p += chave.length();
  while (p < (int)json.length() && json[p] == ' ') p++;
  if (json.startsWith("true", p)) { valor = true; return true; }
  if (json.startsWith("false", p)) { valor = false; return true; }
  return false;
}

bool extrairNumeroJson(const String &json, const String &campo, unsigned long &valor) {
  String chave = "\"" + campo + "\":";
  int p = json.indexOf(chave);
  if (p < 0) return false;
  p += chave.length();
  while (p < (int)json.length() && json[p] == ' ') p++;
  int inicio = p;
  while (p < (int)json.length() && json[p] >= '0' && json[p] <= '9') p++;
  if (p == inicio) return false;
  valor = strtoul(json.substring(inicio, p).c_str(), nullptr, 10);
  return true;
}

String lerArquivoTexto(const char* caminho, size_t maximo) {
  String s;
  File f = LittleFS.open(caminho, "r");
  if (!f) return s;
  while (f.available() && s.length() < maximo) s += (char)f.read();
  f.close();
  s.trim();
  return s;
}

// ============================================================================
// RELOGIO: NTP > RTC DS3231 > sem hora. RTC guarda UTC; fuso -3 na formatacao.
// ============================================================================
void inicializarNTP() {
  configTime(-3 * 3600, 0, "pool.ntp.org", "time.google.com", "time.nist.gov");
}

uint32_t lerRTC() {
  if (!rtcOK || !rtcConfiavel) return 0;
  uint32_t t = rtc.now().unixtime();
  return epochValido(t) ? t : 0;
}

uint32_t obterEpochAtual() {
  uint32_t t = (uint32_t)time(nullptr);
  if (epochValido(t)) return t;
  return lerRTC();
}

bool relogioDisponivel() {
  return epochValido((uint32_t)time(nullptr)) || (rtcOK && rtcConfiavel);
}

void manterRTC() {
  static unsigned long ultimoAjuste = 0;
  static bool ajustouNestaSessao = false;
  static unsigned long ultimaBuscaRTC = 0;
  unsigned long agora = millis();

  if (!rtcOK) {
    if (agora - ultimaBuscaRTC < 60000UL) return;
    ultimaBuscaRTC = agora;
    marcarEtapa(ET_RTC_PROCURAR);
    rtcOK = rtc.begin(&Wire);
    if (rtcOK) rtcConfiavel = !rtc.lostPower();
    return;
  }
  if (ajustouNestaSessao && agora - ultimoAjuste < INTERVALO_AJUSTE_RTC) return;
  uint32_t ntp = (uint32_t)time(nullptr);
  if (!epochValido(ntp)) return;
  marcarEtapa(ET_RTC_LER);
  uint32_t r = rtc.now().unixtime();
  uint32_t diferenca = (r > ntp) ? (r - ntp) : (ntp - r);
  if (!rtcConfiavel || !epochValido(r) || diferenca > 2) {
    marcarEtapa(ET_RTC_GRAVAR);
    rtc.adjust(DateTime(ntp));
  }
  rtcConfiavel = true;
  ajustouNestaSessao = true;
  ultimoAjuste = agora;
}

String formatarData(uint32_t epoch) {
  if (!epochValido(epoch)) return "";
  time_t t = (time_t)epoch;
  struct tm info;
  localtime_r(&t, &info);
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d", info.tm_mday, info.tm_mon + 1, info.tm_year + 1900);
  return String(buffer);
}

String formatarHora(uint32_t epoch) {
  if (!epochValido(epoch)) return "";
  time_t t = (time_t)epoch;
  struct tm info;
  localtime_r(&t, &info);
  char buffer[32];
  snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d", info.tm_hour, info.tm_min, info.tm_sec);
  return String(buffer);
}

// ============================================================================
// PRODUTO / BOOT
// ============================================================================
// Guarda indice E texto: se a lista mudar de ordem no Firebase, o produto
// escolhido continua o mesmo (procurado pelo texto).
void salvarProdutoAtivo() {
  if (!littleFsOK) return;
  File f = LittleFS.open(ARQUIVO_PRODUTO, "w");
  if (!f) return;
  f.print(indiceAtual);
  f.print('\n');
  f.print(produtos[indiceAtual]);
  f.close();
}

void carregarProdutoAtivo() {
  indiceAtual = 0;
  if (!littleFsOK || !LittleFS.exists(ARQUIVO_PRODUTO)) return;
  String s = lerArquivoTexto(ARQUIVO_PRODUTO, 80);
  int nl = s.indexOf('\n');
  int idx = (nl >= 0 ? s.substring(0, nl) : s).toInt();
  String texto = nl >= 0 ? s.substring(nl + 1) : String("");
  texto.trim();
  int achado = texto.length() ? acharProduto(texto) : -1;
  if (achado >= 0) indiceAtual = achado;
  else if (idx >= 0 && idx < totalProdutos) indiceAtual = idx;
}

int acharProduto(const String &texto) {
  for (int i = 0; i < totalProdutos; i++) if (produtos[i] == texto) return i;
  return -1;
}

void carregarListaProdutos() {
  totalProdutos = 0;
  if (littleFsOK && LittleFS.exists(ARQUIVO_PRODUTOS)) {
    File f = LittleFS.open(ARQUIVO_PRODUTOS, "r");
    if (f) {
      while (f.available() && totalProdutos < MAX_PRODUTOS) {
        String l = f.readStringUntil('\n');
        l.trim();
        if (l.length()) produtos[totalProdutos++] = l;
      }
      f.close();
    }
    if (totalProdutos > 0) { origemProdutos = "memoria (ultimo 'atualizar')"; return; }
  }
  for (int i = 0; i < TOTAL_PRODUTOS_PADRAO; i++) produtos[i] = PRODUTOS_PADRAO[i];
  totalProdutos = TOTAL_PRODUTOS_PADRAO;
  origemProdutos = "codigo (lista padrao)";
}

bool salvarListaProdutos() {
  if (!littleFsOK) return false;
  File f = LittleFS.open(ARQUIVO_PRODUTOS, "w");
  if (!f) return false;
  for (int i = 0; i < totalProdutos; i++) { f.print(produtos[i]); f.print('\n'); }
  f.close();
  return true;
}

// Le a resposta do Firebase. Aceita objeto {"01":"A","02":"B"} (ordem pela
// chave, numerica quando a chave e numero) ou lista ["A","B"]. Ignora nulos.
bool lerListaDoPayload(const String &json, String *saida, int &qtd) {
  qtd = 0;
  String chaves[MAX_PRODUTOS];
  int p = 0, n = json.length();
  while (p < n && json[p] == ' ') p++;
  if (p >= n || (json[p] != '{' && json[p] != '[')) return false;
  bool objeto = json[p] == '{';
  p++;
  int indiceArray = 0;
  auto lerString = [&](String &out) -> bool {
    out = "";
    if (p >= n || json[p] != '"') return false;
    p++;
    while (p < n) {
      char c = json[p++];
      if (c == '\\' && p < n) {
        char e = json[p++];
        if (e == 'n' || e == 't' || e == 'r') out += ' ';
        else if (e == 'u') { p += 4; out += '?'; }
        else out += e;
      } else if (c == '"') return true;
      else out += c;
    }
    return false;
  };
  while (p < n) {
    while (p < n && (json[p] == ' ' || json[p] == ',' || json[p] == '\n' || json[p] == '\r')) p++;
    if (p >= n || json[p] == '}' || json[p] == ']') break;
    String chave;
    if (objeto) {
      if (!lerString(chave)) return false;
      while (p < n && (json[p] == ' ' || json[p] == ':')) p++;
    } else {
      chave = String(indiceArray++);
    }
    String valor;
    if (p < n && json[p] == '"') {
      if (!lerString(valor)) return false;
    } else {                       // null, numero, objeto: pula
      int nivel = 0;
      while (p < n) {
        char c = json[p];
        if (c == '{' || c == '[') nivel++;
        else if (c == '}' || c == ']') { if (nivel == 0) break; nivel--; }
        else if (c == ',' && nivel == 0) break;
        p++;
      }
      continue;
    }
    valor.trim();
    valor.replace(";", ",");
    if (valor.length() > (unsigned)TAM_MAX_PRODUTO) valor = valor.substring(0, TAM_MAX_PRODUTO);
    if (valor.length() == 0 || qtd >= MAX_PRODUTOS) continue;
    // insere ordenado pela chave (numerica se as duas forem numeros)
    int pos = qtd;
    for (int i = 0; i < qtd; i++) {
      bool numA = chave.length() && isDigit(chave[0]), numB = chaves[i].length() && isDigit(chaves[i][0]);
      bool menor = (numA && numB) ? (chave.toInt() < chaves[i].toInt()) : (chave < chaves[i]);
      if (menor) { pos = i; break; }
    }
    for (int i = qtd; i > pos; i--) { chaves[i] = chaves[i - 1]; saida[i] = saida[i - 1]; }
    chaves[pos] = chave;
    saida[pos] = valor;
    qtd++;
  }
  return true;
}

void buscarProdutosFirebase() {
  ultimaBuscaProdutos = millis();
  pedidoAtualizarProdutos = false;
  String caminho = String(FIREBASE_PATH_CONFIG) + "/produtos";
  if (!Firebase.RTDB.get(&fbdo, caminho.c_str())) {
    if (fbdo.httpCode() == FIREBASE_ERROR_PATH_NOT_EXIST) {
      // nao e erro: o no simplesmente nao foi criado
      produtosLidosNesteBoot = true;
      TLOG(F("Produtos: ") + caminho + F(" nao existe no Firebase. Usando lista: ") + origemProdutos);
      return;
    }
    TLOG(F("Produtos: falha ao ler ") + caminho + F(" HTTP=") + String(fbdo.httpCode()) + F(" ") + fbdo.errorReason() +
         F(" (mantendo lista atual)"));
    return;
  }
  produtosLidosNesteBoot = true;
  String tipo = fbdo.dataType();
  if (tipo == "null") {
    TLOG(F("Produtos: ") + caminho + F(" nao existe no Firebase (mantendo lista atual: ") + origemProdutos + F(")"));
    return;
  }
  String nova[MAX_PRODUTOS];
  int qtd = 0;
  if (!lerListaDoPayload(fbdo.payload(), nova, qtd) || qtd == 0) {
    TLOG(F("Produtos: formato nao reconhecido ou lista vazia em ") + caminho + F(" (tipo=") + tipo + F("), mantendo lista atual"));
    return;
  }

  bool igual = (qtd == totalProdutos);
  for (int i = 0; igual && i < qtd; i++) if (nova[i] != produtos[i]) igual = false;
  if (igual) { origemProdutos = "Firebase"; TLOG(F("Produtos: lista do Firebase sem alteracao (") + String(qtd) + F(")")); return; }

  String selecionado = produtos[indiceAtual];
  int novoIndice = -1;
  for (int i = 0; i < qtd; i++) if (nova[i] == selecionado) { novoIndice = i; break; }
  if (novoIndice < 0 && !maquinaParada()) {
    // o produto em uso foi removido/renomeado: so troca com a maquina parada
    TLOG(F("Produtos: produto em uso (") + selecionado + F(") nao esta na lista nova; aplica quando a maquina parar"));
    pedidoAtualizarProdutos = true;
    return;
  }

  for (int i = 0; i < qtd; i++) produtos[i] = nova[i];
  totalProdutos = qtd;
  origemProdutos = "Firebase";
  indiceAtual = novoIndice >= 0 ? novoIndice : 0;
  salvarListaProdutos();
  salvarProdutoAtivo();
  TLOG(F("Produtos: lista ATUALIZADA pelo Firebase (") + String(qtd) + F(" itens). Em uso: ") + produtos[indiceAtual]);
  if (novoIndice < 0) {
    mostrarMensagemTemporaria(F("LISTA ATUALIZADA"), F("Produto anterior saiu da lista"), F("CONFIRA O PRODUTO!"), 0xFFE0, 6000);
  } else if (telaPrincipalAtiva) {
    desenharTelaPrincipal();
  }
}

void logProdutos() {
  logln(F("Produtos (") + String(totalProdutos) + F(") origem: ") + origemProdutos);
  for (int i = 0; i < totalProdutos; i++)
    logln(String(i == indiceAtual ? F(" >> ") : F("    ")) + String(i + 1) + F(". ") + produtos[i]);
  logln(F("Editar em: ") + String(FIREBASE_PATH_CONFIG) + F("/produtos  (comando 'atualizar' le agora)"));
}

void carregarBootId() {
  bootId = 1;
  if (LittleFS.exists(ARQUIVO_BOOT)) {
    bootId = strtoul(lerArquivoTexto(ARQUIVO_BOOT, 12).c_str(), nullptr, 10) + 1;
    if (bootId == 0) bootId = 1;
  }
  File f = LittleFS.open(ARQUIVO_BOOT, "w");
  if (f) { f.print(bootId); f.close(); }
}

// ============================================================================
// LINHA DA FILA
// Novo formato (prefixo 4;):
// 4;epoch;boot;ms;teste;peso;estavel;criterio;pesoSp3;pesoMax;pesoEst1;
//   pesoEstU;tara;pacAteEst;pacSp3;msSp3;produto
// Continua lendo os formatos antigos 2; (real), 3; (teste) e JSON.
// ============================================================================
String limparTextoFila(String s) {
  s.replace(";", ",");
  s.replace("\n", " ");
  s.replace("\r", " ");
  return s;
}



bool lerRegistroLegado(const String &linha, Registro &r) {
  r = Registro();
  if (linha.startsWith("4;")) {
    const int N = 15;            // campos antes do produto (epoch ... msSp3)
    String c[N];
    int ini = 2;
    for (int i = 0; i < N; i++) {
      int p = linha.indexOf(';', ini);
      if (p < 0) return false;
      c[i] = linha.substring(ini, p);
      ini = p + 1;
    }
    r.produto   = linha.substring(ini);
    r.epoch     = strtoul(c[0].c_str(), nullptr, 10);
    r.boot      = strtoul(c[1].c_str(), nullptr, 10);
    r.ms        = strtoul(c[2].c_str(), nullptr, 10);
    r.teste     = (c[3] == "1");
    r.peso      = c[4];
    r.estavel   = (c[5] == "1");
    r.criterio  = c[6].length() ? c[6][0] : '?';
    r.pesoSp3   = c[7].toInt();
    r.pesoMax   = c[8].toInt();
    r.pesoEst1  = c[9].toInt();
    r.pesoEstU  = c[10].toInt();
    r.tara      = c[11].toInt();
    r.pacAteEst = strtoul(c[12].c_str(), nullptr, 10);
    r.pacSp3    = strtoul(c[13].c_str(), nullptr, 10);
    r.msSp3     = strtoul(c[14].c_str(), nullptr, 10);
    r.temDiag   = true;
    if (!epochValido(r.epoch)) r.epoch = 0;
    return r.peso.length() > 0;
  }
  if (linha.startsWith("2;") || linha.startsWith("3;")) {
    r.teste = linha.startsWith("3;");
    int p[5];
    int idx = -1;
    for (int i = 0; i < 5; i++) {
      idx = linha.indexOf(';', idx + 1);
      if (idx < 0) return false;
      p[i] = idx;
    }
    r.epoch   = strtoul(linha.substring(p[0] + 1, p[1]).c_str(), nullptr, 10);
    r.boot    = strtoul(linha.substring(p[1] + 1, p[2]).c_str(), nullptr, 10);
    r.ms      = strtoul(linha.substring(p[2] + 1, p[3]).c_str(), nullptr, 10);
    r.peso    = linha.substring(p[3] + 1, p[4]);
    r.produto = linha.substring(p[4] + 1);
    if (!epochValido(r.epoch)) r.epoch = 0;
    return r.peso.length() > 0;
  }
  if (linha.startsWith("{")) {
    if (!extrairStringJson(linha, "peso", r.peso)) return false;
    extrairStringJson(linha, "produto", r.produto);
    extrairBoolJson(linha, "teste", r.teste);
    bool horaOk = true;
    extrairBoolJson(linha, "horaOk", horaOk);
    unsigned long v = 0;
    if (horaOk && extrairNumeroJson(linha, "epoch", v) && epochValido(v)) r.epoch = v;
    if (extrairNumeroJson(linha, "eventoMillis", v)) r.ms = v;
    r.boot = 0;
    return true;
  }
  return false;
}

// ============================================================================
// FILA OFFLINE (LittleFS) V13
// V13 PERSISTENCE / QUEUE FRAGMENT. Replace corresponding v12 functions.
// Registro extras required:
// String id, chaveFirebase; bool v13=false, confiavel=false, estimado=false,
// lacuna=false, recuperado=false; uint8_t amostras=0; char qualidade='I';
// long limite=-1; uint32_t duracao=0;
// TAM_MAX_LINHA >= 768; BYTES_POR_REGISTRO_ESTIM >= 384.
// Keep original lerRegistro under name lerRegistroLegado; wrapper below calls it.
// registrarPesagem must NOT replace id/epoch/produto/quality frozen by checkpoint.
// Call prepararFila before recovering checkpoint or appending/sending records.
// No formatting LittleFS on mount errors. Missing/corrupt metadata fails closed.
// Assumption: LittleFS close commits, rename is atomic, filesystem is preserved.
// Complete flash erasure/replacement destroys identity continuity; do not reuse
// an erased chip namespace against the same database without explicit reprovision.

// BEGIN V13 QUEUE
static bool filaSegura13 = false;
static const char *ID13_A = "/id13.a", *ID13_B = "/id13.b";
static const char *POS13_A = "/pos13.a", *POS13_B = "/pos13.b";
static const char *FILA13_DONE = "/offline.done";

uint32_t crc32(const String &s) {
  uint32_t crc = 0xffffffffUL;
  for (size_t i = 0; i < s.length(); ++i) {
    crc ^= (uint8_t)s[i];
    for (uint8_t b = 0; b < 8; ++b) crc = (crc >> 1) ^ (0xedb88320UL & (0UL - (crc & 1UL)));
  }
  return ~crc;
}
static String hex32_13(uint32_t n) {
  char b[9]; snprintf(b, sizeof(b), "%08lX", (unsigned long)n); return String(b);
}
static String dec64_13(uint64_t n) {
  char b[21]; char *p = b + sizeof(b) - 1; *p = 0;
  do { *--p = '0' + n % 10; n /= 10; } while (n);
  return String(p);
}
static bool uint64_13(const String &s, uint64_t &n) {
  if (!s.length()) return false;
  n = 0;
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] < '0' || s[i] > '9') return false;
    unsigned d = s[i] - '0';
    if (n > (UINT64_MAX - d) / 10) return false;
    n = n * 10 + d;
  }
  return true;
}
static bool uint32_13(const String &s, uint32_t &n) {
  uint64_t v; if (!uint64_13(s, v) || v > UINT32_MAX) return false;
  n = (uint32_t)v; return true;
}
static bool long13(const String &s, long &n) {
  bool neg = s.startsWith("-"); uint64_t v;
  if (!uint64_13(neg ? s.substring(1) : s, v) || v > (neg ? 2147483648ULL : 2147483647ULL)) return false;
  n = neg ? (long)(-(int64_t)v) : (long)v; return true;
}
static bool bit13(const String &s, bool &v) {
  if (s != "0" && s != "1") return false;
  v = s == "1"; return true;
}
static bool checksum13(const String &line, String &payload) {
  int p = line.lastIndexOf(';');
  if (p < 0 || line.length() - p - 1 != 8) return false;
  payload = line.substring(0, p);
  return line.substring(p + 1) == hex32_13(crc32(payload));
}
static bool fields13(const String &payload, String *f, size_t count) {
  int at = 0;
  for (size_t i = 0; i < count; ++i) {
    int p = payload.indexOf(';', at);
    if (i == count - 1) { if (p >= 0) return false; f[i] = payload.substring(at); }
    else { if (p < 0) return false; f[i] = payload.substring(at, p); at = p + 1; }
  }
  return true;
}
static String encode13(const String &s) {
  const char *hex = "0123456789ABCDEF"; String out;
  for (size_t i = 0; i < s.length(); ++i) {
    uint8_t c = (uint8_t)s[i];
    if (c == '%' || c == ';' || c < 32 || c == 127) {
      out += '%'; out += hex[c >> 4]; out += hex[c & 15];
    } else out += (char)c;
  }
  return out;
}
static int nibble13(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}
static bool decode13(const String &s, String &out) {
  out = "";
  for (size_t i = 0; i < s.length(); ++i) {
    if (s[i] != '%') { out += s[i]; continue; }
    if (i + 2 >= s.length()) return false;
    int a = nibble13(s[i + 1]), b = nibble13(s[i + 2]);
    if (a < 0 || b < 0 || (a == 0 && b == 0)) return false;
    out += (char)((a << 4) | b); i += 2;
  }
  return true;
}
static bool chaveValida13(const String &s) {
  if (!s.length() || s.length() > 120) return false;
  for (size_t i = 0; i < s.length(); ++i) {
    uint8_t c = s[i];
    if (c < 32 || c == 127 || c == '.' || c == '#' || c == '$' || c == '[' || c == ']' || c == '/') return false;
  }
  return true;
}

static bool idValido13(const String &id) {
  if (!id.startsWith("e13_") || id.length() < 14 || id[12] != '_') return false;
  for (size_t i = 4; i < 12; ++i) if (nibble13(id[i]) < 0) return false;
  uint64_t n;
  return uint64_13(id.substring(13), n) && n > 0;
}
static bool pesoValido13(const String &peso) {
  if (peso.length() != 5) return false;
  for (size_t i = 0; i < 5; ++i) if (peso[i] < '0' || peso[i] > '9') return false;
  return true;
}

// A slot is valid only with its final newline, exact structure and CRC.
struct Slot13 { bool valid = false; uint64_t generation = 0, value = 0; int index = -1; };
// Explicit declarations keep Arduino's prototype generator after this type.
static Slot13 readSlot13(const char *path, int index);
static bool loadPair13(const char *a, const char *b, Slot13 &latest);
static bool readExact13(const char *path, String &out, size_t maxLen) {
  File f = LittleFS.open(path, "r"); if (!f) return false;
  size_t originalSize = f.size();
  if (originalSize == 0 || originalSize > maxLen) { f.close(); return false; }
  out = "";
  while (f.available()) { int c = f.read(); if (c < 0) { f.close(); return false; } out += (char)c; }
  f.close();
  if (out.length() != originalSize || !out.endsWith("\n")) return false;
  out.remove(out.length() - 1);
  return out.indexOf('\n') < 0 && out.indexOf('\r') < 0;
}
static Slot13 readSlot13(const char *path, int index) {
  Slot13 slot; String line, payload, f[3];
  if (!readExact13(path, line, 96) || !checksum13(line, payload) || !fields13(payload, f, 3) || f[0] != "S13" ||
      !uint64_13(f[1], slot.generation) || !uint64_13(f[2], slot.value)) return slot;
  slot.valid = true; slot.index = index; return slot;
}
static bool loadPair13(const char *a, const char *b, Slot13 &latest) {
  Slot13 sa = readSlot13(a, 0), sb = readSlot13(b, 1);
  if (!sa.valid && !sb.valid) {
    latest = Slot13();
    return !LittleFS.exists(a) && !LittleFS.exists(b); // existing corruption is NOT first boot
  }
  if (sa.valid && sb.valid && sa.generation == sb.generation && sa.value != sb.value) return false;
  latest = !sb.valid || (sa.valid && sa.generation >= sb.generation) ? sa : sb;
  return true;
}
static bool writePair13(const char *a, const char *b, uint64_t value) {
  Slot13 previous;
  if (!loadPair13(a, b, previous) || previous.generation == UINT64_MAX) return false;
  uint64_t generation = previous.generation + 1;
  int index = previous.index == 0 ? 1 : 0;
  const char *target = index == 0 ? a : b;
  String payload = "S13;" + dec64_13(generation) + ";" + dec64_13(value);
  String line = payload + ";" + hex32_13(crc32(payload)) + "\n";
  File f = LittleFS.open(target, "w"); if (!f) return false;
  size_t n = f.print(line); f.flush(); f.close();
  Slot13 verify = readSlot13(target, index);
  return n == line.length() && verify.valid && verify.generation == generation && verify.value == value;
}
bool reservarId(String &id) {
  id = ""; if (!littleFsOK) return false;
  Slot13 last;
  if (!loadPair13(ID13_A, ID13_B, last) || last.value == UINT64_MAX) return false;
  uint64_t next = last.value + 1;
  if (!writePair13(ID13_A, ID13_B, next)) return false;
  char chip[9]; snprintf(chip, sizeof(chip), "%08lX", (unsigned long)ESP.getChipId());
  id = String("e13_") + chip + "_" + dec64_13(next);
  return true;
}

String montarLinha(const Registro &r) {
  if (!r.v13 || !idValido13(r.id) || r.chaveFirebase != r.id) return "";
  String s = "5;" + encode13(r.id) + ";" + encode13(r.chaveFirebase);
  s += ";" + String(r.epoch) + ";" + String(r.boot) + ";" + String(r.ms);
  s += ";" + String(r.teste ? 1 : 0) + ";" + encode13(r.peso);
  s += ";" + String(r.temDiag ? 1 : 0) + ";" + String(r.estavel ? 1 : 0) + ";" + String(r.criterio);
  s += ";" + String(r.pesoSp3) + ";" + String(r.pesoMax) + ";" + String(r.pesoEst1) + ";" + String(r.pesoEstU);
  s += ";" + String(r.tara) + ";" + String(r.pacAteEst) + ";" + String(r.pacSp3) + ";" + String(r.msSp3);
  s += ";" + String(r.confiavel ? 1 : 0) + ";" + String(r.estimado ? 1 : 0) + ";" + String(r.lacuna ? 1 : 0);
  s += ";" + String(r.recuperado ? 1 : 0) + ";" + String(r.amostras) + ";" + String(r.qualidade);
  s += ";" + String(r.limite) + ";" + String(r.duracao) + ";" + encode13(r.produto);
  return s + ";" + hex32_13(crc32(s));
}

bool lerRegistro(const String &linha, Registro &r) {
  r = Registro();
  if (!linha.startsWith("5;")) {
    if (!lerRegistroLegado(linha, r)) return false;
    r.v13 = false;
    // Preserve explicit historical keys when provided. Never recompute them on retries.
    if (linha.startsWith("{")) {
      extrairStringJson(linha, "chaveFirebase", r.chaveFirebase);
      if (!r.chaveFirebase.length()) extrairStringJson(linha, "chave", r.chaveFirebase);
      if (!r.chaveFirebase.length()) extrairStringJson(linha, "key", r.chaveFirebase);
      extrairStringJson(linha, "id", r.id);
    }
    if (!r.chaveFirebase.length()) r.chaveFirebase = r.epoch ? String(r.epoch) : "semhora_" + String(r.boot) + "_" + String(r.ms);
    return chaveValida13(r.chaveFirebase);
  }
  String payload, c[28]; uint32_t samples;
  if (!checksum13(linha, payload) || !fields13(payload, c, 28) || c[0] != "5") return false;
  if (!decode13(c[1], r.id) || !decode13(c[2], r.chaveFirebase) || !idValido13(r.id) || r.id != r.chaveFirebase) return false;
  if (!uint32_13(c[3], r.epoch) || !uint32_13(c[4], r.boot) || !uint32_13(c[5], r.ms) || !bit13(c[6], r.teste) || !decode13(c[7], r.peso) ||
      !bit13(c[8], r.temDiag) || !bit13(c[9], r.estavel) || c[10].length() != 1) return false;
  r.criterio = c[10][0];
  if (!long13(c[11], r.pesoSp3) || !long13(c[12], r.pesoMax) || !long13(c[13], r.pesoEst1) || !long13(c[14], r.pesoEstU) || !long13(c[15], r.tara) ||
      !uint32_13(c[16], r.pacAteEst) || !uint32_13(c[17], r.pacSp3) || !uint32_13(c[18], r.msSp3) || !bit13(c[19], r.confiavel) ||
      !bit13(c[20], r.estimado) || !bit13(c[21], r.lacuna) || !bit13(c[22], r.recuperado) || !uint32_13(c[23], samples) || samples > 255 ||
      c[24].length() != 1 || (c[24] != "S" && c[24] != "M" && c[24] != "I") || !long13(c[25], r.limite) ||
      !uint32_13(c[26], r.duracao) || !decode13(c[27], r.produto)) return false;
  r.amostras = samples; r.qualidade = c[24][0]; r.v13 = true;
  if (r.epoch && !epochValido(r.epoch)) return false;
  return pesoValido13(r.peso) && (r.confiavel != r.estimado) && (!r.confiavel || r.qualidade == 'S');
}

// Preserve raw damaged bytes in rejected file and verify before destructive change.
static bool arquivarTrecho13(unsigned long start, unsigned long end) {
  if (end < start) return false;
  File src = LittleFS.open(ARQUIVO_FILA, "r");
  File dst = LittleFS.open(ARQUIVO_REJEITADOS, "a");
  if (!src || !dst || end > src.size() || !src.seek(start, SeekSet)) { src.close(); dst.close(); return false; }
  unsigned long offset = dst.size(), remaining = end - start;
  uint8_t buf[128]; bool ok = true;
  while (remaining) {
    size_t want = remaining < sizeof(buf) ? remaining : sizeof(buf);
    int n = src.read(buf, want);
    if (n != (int)want || dst.write(buf, want) != want) { ok = false; break; }
    remaining -= want; yield();
  }
  if (ok) ok = dst.print('\n') == 1;
  dst.flush(); dst.close(); src.close();
  if (!ok) return false;
  src = LittleFS.open(ARQUIVO_FILA, "r"); dst = LittleFS.open(ARQUIVO_REJEITADOS, "r");
  if (!src || !dst || !src.seek(start, SeekSet) || !dst.seek(offset, SeekSet)) { src.close(); dst.close(); return false; }
  remaining = end - start;
  while (remaining--) { int a = src.read(), b = dst.read(); if (a < 0 || a != b) { ok = false; break; } if ((remaining & 127UL) == 0) yield(); }
  src.close(); dst.close(); return ok;
}

unsigned long carregarPosicaoFila() {
  Slot13 last;
  if (!loadPair13(POS13_A, POS13_B, last) || last.value > UINT32_MAX) { filaSegura13 = false; return 0; }
  if (last.valid) return (unsigned long)last.value;
  // Import only the existing local cursor; never rewrite historical queue records.
  const char *legacy = LittleFS.exists(ARQUIVO_POSICAO) ? ARQUIVO_POSICAO : (LittleFS.exists(ARQUIVO_POSICAO_TEMP) ? ARQUIVO_POSICAO_TEMP : nullptr);
  if (!legacy) return 0;
  String text = lerArquivoTexto(legacy, 32); uint32_t v;
  if (!uint32_13(text, v)) { filaSegura13 = false; return 0; }
  return v;
}
bool salvarPosicaoFila(unsigned long value) {
  return littleFsOK && writePair13(POS13_A, POS13_B, value);
}

// Rename to a tombstone BEFORE resetting cursor. At boot/append, finish this
// protocol before creating a new queue, preventing an old offset skipping data.
static bool concluirLimpeza13() {
  if (!LittleFS.exists(FILA13_DONE)) return true;
  if (LittleFS.exists(ARQUIVO_FILA)) return false;
  if (!salvarPosicaoFila(0)) return false;
  posicaoFila = 0; pendenciasCache = 0;
  return LittleFS.remove(FILA13_DONE);
}
static bool repararCauda13() {
  File f = LittleFS.open(ARQUIVO_FILA, "r"); if (!f) return !LittleFS.exists(ARQUIVO_FILA);
  unsigned long size = f.size(), complete = size;
  if (size) {
    if (!f.seek(size - 1, SeekSet)) { f.close(); return false; }
    if (f.read() != '\n') {
      complete = 0; unsigned long at = size;
      while (at > 0) { --at; if ((at & 127UL) == 0) yield(); if (!f.seek(at, SeekSet)) { f.close(); return false; } if (f.read() == '\n') { complete = at + 1; break; } }
    }
  }
  f.close();
  if (complete == size) return true;
  if (!arquivarTrecho13(complete, size)) return false;
  f = LittleFS.open(ARQUIVO_FILA, "r+"); if (!f) return false;
  bool ok = f.truncate(complete); f.flush(); f.close();
  return ok && tamanhoFila() == complete;
}
void garantirFimDeLinha() { if (!repararCauda13()) filaSegura13 = false; }
void prepararFila() {
  filaSegura13 = littleFsOK;
  if (!filaSegura13 || !concluirLimpeza13()) { filaSegura13 = false; return; }
  posicaoFila = carregarPosicaoFila(); if (!filaSegura13) return;
  if (!repararCauda13()) { filaSegura13 = false; return; }
  unsigned long size = tamanhoFila();
  if (!LittleFS.exists(ARQUIVO_FILA)) {
    if (!salvarPosicaoFila(0)) { filaSegura13 = false; return; }
    posicaoFila = 0; pendenciasCache = 0; return;
  }
  if (posicaoFila > size) { filaSegura13 = false; return; }
  if (posicaoFila) {
    File f = LittleFS.open(ARQUIVO_FILA, "r");
    bool boundary = f && f.seek(posicaoFila - 1, SeekSet) && f.read() == '\n'; f.close();
    if (!boundary) { filaSegura13 = false; return; }
  }
  recalcularPendencias();
}
void limparFilaSeFinalizada() {
  if (!filaSegura13 || !LittleFS.exists(ARQUIVO_FILA) || posicaoFila != tamanhoFila()) return;
  if (LittleFS.exists(FILA13_DONE) || !LittleFS.rename(ARQUIVO_FILA, FILA13_DONE) || !concluirLimpeza13()) { filaSegura13 = false; return; }
  memoriaCheia = false;
}
void avancarFila(unsigned long value) {
  if (!filaSegura13 || !salvarPosicaoFila(value)) { filaSegura13 = false; return; }
  posicaoFila = value;
  if (pendenciasCache > 0) --pendenciasCache;
  limparFilaSeFinalizada();
  if (memoriaCheia && nivelMemoria() < 2) memoriaCheia = false;
}
int lerProximaLinha(String &line, unsigned long &end) {
  line = ""; end = posicaoFila;
  if (!filaSegura13) return 0;
  File f = LittleFS.open(ARQUIVO_FILA, "r");
  if (!f) { if (LittleFS.exists(ARQUIVO_FILA)) filaSegura13 = false; return 0; }
  if (posicaoFila >= f.size()) { f.close(); return 0; }
  if (!f.seek(posicaoFila, SeekSet)) { f.close(); filaSegura13 = false; return 0; }
  unsigned long originalSize = f.size();
  bool longLine = false, newline = false;
  while (f.available()) {
    int c = f.read(); if (c < 0) break; ++end;
    if (c == '\n') { newline = true; break; }
    if (line.length() < TAM_MAX_LINHA) line += (char)c; else longLine = true;
  }
  f.close();
  if (!newline && end < originalSize) { filaSegura13 = false; return 0; }
  // No EOF-as-commit. Legacy lines are accepted only with their original newline.
  return newline && !longLine ? 1 : 2;
}
bool filaContemId(const String &id, bool &integridade) {
  integridade = filaSegura13; bool found = false;
  if (!integridade) return false;
  File f = LittleFS.open(ARQUIVO_FILA, "r");
  if (!f) { integridade = !LittleFS.exists(ARQUIVO_FILA); return false; }
  String line; bool over = false; unsigned long readBytes = 0, originalSize = f.size();
  while (f.available()) {
    int c = f.read(); if (c < 0) { integridade = false; break; }
    ++readBytes;
    if (c == '\n') {
      if (line.endsWith("\r")) line.remove(line.length() - 1);
      Registro r;
      if (over || !lerRegistro(line, r)) integridade = false;
      else if (r.v13 && r.id == id) found = true;
      line = ""; over = false;
    } else if (line.length() < TAM_MAX_LINHA) line += (char)c; else over = true;
    yield();
  }
  if (line.length() || over || readBytes != originalSize) integridade = false;
  f.close(); return found;
}
bool filaPersistirRegistro(const Registro &r) {
  if (!filaSegura13 || !r.v13 || !concluirLimpeza13()) return false;
  String line = montarLinha(r); Registro verify;
  if (!line.length() || line.length() > TAM_MAX_LINHA || !lerRegistro(line, verify)) return false;
  if ((uint32_t)pendenciasCache >= LIMITE_REGISTROS_PENDENTES) return false;
  atualizarEspacoFs(true);
  if (fsLivreCache <= RESERVA_FS + line.length() + 1) return false;
  // Startup repairs only incomplete tails, preserving complete historical bytes.
  if (!repararCauda13()) { filaSegura13 = false; return false; }
  unsigned long start = tamanhoFila();
  File f = LittleFS.open(ARQUIVO_FILA, "a"); if (!f) return false;
  size_t n = f.print(line);
  // Never seal a partial append: the boot repair quarantines an uncommitted tail.
  if (n == line.length()) n += f.print('\n');
  f.flush(); f.close();
  if (n != line.length() + 1) { filaSegura13 = false; return false; }
  f = LittleFS.open(ARQUIVO_FILA, "r");
  if (!f || f.size() != start + n || !f.seek(start, SeekSet)) { f.close(); filaSegura13 = false; return false; }
  bool ok = true;
  for (size_t i = 0; i < line.length(); ++i) if (f.read() != (uint8_t)line[i]) { ok = false; break; }
  if (ok) ok = f.read() == '\n'; f.close();
  if (!ok) { filaSegura13 = false; return false; }
  ++pendenciasCache; return true;
}

// END V13 QUEUE

// In enviarProximo replace resolverEpoch/key logic with r.epoch and
// r.chaveFirebase. No NTP waiting and no timestamp/key reconstruction on retry.
// The complete replacement below keeps the original diagnostic field names.
// Compare the immutable queued payload after a conditional-create conflict.
// Extra historical fields are allowed; every queued field must match exactly.
static bool mesmoPayload13(FirebaseJson &expected, FirebaseJson &actual) {
  static const char *const keys[] = {
    "data", "hora", "peso", "produto", "teste", "id", "epoch", "confiavel",
    "estimado", "qualidade", "lacuna", "recuperado", "amostras", "limite", "duracao_ms",
    "estavel", "criterio", "peso_kg", "diag/peso_quando_sp3_ligou", "diag/peso_maximo",
    "diag/peso_estavel_primeiro", "diag/peso_estavel_ultimo", "diag/tara",
    "diag/pacotes_ate_estavel", "diag/pacotes_sp3_ligado", "diag/ms_ciclo", "diag/firmware"
  };
  for (size_t i = 0; i < sizeof(keys) / sizeof(keys[0]); ++i) {
    FirebaseJsonData a, b;
    expected.get(a, keys[i]); if (!a.success) continue;
    actual.get(b, keys[i]); if (!b.success) return false;
    if (a.type == "string") { if (b.type != "string" || a.to<String>() != b.to<String>()) return false; }
    else if (a.type == "boolean" || a.type == "bool") { if ((b.type != "boolean" && b.type != "bool") || a.to<bool>() != b.to<bool>()) return false; }
    else {
      if ((b.type != "int" && b.type != "float" && b.type != "double") || a.to<double>() != b.to<double>()) return false;
    }
  }
  return true;
}

ResultadoEnvio enviarProximo() {
  if (!filaSegura13) return ENVIO_FALHOU;
  String line; unsigned long end = 0;
  int st = lerProximaLinha(line, end);
  if (!filaSegura13) return ENVIO_FALHOU;
  if (st == 0) { limparFilaSeFinalizada(); return ENVIO_NADA; }
  if (line.endsWith("\r")) line.remove(line.length() - 1);
  Registro r;
  if (st == 2 || !lerRegistro(line, r)) {
    if (!arquivarTrecho13(posicaoFila, end)) return ENVIO_FALHOU;
    ++registrosCorrompidos; avancarFila(end);
    return filaSegura13 ? ENVIO_OK : ENVIO_FALHOU;
  }
  uint32_t epoch = r.epoch;
  String caminho = String(r.teste ? FIREBASE_PATH_TESTES : FIREBASE_PATH_PESAGENS) + "/" + r.chaveFirebase;
  FirebaseJson json;
  json.set("data", formatarData(epoch)); json.set("hora", formatarHora(epoch));
  json.set("peso", r.peso); json.set("produto", r.produto); json.set("teste", r.teste);
  if (r.v13) {
    json.set("id", r.id); json.set("epoch", (double)r.epoch);
    json.set("confiavel", r.confiavel); json.set("estimado", r.estimado);
    json.set("qualidade", String(r.qualidade)); json.set("lacuna", r.lacuna);
    json.set("recuperado", r.recuperado); json.set("amostras", (int)r.amostras);
    json.set("limite", r.limite); json.set("duracao_ms", (double)r.duracao);
  }
  if (r.temDiag) {
    json.set("estavel", r.estavel); json.set("criterio", nomeCriterio(r));
    if (CASAS_DECIMAIS >= 0) { double div = 1.0; for (int i = 0; i < CASAS_DECIMAIS; ++i) div *= 10.0; json.set("peso_kg", (double)r.peso.toInt() / div); }
    json.set("diag/peso_quando_sp3_ligou", (int)r.pesoSp3); json.set("diag/peso_maximo", (int)r.pesoMax);
    json.set("diag/peso_estavel_primeiro", (int)r.pesoEst1); json.set("diag/peso_estavel_ultimo", (int)r.pesoEstU);
    json.set("diag/tara", (int)r.tara); json.set("diag/pacotes_ate_estavel", (int)r.pacAteEst);
    json.set("diag/pacotes_sp3_ligado", (int)r.pacSp3); json.set("diag/ms_ciclo", (int)r.msSp3);
    json.set("diag/firmware", r.v13 ? (r.criterio == 'D' ? "E2-RS485-SP3EST-v13" : "E2-RS485-SP3EST-v13.1") : "E2-RS485-SP3EST-v12");
  }
  // Create only if absent. Never overwrite any existing historical record.
  bool confirmed = Firebase.RTDB.setJSON(&fbdo, caminho.c_str(), &json, "null_etag");
  if (!confirmed && fbdo.httpCode() == 412) {
    if (!r.v13) {
      TLOG(F("CONFLITO LEGADO: chave existente preservada; fila pausada em ") + caminho);
      return ENVIO_FALHOU;
    }
    FirebaseJson existing;
    confirmed = Firebase.RTDB.getJSON(&fbdo, caminho.c_str(), &existing) && mesmoPayload13(json, existing);
    if (!confirmed) {
      TLOG(F("CONFLITO V13: payload existente diferente ou leitura falhou; fila preservada em ") + caminho);
      return ENVIO_FALHOU;
    }
  }
  if (confirmed) {
    avancarFila(end); // RAM cursor advances only after verified metadata commit.
    falhasSeguidas = 0; ultimoEnvioOk = millis();
    TLOG(F("ENVIADO ") + caminho + F(" peso=") + r.peso + F(" pendentes=") + String(pendenciasCache));
    if (aguardandoResultadoTeste && r.teste) {
      aguardandoResultadoTeste = false;
      mostrarMensagemTemporaria(F("TESTE ENVIADO!"), caminho, F("Pendentes: ") + String(pendenciasCache), 0x07E0, 4000);
    }
    return filaSegura13 ? ENVIO_OK : ENVIO_FALHOU;
  }
  if (falhasSeguidas < 255) ++falhasSeguidas;
  TLOG(F("FALHA NO ENVIO ") + caminho + F(" HTTP=") + String(fbdo.httpCode()) + F(" motivo=") + fbdo.errorReason());
  if (aguardandoResultadoTeste && r.teste) {
    aguardandoResultadoTeste = false;
    mostrarMensagemTemporaria(F("ERRO NO ENVIO"), fbdo.errorReason(), F("HTTP ") + String(fbdo.httpCode()), 0xF800, 6000);
  }
  return ENVIO_FALHOU;
}

// FILA OFFLINE (LittleFS) - mesma logica da versao de 23/10
// ============================================================================
bool espacoLivreSuficiente() {
  atualizarEspacoFs(true);
  return fsLivreCache > RESERVA_FS + BYTES_POR_REGISTRO_ESTIM;
}

void atualizarEspacoFs(bool forcar) {
  if (!littleFsOK) { fsLivreCache = 0; return; }
  if (!forcar && ultimaLeituraFs != 0 && millis() - ultimaLeituraFs < 30000UL) return;
  FSInfo info;
  if (LittleFS.info(info)) {
    fsTotalCache = info.totalBytes;
    fsLivreCache = info.totalBytes > info.usedBytes ? info.totalBytes - info.usedBytes : 0;
  }
  ultimaLeituraFs = millis();
}

// Quantos sacos podem ficar esperando envio, no total (o menor dos dois limites)
uint32_t capacidadeRegistros() {
  uint32_t pend = (uint32_t)pendenciasCache;
  uint32_t livres = fsLivreCache > RESERVA_FS ? (fsLivreCache - RESERVA_FS) / BYTES_POR_REGISTRO_ESTIM : 0;
  uint32_t porEspaco = pend + livres;
  return porEspaco < LIMITE_REGISTROS_PENDENTES ? porEspaco : LIMITE_REGISTROS_PENDENTES;
}

// 0 = normal, 1 = aviso (>= 80%), 2 = cheia (nao grava mais)
int nivelMemoria() {
  if (!littleFsOK) return 2;
  uint32_t cap = capacidadeRegistros();
  uint32_t pend = (uint32_t)pendenciasCache;
  if (cap == 0 || pend >= cap) return 2;
  if ((uint64_t)pend * 100ULL >= (uint64_t)cap * PERCENT_AVISO_MEMORIA) return 1;
  return 0;
}

void carregarPerdidas() {
  if (!littleFsOK || !LittleFS.exists(ARQUIVO_PERDIDAS)) return;
  pesagensPerdidas = strtoul(lerArquivoTexto(ARQUIVO_PERDIDAS, 12).c_str(), nullptr, 10);
}

void salvarPerdidas() {
  if (!littleFsOK) return;
  if (pesagensPerdidas == 0) { LittleFS.remove(ARQUIVO_PERDIDAS); return; }
  File f = LittleFS.open(ARQUIVO_PERDIDAS, "w");   // arquivo minusculo: cabe mesmo com a fila cheia
  if (f) { f.print(pesagensPerdidas); f.close(); }
}

// Saco que NAO coube na memoria: conta, guarda o contador e avisa na tela
void registrarPerda() {
  memoriaCheia = true;
  pesagensPerdidas++;
  salvarPerdidas();
  TLOG(F("SACO NAO GRAVADO: memoria de registros cheia (pendentes=") + String(pendenciasCache) +
       F(", perdidos=") + String(pesagensPerdidas) + F(")"));
  mostrarMensagemTemporaria(F("MEMORIA CHEIA!"), F("Saco NAO foi gravado. Sem internet ha muito tempo."),
                            F("Nao gravados: ") + String(pesagensPerdidas) + F("  Chame o responsavel"), 0xF800, 5000);
}





unsigned long tamanhoFila() {
  File f = LittleFS.open(ARQUIVO_FILA, "r");
  if (!f) return 0;
  unsigned long t = f.size();
  f.close();
  return t;
}



void recalcularPendencias() {
  pendenciasCache = 0;
  File f = LittleFS.open(ARQUIVO_FILA, "r");
  if (!f) return;
  if (posicaoFila > f.size()) posicaoFila = 0;
  f.seek(posicaoFila, SeekSet);
  uint8_t buf[256];
  bool linhaComDados = false;
  while (true) {
    int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    for (int i = 0; i < n; i++) {
      if (buf[i] == '\n') {
        if (linhaComDados) pendenciasCache++;
        linhaComDados = false;
      } else if (buf[i] > ' ') {
        linhaComDados = true;
      }
    }
    yield();
  }
  if (linhaComDados) pendenciasCache++;
  f.close();
}







// 0 = nada, 1 = linha ok, 2 = linha corrompida


void salvarUltimaPesagem(const String &linha) {
  File f = LittleFS.open(ARQUIVO_ULTIMA, "w");
  if (!f) return;
  f.print(linha);
  f.close();
}

void carregarUltimaPesagem() {
  if (!LittleFS.exists(ARQUIVO_ULTIMA)) return;
  Registro r;
  if (!lerRegistro(lerArquivoTexto(ARQUIVO_ULTIMA, TAM_MAX_LINHA), r)) return;
  ultimaPesagemPeso = r.peso;
  ultimaPesagemData = formatarData(r.epoch);
  ultimaPesagemHora = r.epoch ? formatarHora(r.epoch) : String("SEM HORA");
  ultimaPesagemEstavel = r.temDiag ? r.estavel : true;
  ultimaPesagemTeste = r.teste;
  ultimoEpochGravado = r.epoch;
}

// Grava o saco na fila. Completa epoch/boot/ms/produto.
bool registrarPesagem(Registro &r) {
  ultimaPesagemMillis = millis();
  if (!littleFsOK || !r.v13 || !r.id.length()) return false;
  // Epoch, produto e ID foram congelados na abertura do ciclo e no checkpoint.
  // A rotina verifica CRC, newline, flush/close e releitura antes de retornar.
  if (!filaPersistirRegistro(r)) {
    memoriaCheia = ((uint32_t)pendenciasCache >= LIMITE_REGISTROS_PENDENTES || !espacoLivreSuficiente());
    TLOG(F("FILA V13: evento pendente no checkpoint; resolver armazenamento antes de novo ciclo"));
    return false;
  }
  memoriaCheia = false;
  if (r.epoch) ultimoEpochGravado = r.epoch;
  totalRegistrados++;
  if (!r.teste) { if (r.estavel) sacosEstaveis++; else sacosSemEstavel++; }
  ultimaPesagemPeso = r.peso;
  ultimaPesagemData = formatarData(r.epoch);
  ultimaPesagemHora = r.epoch ? formatarHora(r.epoch) : String("SEM HORA");
  ultimaPesagemEstavel = r.estavel;
  ultimaPesagemTeste = r.teste;
  salvarUltimaPesagem(montarLinha(r));
  return true;
}

// ============================================================================
// ENVIO PARA O FIREBASE
// ============================================================================
uint32_t resolverEpoch(const Registro &r) {
  if (epochValido(r.epoch)) return r.epoch;
  if (r.boot != 0 && r.boot == bootId) {
    uint32_t agora = obterEpochAtual();
    if (agora) return agora - (uint32_t)((millis() - r.ms) / 1000UL);
  }
  return 0;
}

const char* nomeCriterio(const Registro &r) {
  if (r.v13) {
    if (r.criterio == 'L') return "indicador_saturado_50kg_estimado";
    if (r.qualidade == 'S') return r.confiavel ? "descarga_ultimo_estavel" : "descarga_estavel_estimado";
    if (r.qualidade == 'M') return "descarga_mediana_tres";
    return "descarga_peso_nao_confiavel";
  }
  if (r.criterio == 'T') return r.estavel ? "tempo_maximo_com_estavel" : "tempo_maximo_sem_estavel";
  return r.estavel ? "sp3_estavel" : "sp3_sem_estavel_peso_maximo";
}



// ADAPTADOR V13
// BEGIN V13 ADAPTER
// LittleFS alterna dois checkpoints. O evento so e liberado apos fila confirmada
// E checkpoint de acknowledge confirmado. Nao grava firmware nem chama Firebase.
struct MetaCiclo13 {
  char id[48];
  char produto[64];
  uint32_t epoch;
  uint32_t boot;
  uint32_t ms;
  int32_t tara;
  uint8_t teste;
  uint8_t recuperado; // flags META13_*; layout binario preservado da v13.
};
struct Checkpoint13 {
  uint32_t magic;
  uint32_t tamanho;
  uint32_t geracao;
  E2V13::State estado;
  MetaCiclo13 meta;
  uint32_t crc;
};
bool lerCheckpoint13(const char *path, Checkpoint13 &cp);
// Flags no byte ja existente: preserva o tamanho/layout dos checkpoints v13.
// A revisao tambem fica congelada por ciclo, mantendo retry de filas antigas.
static const uint8_t META13_RECUPERADO = 0x01;
static const uint8_t META13_SATURADO50 = 0x02;
static const uint8_t META13_REVISAO_1 = 0x04;
static const uint8_t META13_INICIO_SATURADO = 0x08;
const long PESO_SATURADO_INDICADOR13 = 99999;
const long PESO_EQUIVALENTE_50KG13 = 500; // 50,0 kg; estimativa solicitada pelo usuario.
static E2V13::Engine motor13;
static MetaCiclo13 meta13 = {};
static uint32_t geracaoCheckpoint13 = 0;
static uint32_t revisaoSalva13 = 0;
static bool checkpointSeguro13 = true;
static bool conferirFila13 = false;
static bool eventoNaFila13 = false;
static bool drenarEspaco13 = false;
static bool reparoCheckpoint13 = false;
static unsigned long tentativaPersistir13 = 0;
static unsigned long pacoteMotor13 = 0;
static unsigned long ultimoAvisoPersistencia13 = 0;
static const char *CP13_A = "/ciclo13.a", *CP13_B = "/ciclo13.b";
static const uint32_t MAGIC_CP13 = 0x45324313UL;

uint32_t crcBinario13(const uint8_t *p, size_t n) {
  uint32_t crc = 0xffffffffUL;
  while (n--) {
    crc ^= *p++;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xedb88320UL & (0UL - (crc & 1UL)));
  }
  return ~crc;
}
bool lerCheckpoint13(const char *path, Checkpoint13 &cp) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  bool ok = f.size() == sizeof(cp) && f.read((uint8_t *)&cp, sizeof(cp)) == (int)sizeof(cp);
  f.close();
  if (!ok || cp.magic != MAGIC_CP13 || cp.tamanho != sizeof(cp) ||
      cp.crc != crcBinario13((const uint8_t *)&cp, offsetof(Checkpoint13, crc))) return false;
  return memchr(cp.meta.id, 0, sizeof(cp.meta.id)) && memchr(cp.meta.produto, 0, sizeof(cp.meta.produto));
}
void avisarPersistencia13() {
  if (ultimoAvisoPersistencia13 && millis() - ultimoAvisoPersistencia13 < 10000UL) return;
  ultimoAvisoPersistencia13 = millis();
  TLOG(F("PERSISTENCIA V13: ciclo retido; pausar maquina ate normalizar armazenamento"));
  mostrarMensagemTemporaria(F("PAUSAR ENSAQUE"), F("Falha/memoria cheia: evento retido"),
                            F("Normalizar LittleFS antes de novos sacos"), 0xF800, 8000);
}
bool salvarCheckpoint13() {
  if (!littleFsOK || !checkpointSeguro13 || geracaoCheckpoint13 == UINT32_MAX) return false;
  Checkpoint13 cp = {};
  cp.magic = MAGIC_CP13; cp.tamanho = sizeof(cp); cp.geracao = geracaoCheckpoint13 + 1;
  cp.estado = motor13.snapshot(); cp.meta = meta13;
  cp.crc = crcBinario13((const uint8_t *)&cp, offsetof(Checkpoint13, crc));
  const char *target = (cp.geracao & 1) ? CP13_A : CP13_B;
  File f = LittleFS.open(target, "w");
  if (!f) return false;
  size_t n = f.write((const uint8_t *)&cp, sizeof(cp)); f.flush(); f.close();
  Checkpoint13 verify;
  if (n != sizeof(cp) || !lerCheckpoint13(target, verify) ||
      memcmp(&cp, &verify, sizeof(cp)) != 0) return false;
  geracaoCheckpoint13 = cp.geracao;
  revisaoSalva13 = motor13.state().logicalRevision;
  reparoCheckpoint13 = false;
  return true;
}

// ESCALA_LIMITE13 e independente de CASAS_DECIMAIS (que permanece como na v12).
// Confirmar uma casa decimal no indicador antes de qualquer gravacao em campo.
// Ex.: 25KG -> raw nominal250, limite100; 50KG ->500/200; 40KG ->400/160.
const uint16_t ESCALA_LIMITE13 = 10;
const uint8_t PERCENTUAL_DESCARGA13 = 40;
const int32_t LIMITE_DESCARGA_PADRAO13 = 100;
struct AjusteProduto13 { const char *codigo; int32_t limiteRaw; };
const AjusteProduto13 AJUSTES_DESCARGA13[] = {
  {"598", 100}, {"121", 100}, {"19", 200}, {"5270", 200}, {"5375", 100}, {"17800", 160}
};
int32_t limiteProduto13(const String &produto) {
  int sep = produto.indexOf(" - ");
  String codigo = sep > 0 ? produto.substring(0, sep) : "";
  for (size_t i = 0; i < sizeof(AJUSTES_DESCARGA13) / sizeof(AJUSTES_DESCARGA13[0]); ++i)
    if (codigo == AJUSTES_DESCARGA13[i].codigo) return AJUSTES_DESCARGA13[i].limiteRaw;
  int kg = produto.lastIndexOf("KG");
  if (kg > 0) {
    int ini = kg;
    while (ini > 0 && produto[ini - 1] >= '0' && produto[ini - 1] <= '9') --ini;
    long nominal = produto.substring(ini, kg).toInt();
    if (nominal > 0 && nominal <= 1000) return nominal * ESCALA_LIMITE13 * PERCENTUAL_DESCARGA13 / 100;
  }
  return LIMITE_DESCARGA_PADRAO13;
}
void sincronizarCiclo13() {
  ciclo.ativo = motor13.state().active || motor13.pending() || reparoCheckpoint13 || !checkpointSeguro13;
  ciclo.simulado = meta13.teste;
  ciclo.numero = motor13.state().cycleNumber;
}
void carregarCiclo13() {
  Checkpoint13 a, b;
  bool va = lerCheckpoint13(CP13_A, a), vb = lerCheckpoint13(CP13_B, b);
  if (!va && !vb) {
    checkpointSeguro13 = !LittleFS.exists(CP13_A) && !LittleFS.exists(CP13_B);
    sincronizarCiclo13(); return;
  }
  const Checkpoint13 &cp = !vb || (va && a.geracao >= b.geracao) ? a : b;
  geracaoCheckpoint13 = cp.geracao;
  checkpointSeguro13 = motor13.restore(cp.estado);
  if (!checkpointSeguro13) { sincronizarCiclo13(); return; }
  meta13 = cp.meta;
  revisaoSalva13 = motor13.state().logicalRevision;
  if (motor13.state().active || motor13.pending()) {
    meta13.recuperado |= META13_RECUPERADO;
    // Nunca juntar uma baixa antes da queda de energia com outra apos o boot.
    // Evento ja confirmado conserva integralmente o payload e sua qualidade.
    if (!motor13.pending()) { motor13.invalidate(); motor13.rebaseClock(millis()); }
    conferirFila13 = true;
    reparoCheckpoint13 = true;
  } else {
    motor13.reset(); // boot ocioso aprende descarga antes de aceitar nova borda.
    meta13 = MetaCiclo13();
  }
  sincronizarCiclo13();
}
void preencherRegistro13(Registro &r) {
  const E2V13::Event &e = motor13.result();
  r = Registro(); r.v13 = true;
  r.id = meta13.id; r.chaveFirebase = r.id;
  r.epoch = meta13.epoch; r.boot = meta13.boot; r.ms = meta13.ms;
  r.produto = meta13.produto; r.teste = meta13.teste;
  r.temDiag = true; r.criterio = (meta13.recuperado & META13_REVISAO_1) ? 'd' : 'D'; r.estavel = e.stable;
  r.peso = pesoTexto(e.raw); r.pesoSp3 = e.startRaw; r.pesoMax = e.maximumRaw;
  r.pesoEst1 = e.firstStableRaw; r.pesoEstU = e.lastStableRaw;
  r.tara = meta13.tara; r.pacSp3 = e.sp3Readings; r.pacAteEst = e.packetsToStable;
  r.msSp3 = e.durationMs; r.duracao = e.durationMs;
  r.limite = motor13.state().config.lowLimitRaw;
  r.lacuna = e.gap; r.recuperado = (meta13.recuperado & META13_RECUPERADO) != 0;
  r.amostras = e.readings > 255 ? 255 : e.readings;
  r.qualidade = e.quality == E2V13::Quality::Stable ? 'S' : (e.quality == E2V13::Quality::Median ? 'M' : 'I');
  r.confiavel = e.stable && !e.ambiguous && !e.gap && !r.recuperado;
  r.estimado = !r.confiavel;
  if (meta13.recuperado & META13_SATURADO50) {
    // 99999 e um codigo de saturacao, nao 9999,9 kg. A conversao vale para
    // o ciclo inteiro, por regra operacional; nunca declara uma medicao exata.
    r.peso = pesoTexto(PESO_EQUIVALENTE_50KG13);
    r.criterio = 'L'; r.qualidade = 'I';
    r.estimado = true; r.confiavel = false; r.estavel = false;
    r.pesoMax = PESO_SATURADO_INDICADOR13; // preserva o maior valor recebido.
    if (meta13.recuperado & META13_INICIO_SATURADO) r.pesoSp3 = PESO_SATURADO_INDICADOR13;
  }
}
void manterPersistencia13() {
  if (!littleFsOK || !checkpointSeguro13) { sincronizarCiclo13(); return; }
  if (!filaSegura13) {
    prepararFila(); // retry local apos escrita curta; preserva/arquiva cauda antes de truncar.
    if (!filaSegura13) { reparoCheckpoint13 = true; avisarPersistencia13(); sincronizarCiclo13(); return; }
    conferirFila13 = true;
  }
  if ((motor13.state().active || motor13.pending()) && meta13.id[0] == 0) {
    // Preserva o ciclo ANTES de reservar o contador. Um corte entre os dois
    // commits deixa um ciclo recuperavel ainda sem ID, nunca um ID transmitido.
    if (!salvarCheckpoint13()) { reparoCheckpoint13 = true; avisarPersistencia13(); sincronizarCiclo13(); return; }
    String id;
    if (!reservarId(id) || id.length() >= sizeof(meta13.id)) { reparoCheckpoint13 = true; avisarPersistencia13(); sincronizarCiclo13(); return; }
    id.toCharArray(meta13.id, sizeof(meta13.id));
    reparoCheckpoint13 = true;
  }
  if (reparoCheckpoint13 || motor13.state().logicalRevision != revisaoSalva13) {
    if (!salvarCheckpoint13()) { reparoCheckpoint13 = true; avisarPersistencia13(); sincronizarCiclo13(); return; }
  }
  if (!motor13.pending()) { sincronizarCiclo13(); return; }
  if (!eventoNaFila13 && conferirFila13) {
    bool integridade;
    eventoNaFila13 = filaContemId(meta13.id, integridade);
    if (!integridade) { reparoCheckpoint13 = true; avisarPersistencia13(); sincronizarCiclo13(); return; }
    conferirFila13 = false;
  }
  if (!eventoNaFila13) {
    Registro r; preencherRegistro13(r);
    atualizarEspacoFs(true);
    const size_t bytes = montarLinha(r).length() + 1;
    if ((uint32_t)pendenciasCache >= LIMITE_REGISTROS_PENDENTES || fsLivreCache <= RESERVA_FS + bytes) {
      // Ainda nao tentou append: e seguro drenar registros anteriores para
      // liberar espaco. Nunca enviar evento com acknowledge local indefinido.
      memoriaCheia = true; drenarEspaco13 = true;
      avisarPersistencia13(); sincronizarCiclo13(); return;
    }
    drenarEspaco13 = false;
    if (!registrarPesagem(r)) {
      conferirFila13 = true; reparoCheckpoint13 = true; avisarPersistencia13(); sincronizarCiclo13(); return;
    }
    eventoNaFila13 = true;
    TLOG(F("CICLO13;id=") + r.id + F(";peso=") + r.peso + F(";qualidade=") + String(r.qualidade) +
         F(";estimado=") + String(r.estimado ? 1 : 0) + F(";descarga=2;fila=") + String(pendenciasCache));
    if (r.teste) { aguardandoResultadoTeste = true; inicioEsperaTeste = millis(); }
  }
  // Se houver corte aqui, boot encontra checkpoint pending e o mesmo ID na fila.
  E2V13::State pendente = motor13.snapshot();
  motor13.acknowledge();
  if (!salvarCheckpoint13()) {
    motor13.restore(pendente); reparoCheckpoint13 = true; sincronizarCiclo13(); return;
  }
  eventoNaFila13 = false; conferirFila13 = false; drenarEspaco13 = false;
  meta13 = MetaCiclo13();
  sincronizarCiclo13();
}
void alimentarMotor13(long peso, long tara, bool sp3, bool instavel, bool simulado) {
  if (!checkpointSeguro13) return;
  const bool saturado50 = !simulado && peso == PESO_SATURADO_INDICADOR13;
  const long pesoEfetivo = saturado50 ? PESO_EQUIVALENTE_50KG13 : peso;
  unsigned long agora = millis();
  if (pacoteMotor13 && (uint32_t)(agora - pacoteMotor13) > 2000UL) motor13.invalidate();
  pacoteMotor13 = agora;
  if (!motor13.state().active && !motor13.pending()) {
    E2V13::Config cfg = E2V13::defaultConfig();
    cfg.lowLimitRaw = simulado ? LIMITE_DESCARGA_PADRAO13 : limiteProduto13(produtos[indiceAtual]);
    if (motor13.state().config.lowLimitRaw != cfg.lowLimitRaw) motor13.configure(cfg);
  }
  E2V13::Step step = motor13.feed(pesoEfetivo, sp3, instavel || saturado50, agora);
  if (step == E2V13::Step::Started) {
    meta13 = MetaCiclo13();
    String produto = limparTextoFila(produtos[indiceAtual]);
    produto.toCharArray(meta13.produto, sizeof(meta13.produto));
    meta13.epoch = obterEpochAtual(); meta13.boot = bootId; meta13.ms = agora;
    meta13.teste = simulado; meta13.tara = tara;
    meta13.recuperado = META13_REVISAO_1;
    if (saturado50) meta13.recuperado |= META13_INICIO_SATURADO;
    eventoNaFila13 = false; conferirFila13 = false;
    reparoCheckpoint13 = true;
  }
  if (saturado50 && motor13.state().active && !(meta13.recuperado & META13_SATURADO50)) {
    meta13.recuperado |= META13_SATURADO50 | META13_REVISAO_1;
    reparoCheckpoint13 = true; // a primeira saturacao deve sobreviver ao reboot.
  }
  if (motor13.state().active && tara >= 0) meta13.tara = tara;
  sincronizarCiclo13();
  manterPersistencia13();
}
void lacunaMotor13() {
  motor13.invalidate();
  sincronizarCiclo13();
  // Sem ciclo, nao desgastar flash por ruido no cabo.
  if (motor13.state().active) manterPersistencia13();
}
// END V13 ADAPTER

void iniciarFirebase() {
  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  auth.user.email = FIREBASE_EMAIL;
  auth.user.password = FIREBASE_SENHA;
  // Igual a Ensacadeira 1: usa os valores padrao da biblioteca
  // (buffer SSL 2048/512, resposta 2048, tempos de espera padrao).
  diagRtc.heapAntesLogin = ESP.getFreeHeap();
  diagRtc.blocoAntesLogin = ESP.getMaxFreeBlockSize();
  ESP.rtcUserMemoryWrite(DIAG_OFFSET, (uint32_t*)&diagRtc, sizeof(diagRtc));
  TLOG(F("Iniciando Firebase (login). Memoria livre=") + String(diagRtc.heapAntesLogin) +
       F(" maior bloco=") + String(diagRtc.blocoAntesLogin));
  marcarEtapa(ET_FB_BEGIN);
  Firebase.begin(&config, &auth);
  marcarEtapa(ET_FB_DEPOIS_BEGIN);
  Firebase.reconnectWiFi(true);
  firebaseIniciado = true;
  TLOG(F("Firebase iniciado. Memoria livre=") + String(ESP.getFreeHeap()));
}

bool publicarIpWemos() {
  ultimaTentativaIpFirebase = millis();
  String ipAtual = WiFi.localIP().toString();
  String caminho = String(FIREBASE_PATH_IPS_WEMOS) + "/" + FIREBASE_ID_DISPOSITIVO;
  FirebaseJson json;
  json.set("ip", ipAtual);
  json.set("mac", WiFi.macAddress());
  json.set("rssi_dbm", WiFi.RSSI());
  json.set("boot", (int)bootId);
  json.set("firmware", FW_VERSAO);
  json.set("telnet", ipAtual + ":" + String(TELNET_PORTA));
  json.set("produto_ativo", produtos[indiceAtual]);
  json.set("ultimo_reset", ultimoResetTexto);
  json.set("pendentes_sem_envio", pendenciasCache);
  json.set("sacos_nao_gravados", (int)pesagensPerdidas);
  json.set("tempo_ligado_segundos", (double)(millis() / 1000UL));
  json.set("atualizado_em/.sv", "timestamp");
  if (Firebase.RTDB.updateNode(&fbdo, caminho.c_str(), &json)) {
    ipFirebasePendente = false;
    ipFirebasePublicado = true;
    TLOG(F("IP atualizado no Firebase: ") + caminho + F(" ip=") + ipAtual);
    return true;
  }
  TLOG(F("Falha ao atualizar IP: HTTP=") + String(fbdo.httpCode()) + F(" motivo=") + fbdo.errorReason());
  return false;
}

// Rede so quando NAO ha ciclo de SP3 em andamento. Os bytes da balanca que
// chegarem enquanto o envio trava ficam no buffer da serial (1 KB = ~15 s)
// e sao processados em ordem logo depois: nenhum pacote se perde.
void processarRede() {
  marcarEtapa(ET_REDE);
  if (otaAtiva && !USAR_OTA) return;   // OTA liberada: Firebase pausado ate reiniciar
  if (!internetConectada()) { firebaseProntoCache = false; return; }
  if ((ciclo.ativo && !drenarEspaco13) || simulacaoAtiva || !filaSegura13 || !checkpointSeguro13) return;

  bool parada = maquinaParada();
  unsigned long intervalo = parada ? INTERVALO_REDE_PARADA : INTERVALO_REDE_PRODUCAO;
  if (millis() - ultimaOperacaoRede < intervalo) return;
  ultimaOperacaoRede = millis();

  if (!firebaseIniciado) { iniciarFirebase(); return; }
  if ((long)(millis() - proximaTentativaFirebase) < 0) return;

  marcarEtapa(ET_FB_READY);
  firebaseProntoCache = Firebase.ready();
  if (!firebaseProntoCache) {
    proximaTentativaFirebase = millis() + 20000UL;
    TLOG(F("Firebase NAO pronto: ") + String(Firebase.authTokenInfo().error.message.c_str()) + F(" (nova tentativa em 20 s)"));
    return;
  }

  // IP: publicado logo que conecta (fora de ciclo de SP3), sem esperar parada
  if (ipFirebasePendente &&
      (ultimaTentativaIpFirebase == 0 || millis() - ultimaTentativaIpFirebase >= 30000UL)) {
    marcarEtapa(ET_FB_IP);
    publicarIpWemos();
    return;
  }

  // Produtos: SOMENTE quando pedido pelo comando "atualizar" no Telnet.
  // (Se o produto em uso saiu da lista, a troca espera a maquina parar e
  //  tenta de novo a cada 30 s ate conseguir.)
  bool podeRepetir = (ultimaBuscaProdutos == 0) || (millis() - ultimaBuscaProdutos >= 30000UL);
  if (pedidoAtualizarProdutos && podeRepetir) {
    marcarEtapa(ET_FB_PRODUTOS);
    buscarProdutosFirebase();
    return;
  }

  if (!LittleFS.exists(ARQUIVO_FILA)) return;
  marcarEtapa(ET_FB_ENVIO);
  enviarProximo();

  if (falhasSeguidas >= 10) {
    TLOG(F("10 falhas seguidas: reiniciando o Wi-Fi"));
    falhasSeguidas = 0;
    firebaseProntoCache = false;
    WiFi.disconnect();
    delay(20);
    WiFi.begin(WIFI_SSID, WIFI_SENHA);
    ultimaTentativaWiFi = millis();
  }
}

void gerenciarConexao() {
  static bool estavaConectado = false;
  unsigned long agora = millis();

  if (internetConectada()) {
    IPAddress ipAtual = WiFi.localIP();
    bool novaConexao = !estavaConectado;
    if (novaConexao) {
      estavaConectado = true;
      momentoWifiConectou = agora;
      iniciarServicosRede();
      TLOG(F("WiFi conectado: ") + ipAtual.toString());
    }
    if (novaConexao || ipAtual != ultimoIpDetectado) {
      ultimoIpDetectado = ipAtual;
      ipFirebasePendente = true;
      ultimaTentativaIpFirebase = 0;
    }
    ultimoTempoConectado = agora;
    return;
  }

  if (estavaConectado) ipFirebasePendente = true;
  estavaConectado = false;
  firebaseProntoCache = false;

  if (agora - ultimaTentativaWiFi >= INTERVALO_RECONEXAO_WIFI) {
    WiFi.begin(WIFI_SSID, WIFI_SENHA);
    ultimaTentativaWiFi = agora;
  }
  if (agora - ultimoTempoConectado >= TIMEOUT_WIFI_SEM_CONEXAO) {
    WiFi.disconnect();
    delay(20);
    WiFi.begin(WIFI_SSID, WIFI_SENHA);
    ultimoTempoConectado = agora;
    ultimaTentativaWiFi = agora;
  }
}

void verificarSaude() {
  static unsigned long ultimaVerificacao = 0;
  static uint8_t leiturasRuins = 0;
  if (millis() - ultimaVerificacao < 10000UL) return;
  ultimaVerificacao = millis();

  // so considera critico de verdade (antes 5000/2500 podia reiniciar sem necessidade)
  bool heapRuim = ESP.getFreeHeap() < 3000 || ESP.getMaxFreeBlockSize() < 1500;
  if (heapRuim) { if (leiturasRuins < 255) leiturasRuins++; }
  else leiturasRuins = 0;

  bool envioTravado = internetConectada() && pendenciasCache > 0 &&
                      ultimoEnvioOk != 0 && millis() - ultimoEnvioOk > REINICIO_SEM_ENVIO_MS;

  if (maquinaParada() && (leiturasRuins >= 6 || envioTravado)) {
    reiniciarComMotivo(envioTravado ? String("saude: 30 min sem enviar")
                                    : "saude: memoria " + String(ESP.getFreeHeap()) + "/" + String(ESP.getMaxFreeBlockSize()));
  }
}

// ============================================================================
// LEITURA DA BALANCA - T02 STD (15 bytes)
//  [0]=02 STX  [1]=STS1  [2]=STS2  [3..7]=peso ASCII  [8..12]=tara ASCII
//  [13]=03 ETX [14]=BCC (XOR dos bytes 0..13)
// ============================================================================
bool lerNumeroAscii(const uint8_t *p, long &valor) {
  long v = 0;
  for (int i = 0; i < 5; i++) {
    if (p[i] < '0' || p[i] > '9') return false;
    v = v * 10 + (p[i] - '0');
  }
  valor = v;
  return true;
}

// Monta pacotes byte a byte. Se o pacote vier errado, procura o proximo 02
// dentro dos bytes ja recebidos (ressincroniza sem perder o pacote seguinte).
void receberByte(uint8_t b) {
  if (rxPos == 0 && b != 0x02) return;
  rxBuf[rxPos++] = b;
  if (rxPos < 15) return;

  uint8_t bcc = 0;
  for (int i = 0; i < 14; i++) bcc ^= rxBuf[i];
  bool ok = (rxBuf[13] == 0x03) && (!VALIDAR_BCC || rxBuf[14] == bcc);

  if (ok) {
    rxPos = 0;
    processarPacote(rxBuf, false);
    return;
  }

  pacotesInvalidos++;
  lacunaMotor13();
  uint8_t prox = 0;
  for (uint8_t i = 1; i < 15; i++) if (rxBuf[i] == 0x02) { prox = i; break; }
  if (prox == 0) { rxPos = 0; return; }
  uint8_t resto = 15 - prox;
  memmove(rxBuf, rxBuf + prox, resto);
  rxPos = resto;
}

void processarPacote(const uint8_t *p, bool simulado) {
  uint8_t sts1 = p[1];
  uint8_t sts2 = p[2];
  bool instavel = (sts1 >> BIT_INSTAVEL_STS1) & 1;
  bool sp3 = (sts2 >> BIT_SP3_STS2) & 1;
  long peso = -1, tara = -1;
  bool pesoOk = lerNumeroAscii(p + 3, peso);
  lerNumeroAscii(p + 8, tara);

  if (!simulado) {
    pacotesValidos++;
    memcpy(ultimoPacote, p, 15);
    ultimoPacoteMs = millis();
  }

  // Monitor "pacotes": uma linha CSV por pacote (para salvar no log do PuTTY)
  if (monitorPacotes && telnetPronto()) {
    logln(F("PKT;") + String(millis()) + F(";") + (pesoOk ? pesoTexto(peso) : String(F("?????"))) + F(";") +
          pesoTexto(tara) + F(";") + hex2(sts1) + F(";") + hex2(sts2) + F(";") + (instavel ? F("0") : F("1")) + F(";") +
          (sp3 ? F("1") : F("0")) + F(";") + (ciclo.ativo ? String(ciclo.numero) : String(F("-"))) + (simulado ? F(";SIM") : F("")));
  }
  // Monitor "eventos": so quando STS1 ou STS2 mudam
  if (monitorEventos && telnetPronto() && (sts1 != sts1Anterior || sts2 != sts2Anterior)) {
    logln(F("EVT ") + String(simulado ? F("(SIM) ") : F("")) + F("STS1 ") + hex2(sts1Anterior) + F("->") + hex2(sts1) +
          F(" STS2 ") + hex2(sts2Anterior) + F("->") + hex2(sts2) + F(" | peso=") + (pesoOk ? pesoTexto(peso) : String(F("?????"))) +
          (instavel ? F(" INSTAVEL") : F(" ESTAVEL")) + (sp3 ? F(" SP3=1") : F(" SP3=0")));
  }
  sts1Anterior = sts1;
  sts2Anterior = sts2;

  if (!pesoOk) { if (!simulado) pacotesPesoInvalido++; lacunaMotor13(); return; }

  pesoAtual = peso;
  instavelAtual = instavel;
  sp3Atual = sp3;

  alimentarMotor13(peso, tara, sp3, instavel, simulado);
}

void resetarLeitura() {
  rxPos = 0;
  pacoteMotor13 = 0;
  if (!motor13.state().active && !motor13.pending()) motor13.reset();
  // Jamais apagar ciclo restaurado ou evento aguardando armazenamento.
  sincronizarCiclo13();
}

void lerBalanca() {
  if (Serial.hasOverrun()) { overrunsSerial++; lacunaMotor13(); }
  int lidos = 0;
  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    bytesRecebidos++;
    ultimoByteMs = millis();

    if (debugBytesFim != 0 && (long)(millis() - debugBytesFim) < 0 && telnetPronto()) {
      linhaCrua += hex2(b);
      linhaCrua += ' ';
      if (b == 0x03 || linhaCrua.length() >= 48) { logln(F("RX ") + linhaCrua); linhaCrua = ""; }
    }

    if (simulacaoAtiva) continue;   // durante o teste os bytes reais sao descartados
    receberByte(b);
    if (++lidos >= 64) { lidos = 0; yield(); }
  }
}

// ============================================================================
// BOTOES
// ============================================================================
bool botaoRetornarPressionado() { return digitalRead(PINO_RETORNAR) == HIGH; }

void lerBotoes() {
  static bool avancarAcionado = false, retornarAcionado = false, travarAteSoltar = false;
  static unsigned long tempoAvancar = 0, tempoRetornar = 0, tempoAmbos = 0;

  bool av = (digitalRead(PINO_AVANCAR) == LOW);
  bool rt = botaoRetornarPressionado();

  if (av && rt) {
    tempoAvancar = 0; tempoRetornar = 0;
    if (tempoAmbos == 0) tempoAmbos = millis();
    if (!travarAteSoltar && millis() - tempoAmbos >= 3000) {
      iniciarSimulacao();
      travarAteSoltar = true;
    }
    return;
  }
  tempoAmbos = 0;

  if (travarAteSoltar) {
    if (!av && !rt) travarAteSoltar = false;
    avancarAcionado = true; retornarAcionado = true;
    tempoAvancar = 0; tempoRetornar = 0;
    return;
  }

  if (av) {
    if (tempoAvancar == 0) tempoAvancar = millis();
    if (!avancarAcionado && millis() - tempoAvancar >= 700) {
      indiceAtual = (indiceAtual + 1) % totalProdutos;
      salvarProdutoAtivo();
      desenharTelaPrincipal();
      TLOG(F("Produto: ") + produtos[indiceAtual]);
      avancarAcionado = true;
    }
  } else { tempoAvancar = 0; avancarAcionado = false; }

  if (rt) {
    if (tempoRetornar == 0) tempoRetornar = millis();
    if (!retornarAcionado && millis() - tempoRetornar >= 700) {
      indiceAtual = (indiceAtual - 1 + totalProdutos) % totalProdutos;
      salvarProdutoAtivo();
      desenharTelaPrincipal();
      TLOG(F("Produto: ") + produtos[indiceAtual]);
      retornarAcionado = true;
    }
  } else { tempoRetornar = 0; retornarAcionado = false; }
}

// ============================================================================
// SIMULACAO: gera pacotes T02 validos (com ETX e BCC) que imitam um saco:
// enchendo instavel -> SP3 liga -> estabiliza em 99999 -> SP3 desliga -> zero.
// Passa pela MESMA logica dos pacotes reais. Grava em /testes_ensacadeira_2.
// ============================================================================
void iniciarSimulacao() {
  if (simulacaoAtiva) return;
  if (ciclo.ativo) {
    TLOG(F("Teste recusado: ha um ciclo real de SP3 em andamento"));
    mostrarMensagemTemporaria(F("TESTE RECUSADO"), F("Ciclo real em andamento"), F(""), 0xFFE0, 3000);
    return;
  }
  resetarLeitura();
  simulacaoAtiva = true;
  simulacaoPasso = 0;
  simulacaoUltimo = 0;
  aguardandoResultadoTeste = false;
  TLOG(F("SIMULACAO iniciada (99999 em /testes_ensacadeira_2)"));
  mostrarMensagem(F("MODO TESTE"), F("Simulando saco 99999"), F("Vai para /testes_ensacadeira_2"), 0xFFE0);
}

void processarSimulacao() {
  if (!simulacaoAtiva) return;
  if (millis() - simulacaoUltimo < 250) return;
  simulacaoUltimo = millis();

  // passo: peso, instavel, sp3
  static const long  PESO[] = {0, 0, 25000, 50000, 80000, 99990, 99999, 99999, 99999, 99999, 99999, 0, 0, 0, 0};
  static const bool  INST[] = {0, 0, 1,     1,     1,     1,     1,     0,     0,     0,     0,     1, 1, 0, 0};
  static const bool  SP3[]  = {0, 0, 0,     0,     0,     1,     1,     1,     1,     1,     0,     0, 0, 0, 0};
  const uint8_t TOTAL = sizeof(PESO) / sizeof(PESO[0]);

  if (simulacaoPasso >= TOTAL) {
    simulacaoAtiva = false;
    resetarLeitura();
    while (Serial.available() > 0) Serial.read();
    Serial.hasOverrun();
    TLOG(F("SIMULACAO terminada"));
    if (!aguardandoResultadoTeste && !mensagemTemporariaAtiva) desenharTelaPrincipal();
    return;
  }

  uint8_t pk[15];
  char d[6];
  pk[0] = 0x02;
  pk[1] = INST[simulacaoPasso] ? 0x91 : 0x81;
  pk[2] = SP3[simulacaoPasso] ? 0x0F : 0x09;
  snprintf(d, sizeof(d), "%05ld", PESO[simulacaoPasso]);
  for (int i = 0; i < 5; i++) pk[3 + i] = d[i];
  for (int i = 0; i < 5; i++) pk[8 + i] = '0';
  pk[13] = 0x03;
  uint8_t bcc = 0;
  for (int i = 0; i < 14; i++) bcc ^= pk[i];
  pk[14] = bcc;
  simulacaoPasso++;
  processarPacote(pk, true);
}

void verificarTimeoutTeste() {
  if (!aguardandoResultadoTeste) return;
  if (millis() - inicioEsperaTeste < TIMEOUT_RESULTADO_TESTE) return;
  aguardandoResultadoTeste = false;
  if (!internetConectada())
    mostrarMensagemTemporaria(F("NAO ENVIOU"), F("Sem Wi-Fi"), F("Ficou na memoria"), 0xF800, 6000);
  else if (!firebaseProntoCache)
    mostrarMensagemTemporaria(F("FIREBASE NAO PRONTO"), String(Firebase.authTokenInfo().error.message.c_str()), F("Ficou na memoria"), 0xF800, 6000);
  else
    mostrarMensagemTemporaria(F("NAO ENVIOU"), F("Fila nao processada"), F("Pendentes: ") + String(pendenciasCache), 0xF800, 6000);
}

// ============================================================================
// DISPLAY ILI9488 480x320 (rotacao 1)
//  y   0- 14  linha pequena: PRODUTO n/total  COD xxx  (avisos a direita)
//  y  16-262  nome do produto, o maior possivel (tamanho ate 12)
//  y 268-320  barra: bolinha + ONLINE/SEM FIREBASE/SEM INTERNET, hora,
//             FILA: n e IP. Amarela/vermelha quando a memoria de registros
//             passa de 80% / enche.
// ============================================================================
#define COR_FUNDO   0x0000
#define COR_TEXTO   0xFFFF
#define COR_CINZA   0x8410
#define COR_VERDE   0x07E0
#define COR_AMARELO 0xFFE0
#define COR_VERMELHO 0xF800
#define COR_LARANJA 0xFD20
#define COR_AZUL    0x5D9F

String textoSemAcento(String s) {
  const char* de[]   = {"á","à","ã","â","é","ê","í","ó","ô","õ","ú","ç","Á","À","Ã","Â","É","Ê","Í","Ó","Ô","Õ","Ú","Ç"};
  const char* para[] = {"a","a","a","a","e","e","i","o","o","o","u","c","A","A","A","A","E","E","I","O","O","O","U","C"};
  for (int i = 0; i < 24; i++) s.replace(de[i], para[i]);
  return s;
}

void desenharProduto() {
  String produto = produtos[indiceAtual];
  int sep = produto.indexOf(" - ");
  String codigo = sep >= 0 ? produto.substring(0, sep) : String("");
  if (sep >= 0) produto = produto.substring(sep + 3);
  produto = textoSemAcento(produto);
  produto.trim();
  if (produto.length() == 0) produto = "SEM PRODUTO";

  const uint8_t MAXP = 10;
  String pal[MAXP];
  uint8_t np = 0;
  int ini = 0;
  while (ini < (int)produto.length() && np < MAXP) {
    while (ini < (int)produto.length() && produto[ini] == ' ') ini++;
    if (ini >= (int)produto.length()) break;
    int fim = produto.indexOf(' ', ini);
    if (fim < 0) fim = produto.length();
    pal[np++] = produto.substring(ini, fim);
    ini = fim + 1;
  }

  const int margem = 8, topo = 4;
  const int areaTopo = 16, areaFim = 262;          // produto ocupa y 16..262
  const int alturaMax = areaFim - areaTopo;
  const int larguraMax = (int)tft->width() - 2 * margem;
  String linhas[MAXP];
  uint8_t nl = 0;
  int tam = 1;

  for (int t = 12; t >= 1; t--) {
    int cw = 6 * t, ch = 8 * t, gapT = t > 4 ? t : 4;
    String tmp[MAXP];
    uint8_t n = 0;
    bool cabe = true;
    for (uint8_t i = 0; i < np; i++) {
      if ((int)pal[i].length() * cw > larguraMax) { cabe = false; break; }
      if (n == 0) { tmp[n++] = pal[i]; continue; }
      String tentativa = tmp[n - 1] + " " + pal[i];
      if ((int)tentativa.length() * cw <= larguraMax) tmp[n - 1] = tentativa;
      else if (n < MAXP) tmp[n++] = pal[i];
      else { cabe = false; break; }
    }
    if (!cabe) continue;
    if (n * ch + (n - 1) * gapT <= alturaMax) {
      tam = t; nl = n;
      for (uint8_t i = 0; i < n; i++) linhas[i] = tmp[i];
      break;
    }
  }
  if (nl == 0) { linhas[0] = produto; nl = 1; tam = 1; }

  tft->setTextWrap(false);
  tft->setTextSize(1);
  tft->setTextColor(COR_CINZA);
  tft->setCursor(margem, topo);
  tft->print(F("PRODUTO ") + String(indiceAtual + 1) + F("/") + String(totalProdutos) + F("  COD ") + codigo);

  int ch = 8 * tam;
  int gap = tam > 4 ? tam : 4;
  int alturaTotal = nl * ch + (nl - 1) * gap;
  int y = areaTopo + (alturaMax - alturaTotal) / 2;
  tft->setTextSize(tam);
  tft->setTextColor(COR_TEXTO);
  for (uint8_t i = 0; i < nl; i++) {
    int x = ((int)tft->width() - (int)linhas[i].length() * 6 * tam) / 2;
    if (x < margem) x = margem;
    tft->setCursor(x, y);
    tft->print(linhas[i]);
    y += ch + gap;
    yield();
  }
}

void desenharTelaPrincipal() {
  mensagemTemporariaAtiva = false;
  telaPrincipalAtiva = true;
  if (!displayOK) return;
  tft->fillScreen(COR_FUNDO);
  desenharProduto();
  atualizarIndicadorStatus(true);
}

// Barra de status: y 268..320. Cada campo e redesenhado sozinho, so quando muda.
//  linha 1 (y 276): bolinha + ONLINE/SEM FIREBASE/SEM INTERNET ...... hora
//  linha 2 (y 298):            FILA: n .............................. IP
const int BARRA_Y = 268;

void invalidarBarra() {
  barraNivel = -1; barraConexao = -1; barraHora = ""; barraFila = -1;
  barraIp = ""; barraAviso = "#"; barraAlerta = "";
}

void atualizarIndicadorStatus(bool forcar) {
  if (!displayOK || !telaPrincipalAtiva || mensagemTemporariaAtiva) return;
  if (forcar) invalidarBarra();
  atualizarEspacoFs(false);
  const int W = tft->width(), H = tft->height();
  tft->setTextWrap(false);
  tft->setTextSize(2);

  // Aviso pequeno no canto superior direito (sacos nao gravados / sem hora)
  String aviso = pesagensPerdidas > 0 ? String(F("NAO GRAVADOS: ")) + String(pesagensPerdidas)
                                      : (relogioDisponivel() ? String("") : String(F("SEM HORA")));
  if (aviso != barraAviso) {
    tft->fillRect(W - 170, 0, 170, 14, COR_FUNDO);
    if (aviso.length()) {
      tft->setTextSize(1);
      tft->setTextColor(COR_VERMELHO);
      tft->setCursor(W - 6 * (int)aviso.length() - 8, 4);
      tft->print(aviso);
      tft->setTextSize(2);
    }
    barraAviso = aviso;
  }

  int nivel = nivelMemoria();
  if (nivel != barraNivel) {
    uint16_t fundo = nivel == 2 ? COR_VERMELHO : (nivel == 1 ? COR_AMARELO : COR_FUNDO);
    tft->fillRect(0, BARRA_Y, W, H - BARRA_Y, fundo);
    if (nivel == 0) tft->drawFastHLine(0, BARRA_Y, W, COR_CINZA);
    barraNivel = nivel;
    barraConexao = -1; barraHora = ""; barraFila = -1; barraIp = ""; barraAlerta = "";
  }

  // Alerta de memoria de registros: faixa inteira colorida
  if (nivel >= 1) {
    uint32_t cap = capacidadeRegistros();
    uint32_t pct = cap ? (uint32_t)((uint64_t)pendenciasCache * 100ULL / cap) : 100;
    String l1 = nivel == 2 ? String(F("MEMORIA CHEIA!"))
                           : String(F("SEM ENVIO: ")) + String(pendenciasCache) + F(" SACOS (") + String(pct) + F("%)");
    String l2 = nivel == 2 ? String(F("NAO GRAVADOS: ")) + String(pesagensPerdidas) : String(F("VERIFIQUE A INTERNET"));
    if (l1 + "|" + l2 != barraAlerta) {
      uint16_t fundo = nivel == 2 ? COR_VERMELHO : COR_AMARELO;
      tft->fillRect(0, BARRA_Y, W, H - BARRA_Y, fundo);
      tft->setTextColor(nivel == 2 ? COR_TEXTO : COR_FUNDO);
      tft->setCursor(10, 276); tft->print(l1);
      tft->setCursor(10, 298); tft->print(l2);
      barraAlerta = l1 + "|" + l2;
    }
    return;
  }

  // Conexao: bolinha (raio 12) + palavra
  int con = internetConectada() ? (firebaseProntoCache ? 2 : 1) : 0;
  if (con != barraConexao) {
    uint16_t cor = con == 2 ? COR_VERDE : (con == 1 ? COR_AMARELO : COR_VERMELHO);
    tft->fillRect(8, 280, 34, 30, COR_FUNDO);
    tft->fillCircle(24, 294, 12, cor);
    tft->fillRect(46, 276, 190, 16, COR_FUNDO);
    tft->setTextColor(cor);
    tft->setCursor(46, 276);
    if (con == 2) tft->print(F("ONLINE"));
    else if (con == 1) tft->print(F("SEM FIREBASE"));
    else tft->print(F("SEM INTERNET"));
    barraConexao = con;
  }

  // Hora HH:MM (muda 1x por minuto)
  uint32_t t = obterEpochAtual();
  String hora = t ? formatarHora(t).substring(0, 5) : String(F("--:--"));
  if (hora != barraHora) {
    tft->fillRect(W - 80, 276, 80, 16, COR_FUNDO);
    tft->setTextColor(COR_TEXTO);
    tft->setCursor(W - 12 * (int)hora.length() - 8, 276);
    tft->print(hora);
    barraHora = hora;
  }

  // Fila de sacos esperando envio (amarela quando > 0)
  if ((long)pendenciasCache != barraFila) {
    tft->fillRect(46, 298, 220, 16, COR_FUNDO);
    tft->setTextColor(pendenciasCache == 0 ? COR_TEXTO : COR_AMARELO);
    tft->setCursor(46, 298);
    tft->print(F("FILA: ") + String(pendenciasCache));
    barraFila = pendenciasCache;
  }

  // IP (ou -- sem internet)
  String ip = internetConectada() ? WiFi.localIP().toString() : String(F("--"));
  if (ip != barraIp) {
    tft->fillRect(W - 190, 298, 190, 16, COR_FUNDO);
    tft->setTextColor(COR_CINZA);
    tft->setCursor(W - 12 * (int)ip.length() - 8, 298);
    tft->print(ip);
    barraIp = ip;
  }
}

void mostrarMensagem(const String &titulo, const String &linha1, const String &linha2, uint16_t cor) {
  mensagemTemporariaAtiva = false;
  telaPrincipalAtiva = false;
  if (!displayOK) return;
  tft->fillScreen(COR_FUNDO);
  tft->setTextWrap(false);
  tft->setTextSize(4);
  tft->setTextColor(cor);
  int x = ((int)tft->width() - (int)titulo.length() * 24) / 2;
  if (x < 10) x = 10;
  tft->setCursor(x, 60);
  tft->print(titulo);
  tft->setTextSize(2);
  tft->setTextColor(COR_TEXTO);
  tft->setTextWrap(true);
  tft->setCursor(10, 140);
  tft->print(linha1);
  if (linha2.length() > 0) {
    tft->setCursor(10, tft->getCursorY() + 30);
    tft->print(linha2);
  }
  tft->setTextWrap(false);
}

void mostrarMensagemTemporaria(const String &titulo, const String &linha1, const String &linha2, uint16_t cor, unsigned long ms) {
  mostrarMensagem(titulo, linha1, linha2, cor);
  mensagemTemporariaAtiva = true;
  mensagemTemporariaFim = millis() + ms;
}

void verificarMensagemTemporaria() {
  if (!mensagemTemporariaAtiva) return;
  if ((long)(millis() - mensagemTemporariaFim) < 0) return;
  if (simulacaoAtiva) return;
  desenharTelaPrincipal();
}

// ============================================================================
// REDE LOCAL: OTA + TELNET (PuTTY: tipo Telnet, IP do WeMos, porta 23)
// ============================================================================
bool telnetPronto() { return servicosRedeIniciados && telnetClient && telnetClient.connected(); }

String prefixoLog() {
  uint32_t t = (uint32_t)time(nullptr);
  if (epochValido(t)) return "[" + formatarHora(t) + "] ";
  return "[+" + String(millis() / 1000UL) + "s] ";
}

void logln(const String &s) {
  if (!telnetPronto()) return;
  String linha = prefixoLog() + s + "\r\n";
  if (telnetClient.availableForWrite() < (int)linha.length()) { telnetLinhasPerdidas++; return; }
  telnetClient.write((const uint8_t*)linha.c_str(), linha.length());
}

const char* nomeErroOta(ota_error_t e) {
  switch (e) {
    case OTA_AUTH_ERROR:    return "senha errada";
    case OTA_BEGIN_ERROR:   return "sem espaco para o arquivo novo";
    case OTA_CONNECT_ERROR: return "WeMos nao conectou no PC (firewall?)";
    case OTA_RECEIVE_ERROR: return "falha ao receber (Wi-Fi instavel?)";
    case OTA_END_ERROR:     return "falha ao finalizar";
    default:                return "desconhecido";
  }
}

void iniciarServicosRede() {
  if (servicosRedeIniciados) return;
  telnetServer.setNoDelay(true);
  telnetServer.begin();
  servicosRedeIniciados = true;
  if (USAR_OTA) configurarOta();
}

// Prepara e liga o servico de atualizacao pelo Wi-Fi (com mDNS: o Arduino IDE
// passa a mostrar a porta "ensacadeira2 at <ip>")
void configurarOta() {
  if (otaAtiva) return;
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_SENHA);
  ArduinoOTA.onStart([]() {
    otaTelaTrocada = true;
    otaCodigoErro = -1;
    TLOG(F("OTA iniciando"));
    mostrarMensagem(F("ATUALIZANDO"), F("Nao desligue"), F(""), COR_AMARELO);
  });
  ArduinoOTA.onEnd([]() {
    TLOG(F("OTA concluida, reiniciando"));
    strncpy(diagRtc.motivo, "atualizacao OTA", sizeof(diagRtc.motivo) - 1);
    ESP.rtcUserMemoryWrite(DIAG_OFFSET, (uint32_t*)&diagRtc, sizeof(diagRtc));
  });
  ArduinoOTA.onError([](ota_error_t e) {
    otaCodigoErro = (int)e;
    TLOG(F("OTA erro ") + String((int)e) + F(": ") + nomeErroOta(e));
  });
  ArduinoOTA.begin();
  otaAtiva = true;
  otaLiberadaEm = millis();
}

// Comando Telnet "ota": libera a gravacao remota por alguns minutos.
// Fecha a conexao do Firebase para sobrar memoria; os sacos continuam sendo
// gravados na fila e sao enviados depois que a WeMos reiniciar.
void liberarOtaSobDemanda() {
  if (otaAtiva) {
    logln(F("OTA ja esta liberada. Porta no IDE: ") + String(OTA_HOSTNAME) + F(" at ") + WiFi.localIP().toString());
    return;
  }
  if (!internetConectada()) { logln(F("OTA recusada: sem Wi-Fi")); return; }
  if (ciclo.ativo || simulacaoAtiva) { logln(F("OTA recusada: ciclo de saco em andamento, tente de novo")); return; }
  if (String(OTA_SENHA).startsWith("PREENCHER")) {
    logln(F("OTA recusada: preencha OTA_SENHA no codigo (gravar uma vez pelo cabo USB)"));
    return;
  }
  fbdo.stopWiFiClient();               // solta a memoria da conexao segura
  firebaseProntoCache = false;
  configurarOta();
  logln(F("OTA LIBERADA por ") + String(OTA_MINUTOS_LIBERADA) + F(" min. Memoria livre=") + String(ESP.getFreeHeap()));
  logln(F("No Arduino IDE: Ferramentas > Porta > ") + String(OTA_HOSTNAME) + F(" at ") + WiFi.localIP().toString() +
        F("  (pode levar ~30 s para aparecer)"));
  logln(F("Envio ao Firebase pausado ate reiniciar. Sem atualizacao, reinicia sozinha no fim do prazo."));
  mostrarMensagemTemporaria(F("ATUALIZACAO LIBERADA"), F("Gravacao pelo Wi-Fi por ") + String(OTA_MINUTOS_LIBERADA) + F(" min"),
                            WiFi.localIP().toString(), COR_AMARELO, 5000);
}

void logStatus() {
  logln(F("------------- STATUS -------------"));
  logln(String(F("Firmware ")) + FW_VERSAO + F("  boot #") + String(bootId) + F("  ligado ha ") + String(millis() / 1000UL) + F(" s"));
  logln(F("Ultimo reinicio: ") + ultimoResetTexto);
  logln(F("Produto: ") + produtos[indiceAtual] + F("  (lista: ") + origemProdutos + F(")"));
  logln(F("WiFi: ") + (internetConectada() ? WiFi.localIP().toString() + F("  RSSI ") + String(WiFi.RSSI()) + F(" dBm")
                                        : String(F("DESCONECTADO"))));
  logln(F("Firebase: ") + String(firebaseProntoCache ? F("pronto") : F("NAO pronto")) + F("  falhas seguidas=") + String(falhasSeguidas) +
        F("  ultimo envio ok: ") + (ultimoEnvioOk ? String((millis() - ultimoEnvioOk) / 1000UL) + F(" s atras") : String(F("nenhum"))));
  atualizarEspacoFs(true);
  {
    uint32_t cap = capacidadeRegistros();
    logln(F("Memoria de registros: pendentes=") + String(pendenciasCache) + F(" de ") + String(cap) +
          F(" (") + String(cap ? (uint32_t)((uint64_t)pendenciasCache * 100ULL / cap) : 100) + F("%)  cabem mais ~") +
          String(cap > (uint32_t)pendenciasCache ? cap - (uint32_t)pendenciasCache : 0) + F(" sacos  FS livre=") +
          String(fsLivreCache / 1024) + F(" KB de ") + String(fsTotalCache / 1024) + F(" KB  nao gravados=") +
          String(pesagensPerdidas));
  }
  logln(F("Fila: pendentes=") + String(pendenciasCache) + F("  registrados=") + String(totalRegistrados) +
        F(" (estaveis=") + String(sacosEstaveis) + F(" sem_estab=") + String(sacosSemEstavel) + F(")") +
        F("  perdidas=") + String(pesagensPerdidas) + F("  corrompidas=") + String(registrosCorrompidos));
  logln(F("RX: bytes=") + String(bytesRecebidos) + F("  pacotes validos=") + String(pacotesValidos) +
        F("  invalidos=") + String(pacotesInvalidos) + F("  overruns=") + String(overrunsSerial));
  logln(F("Balanca agora: peso=") + pesoTexto(pesoAtual) + (instavelAtual ? F(" INSTAVEL") : F(" ESTAVEL")) +
        (sp3Atual ? F(" SP3=1") : F(" SP3=0")) + F("  ciclo=") + (ciclo.ativo ? String(ciclo.numero) : String(F("nenhum"))) +
        (aguardarSp3Desligar ? F("  (aguardando SP3 desligar)") : F("")));
  uint32_t t = obterEpochAtual();
  logln(F("Hora: ") + (t ? formatarData(t) + F(" ") + formatarHora(t) : String(F("SEM HORA"))) +
        F("  rtcOK=") + String(rtcOK ? F("sim") : F("nao")) + F("  rtcConfiavel=") + String(rtcConfiavel ? F("sim") : F("nao")));
  logln(F("Ultimo saco: ") + (ultimaPesagemPeso.length() ? ultimaPesagemPeso + F(" ") + ultimaPesagemData + F(" ") + ultimaPesagemHora
                                                      : String(F("nenhum"))));
  logln(F("Atualizacao pelo Wi-Fi (OTA): ") + String(otaAtiva ? F("LIBERADA") : F("desligada (comando: ota)")));
  logln(F("Monitores: eventos=") + String(monitorEventos ? F("LIGADO") : F("desligado")) + F("  pacotes=") + String(monitorPacotes ? F("LIGADO") : F("desligado")));
  logln(F("Memoria: heap=") + String(ESP.getFreeHeap()) + F(" (min ") + String(diagRtc.heapMin) + F(")  maiorBloco=") +
        String(ESP.getMaxFreeBlockSize()) + F(" (min ") + String(diagRtc.blocoMin) + F(")") +
        F("  livreOTA=") + String(ESP.getFreeSketchSpace()));
  logln(F("----------------------------------"));
}

void logLeitura() {
  logln(F("------------- LEITURA RX -------------"));
  if (bytesRecebidos == 0) {
    logln(F("NADA RECEBIDO desde o boot. Confira: RO no RX/D0, /RE e DE no GND,"));
    logln(F("VCC/GND do MAX485, A/B/GND do Alfa e RS=485 / PR=T02 / VC=115 no Alfa."));
    return;
  }
  logln(F("Bytes: total=") + String(bytesRecebidos) + F("  ultimo segundo=") + String(bytesUltimoSegundo) +
        F("  ultimo byte ha ") + String(millis() - ultimoByteMs) + F(" ms"));
  logln(F("Pacotes T02: validos=") + String(pacotesValidos) + F("  ETX/BCC invalidos=") + String(pacotesInvalidos) +
        F("  peso nao numerico=") + String(pacotesPesoInvalido));
  if (pacotesValidos > 0) {
    String h;
    for (int i = 0; i < 15; i++) { h += hex2(ultimoPacote[i]); h += ' '; }
    logln(F("Ultimo pacote (ha ") + String(millis() - ultimoPacoteMs) + F(" ms): ") + h);
  }
  logln(F("--------------------------------------"));
}

void logHeartbeat() {
  static unsigned long ultimo = 0;
  if (!telnetPronto() || millis() - ultimo < 30000UL) return;
  ultimo = millis();
  logln(F("vivo | wifi=") + String(internetConectada() ? F("ok") : F("OFF")) + F(" fb=") + String(firebaseProntoCache ? F("ok") : F("--")) +
        F(" pend=") + String(pendenciasCache) + F(" sacos=") + String(totalRegistrados) +
        F(" pacotes=") + String(pacotesValidos) + F(" heap=") + String(ESP.getFreeHeap()));
}

void logAjuda() {
  logln(F("status    situacao geral"));
  logln(F("leitura   bytes/pacotes recebidos e ultimo pacote"));
  logln(F("eventos   liga/desliga: mostra cada mudanca de STS1/STS2 (SP3, estabilidade)"));
  logln(F("pacotes   liga/desliga: 1 linha CSV por pacote (ate desligar)"));
  logln(F("          formato: PKT;millis;peso;tara;STS1;STS2;estavel;sp3;ciclo"));
  logln(F("bytes     bytes crus em HEX por 60 s"));
  logln(F("produtos  mostra a lista de produtos e de onde ela veio"));
  logln(F("zerar     zera o contador de sacos nao gravados (memoria cheia)"));
  logln(F("ota       libera a gravacao pelo Wi-Fi (Arduino IDE) por alguns minutos"));
  logln(F("atualizar le a lista de produtos do Firebase agora"));
  logln(F("teste     simula um saco 99999 em /testes_ensacadeira_2"));
  logln(F("reinicios historico dos ultimos reinicios e o motivo de cada um"));
  logln(F("reiniciar reinicia o WeMos"));
  logln(F("sair      fecha a sessao"));
  logln(F("Linhas CICLO;... = resumo de cada saco registrado."));
}

void telnetComando(String cmd) {
  cmd.trim();
  cmd.toLowerCase();
  if (cmd == "s" || cmd == "status") logStatus();
  else if (cmd == "l" || cmd == "leitura") logLeitura();
  else if (cmd == "eventos" || cmd == "e") {
    monitorEventos = !monitorEventos;
    logln(F("Eventos: ") + String(monitorEventos ? F("LIGADO") : F("DESLIGADO")));
  } else if (cmd == "pacotes" || cmd == "p") {
    monitorPacotes = !monitorPacotes;
    logln(F("Pacotes: ") + String(monitorPacotes ? F("LIGADO (PKT;millis;peso;tara;STS1;STS2;estavel;sp3;ciclo)") : F("DESLIGADO")));
  } else if (cmd == "bytes" || cmd == "b") {
    if (debugBytesFim != 0 && (long)(millis() - debugBytesFim) < 0) { debugBytesFim = 0; logln(F("Bytes: DESLIGADO")); }
    else { debugBytesFim = millis() + 60000UL; linhaCrua = ""; logln(F("Bytes crus: LIGADO por 60 s")); }
  } else if (cmd == "reinicios" || cmd == "r") {
    logReinicios();
  } else if (cmd == "ota") {
    liberarOtaSobDemanda();
  } else if (cmd == "zerar") {
    pesagensPerdidas = 0;
    salvarPerdidas();
    memoriaCheia = false;
    invalidarBarra();
    logln(F("Contador de sacos nao gravados ZERADO"));
  } else if (cmd == "produtos") {
    logProdutos();
  } else if (cmd == "atualizar") {
    pedidoAtualizarProdutos = true;
    ultimaBuscaProdutos = 0;
    logln(F("Leitura da lista de produtos pedida (acontece no proximo momento sem ciclo de SP3)"));
  } else if (cmd == "teste") {
    if (ciclo.ativo || simulacaoAtiva) logln(F("Teste recusado: ciclo em andamento"));
    else iniciarSimulacao();
  } else if (cmd == "reiniciar") {
    reiniciarComMotivo("comando reiniciar (Telnet)");
  } else if (cmd == "ajuda" || cmd == "h" || cmd == "?") logAjuda();
  else if (cmd == "sair") { logln(F("Sessao encerrada")); telnetClient.stop(); }
  else if (cmd.length() > 0) logln(F("Comando desconhecido: ") + cmd + F("   (digite ajuda)"));
}

bool telnetFeed(int c) {
  if (telnetIac == 1) {
    if (c >= 251 && c <= 254) telnetIac = 2;
    else if (c == 250) telnetIac = 3;
    else telnetIac = 0;
    return false;
  }
  if (telnetIac == 2) { telnetIac = 0; return false; }
  if (telnetIac == 3) { if (c == 255) telnetIac = 1; return false; }
  if (c == 255) { telnetIac = 1; return false; }
  if (c == '\r' || c == '\n') {
    if (telnetLen == 0) return false;
    telnetBuf[telnetLen] = 0;
    telnetLen = 0;
    return true;
  }
  if ((c == 8 || c == 127) && telnetLen > 0) { telnetLen--; return false; }
  if (c >= 32 && c < 127 && telnetLen < sizeof(telnetBuf) - 1) telnetBuf[telnetLen++] = (char)c;
  return false;
}

void telnetAceitar() {
  if (!telnetServer.hasClient()) return;
  WiFiClient novo = telnetServer.accept();
  if (!novo) return;
  if (telnetClient && telnetClient.connected()) {
    telnetClient.print(F("\r\n*** outra sessao Telnet assumiu ***\r\n"));
    telnetClient.stop();
  }
  telnetClient = novo;
  telnetClient.setNoDelay(true);
  telnetClient.keepAlive(15, 5, 3);
  telnetLen = 0;
  telnetIac = 0;
  monitorEventos = true;
  monitorPacotes = false;
  logln(F("=== ENSACADEIRA 2 === digite ajuda. Monitor de eventos LIGADO."));
  logStatus();
}

void telnetProcessar() {
  if (!servicosRedeIniciados) return;
  telnetAceitar();
  if (!telnetPronto()) return;
  int guarda = 0;
  while (telnetClient.available() > 0 && guarda++ < 64) {
    int c = telnetClient.read();
    if (c < 0) break;
    if (telnetFeed(c)) telnetComando(String(telnetBuf));
    if (!telnetPronto()) return;
  }
  logHeartbeat();
}

// ============================================================================
// DIAGNOSTICO DE REINICIO - funcoes
// ============================================================================
void diagIniciar() {
  rst_info *ri = ESP.getResetInfoPtr();
  uint32_t motivo = ri ? ri->reason : 255;
  String nome;
  switch (motivo) {
    case REASON_DEFAULT_RST:      nome = "ENERGIA (ligou ou queda de tensao)"; break;
    case REASON_WDT_RST:          nome = "WATCHDOG DE HARDWARE (travou)"; break;
    case REASON_EXCEPTION_RST:    nome = "CRASH (excecao " + String(ri->exccause) + ")"; break;
    case REASON_SOFT_WDT_RST:     nome = "WATCHDOG DE SOFTWARE (loop travado)"; break;
    case REASON_SOFT_RESTART:     nome = "REINICIO PELO PROPRIO CODIGO"; break;
    case REASON_DEEP_SLEEP_AWAKE: nome = "deep sleep"; break;
    case REASON_EXT_SYS_RST:      nome = "RESET EXTERNO (botao RESET ou queda de tensao)"; break;
    default:                      nome = "desconhecido (" + String(motivo) + ")"; break;
  }
  ultimoResetTexto = nome;
  if (ri && (motivo == REASON_EXCEPTION_RST || motivo == REASON_WDT_RST || motivo == REASON_SOFT_WDT_RST)) {
    char b[64];
    snprintf(b, sizeof(b), " epc1=0x%08x excvaddr=0x%08x", ri->epc1, ri->excvaddr);
    ultimoResetTexto += b;
  }

  DiagRtc anterior;
  ESP.rtcUserMemoryRead(DIAG_OFFSET, (uint32_t*)&anterior, sizeof(anterior));
  if (anterior.magic == DIAG_MAGIC) {
    anterior.motivo[sizeof(anterior.motivo) - 1] = 0;
    ultimoResetTexto += " | antes: ligado " + String(anterior.uptimeS) + " s, parte=" + String(nomeEtapa(anterior.etapa)) +
                        ", heap min=" + String(anterior.heapMin) + ", bloco min=" + String(anterior.blocoMin);
    if (anterior.motivo[0]) ultimoResetTexto += ", motivo=" + String(anterior.motivo);
    if (anterior.heapAntesLogin)
      ultimoResetTexto += ", memoria antes do login=" + String(anterior.heapAntesLogin) + "/" + String(anterior.blocoAntesLogin);
  } else {
    ultimoResetTexto += " | sem dados anteriores (memoria RTC apagada = faltou energia)";
  }

  memset(&diagRtc, 0, sizeof(diagRtc));
  diagRtc.magic = DIAG_MAGIC;
  diagRtc.heapMin = ESP.getFreeHeap();
  diagRtc.blocoMin = ESP.getMaxFreeBlockSize();
  diagRtc.etapa = ET_SETUP;
  ESP.rtcUserMemoryWrite(DIAG_OFFSET, (uint32_t*)&diagRtc, sizeof(diagRtc));
}

// 1x por segundo: tempo ligado e memoria minima
void diagAtualizar() {
  static unsigned long ultimo = 0;
  uint32_t h = ESP.getFreeHeap(), b = ESP.getMaxFreeBlockSize();
  if (h < diagRtc.heapMin) diagRtc.heapMin = h;
  if (b < diagRtc.blocoMin) diagRtc.blocoMin = b;
  if (millis() - ultimo < 1000UL) return;
  ultimo = millis();
  diagRtc.uptimeS = millis() / 1000UL;
  diagRtc.boot = bootId;
  ESP.rtcUserMemoryWrite(DIAG_OFFSET, (uint32_t*)&diagRtc, sizeof(diagRtc));
}

// Depois do LittleFS montar: guarda uma linha no historico
void diagRegistrarBoot() {
  if (!littleFsOK) return;
  File t = LittleFS.open(ARQUIVO_REINICIOS, "r");
  if (t) {
    size_t tam = t.size();
    t.close();
    if (tam > 6000) LittleFS.remove(ARQUIVO_REINICIOS);   // mantem o arquivo pequeno
  }
  File f = LittleFS.open(ARQUIVO_REINICIOS, "a");
  if (!f) return;
  uint32_t ep = obterEpochAtual();
  f.print("boot #" + String(bootId) + " " + (ep ? formatarData(ep) + " " + formatarHora(ep) : String("sem hora")) +
          " : " + ultimoResetTexto + "\n");
  f.close();
}

void reiniciarComMotivo(const String &motivo) {
  strncpy(diagRtc.motivo, motivo.c_str(), sizeof(diagRtc.motivo) - 1);
  diagRtc.motivo[sizeof(diagRtc.motivo) - 1] = 0;
  diagRtc.uptimeS = millis() / 1000UL;
  ESP.rtcUserMemoryWrite(DIAG_OFFSET, (uint32_t*)&diagRtc, sizeof(diagRtc));
  TLOG(F("REINICIANDO: ") + motivo);
  delay(200);
  ESP.restart();
}

void logReinicios() {
  logln(F("Ultimo reinicio: ") + ultimoResetTexto);
  if (!littleFsOK || !LittleFS.exists(ARQUIVO_REINICIOS)) { logln(F("(sem historico)")); return; }
  File f = LittleFS.open(ARQUIVO_REINICIOS, "r");
  if (!f) return;
  // mostra as ultimas 15 linhas
  const int MAXL = 15;
  String linhas[MAXL];
  int n = 0;
  while (f.available()) {
    String l = f.readStringUntil('\n');
    if (l.length() == 0) continue;
    linhas[n % MAXL] = l;
    n++;
  }
  f.close();
  int ini = n > MAXL ? n - MAXL : 0;
  logln(F("--- historico (ultimos ") + String(n - ini) + F(" de ") + String(n) + F(") ---"));
  for (int i = ini; i < n; i++) {
    unsigned long t0 = millis();
    while (telnetPronto() && telnetClient.availableForWrite() < 300 && millis() - t0 < 300) { delay(5); }
    logln(linhas[i % MAXL]);
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  diagIniciar();
  // Buffer: com ~60 bytes/s da balanca, 1 KB segura ~15 s de pacotes
  // enquanto o Wi-Fi/Firebase estiver ocupado (2 KB fazia falta no login).
  Serial.setRxBufferSize(1024);
  Serial.begin(115200, SERIAL_8N1);   // RX = GPIO3 (sem Serial.swap)

  displayOK = tft->begin(TFT_SPI_HZ);
  if (displayOK) { tft->setRotation(1); tft->fillScreen(COR_FUNDO); }

  // Botoes DEPOIS do tft->begin(): o SPI reserva GPIO12 (MISO)
  pinMode(PINO_AVANCAR, INPUT_PULLUP);
  pinMode(PINO_RETORNAR, INPUT);      // GPIO15 tem pull-down na placa

  Wire.begin(PINO_RTC_SDA, PINO_RTC_SCL);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(2000);
  rtcOK = rtc.begin(&Wire);
  rtcConfiavel = rtcOK && !rtc.lostPower();

  inicializarNTP();

  LittleFSConfig configFs13;
  configFs13.setAutoFormat(false);
  LittleFS.setConfig(configFs13);
  littleFsOK = LittleFS.begin();     // NUNCA formata sozinho (protege a fila)
  if (littleFsOK) {
    carregarBootId();
    prepararFila();
    carregarCiclo13();
  }
  diagRtc.boot = bootId;
  diagRegistrarBoot();
  carregarListaProdutos();          // LittleFS (ultima lista do Firebase) ou lista do codigo
  carregarProdutoAtivo();
  if (littleFsOK) {
    carregarUltimaPesagem();
    carregarPerdidas();
    atualizarEspacoFs(true);
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.hostname(OTA_HOSTNAME);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_SENHA);
  ultimoTempoConectado = millis();
  ultimaTentativaWiFi = millis();

  String relogio;
  if (!rtcOK) relogio = "RTC NAO ENCONTRADO";
  else if (!rtcConfiavel) relogio = "RTC sem hora";
  else { uint32_t t = lerRTC(); relogio = "RTC " + formatarData(t) + " " + formatarHora(t); }

  if (littleFsOK) {
    mostrarMensagem(F("INICIANDO"), String(FW_VERSAO) + F("  Pendentes: ") + String(pendenciasCache) + F("  ") + relogio,
                    F("Reinicio: ") + ultimoResetTexto, rtcConfiavel ? COR_VERDE : COR_AMARELO);
  } else {
    mostrarMensagem(F("LittleFS FALHOU"), F("Pesagens NAO serao gravadas"), F("Confira Flash Size (FS:2MB)"), COR_VERMELHO);
  }
  delay(2500);

  // Descarta o que chegou durante a inicializacao e comeca a leitura limpa
  while (Serial.available() > 0) Serial.read();
  Serial.hasOverrun();
  resetarLeitura();
  desenharTelaPrincipal();
}

// ============================================================================
// LOOP
// ============================================================================
void loop() {
  marcarEtapa(ET_OTA);
  if (otaAtiva) {
    // OTA liberada pelo comando: prazo esgotado sem atualizacao -> reinicia
    // (volta ao normal, com o Firebase e sem o mDNS ocupando memoria)
    if (!USAR_OTA && millis() - otaLiberadaEm > OTA_MINUTOS_LIBERADA * 60000UL && !ciclo.ativo) {
      reiniciarComMotivo("OTA liberada e nao usada");
    }
    ArduinoOTA.handle();
    if (otaTelaTrocada) {
      otaTelaTrocada = false;
      mostrarMensagemTemporaria(F("OTA FALHOU"), F("Codigo ") + String(otaCodigoErro), F("O codigo anterior continua"), COR_VERMELHO, 6000);
    }
  }
  marcarEtapa(ET_TELNET);
  telnetProcessar();
  marcarEtapa(ET_WIFI);
  gerenciarConexao();
  marcarEtapa(ET_BOTOES);
  lerBotoes();

  marcarEtapa(ET_BALANCA);
  lerBalanca();
  marcarEtapa(ET_SIMULACAO);
  processarSimulacao();
  if (pacoteMotor13 && millis() - pacoteMotor13 > 2000UL) lacunaMotor13();
  if ((motor13.pending() || reparoCheckpoint13) && millis() - tentativaPersistir13 > 1000UL) {
    tentativaPersistir13 = millis(); manterPersistencia13();
  }

  marcarEtapa(ET_RTC);
  manterRTC();
  processarRede();       // marca as etapas do Firebase por dentro
  marcarEtapa(ET_BALANCA);
  lerBalanca();          // depois de possivel envio, esvazia o buffer logo

  marcarEtapa(ET_DISPLAY);
  verificarTimeoutTeste();
  verificarMensagemTemporaria();

  if (millis() - ultimaChecagemStatus >= 1000UL) {
    atualizarIndicadorStatus(false);
    ultimaChecagemStatus = millis();
  }

  static unsigned long ultimaTaxaRx = 0;
  static uint32_t bytesNaUltimaTaxa = 0;
  if (millis() - ultimaTaxaRx >= 1000UL) {
    bytesUltimoSegundo = bytesRecebidos - bytesNaUltimaTaxa;
    bytesNaUltimaTaxa = bytesRecebidos;
    ultimaTaxaRx = millis();
  }

  marcarEtapa(ET_SAUDE);
  verificarSaude();
  diagAtualizar();
  marcarEtapa(ET_OCIOSO);
  yield();
}
