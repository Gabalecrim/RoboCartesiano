#include "encoder.h"

// Conferências em tempo de compilação: um parâmetro errado em
// build_flags para o build aqui, em vez de virar bug na bancada.
_Static_assert(ENCODER_SAMPLE_PERIOD_US >= 100,
               "amostragem acima de 10 kHz pesa demais no esp_timer");
_Static_assert(ENCODER_DEBOUNCE_US % ENCODER_SAMPLE_PERIOD_US == 0,
               "ENCODER_DEBOUNCE_US precisa ser múltiplo de ENCODER_SAMPLE_PERIOD_US");
_Static_assert(ENCODER_DEBOUNCE_SAMPLES >= 1 && ENCODER_DEBOUNCE_SAMPLES <= 255,
               "debounce precisa dar de 1 a 255 amostras");

// Chamada pelo esp_timer a cada ENCODER_SAMPLE_PERIOD_US.
// Roda na tarefa do esp_timer (não é ISR), mas vale a mesma regra:
// curta, sem Serial, sem delay, sem alocar memória.
static void onSample(void *arg) {
  Encoder *self = (Encoder *)arg;
  int64_t t0 = esp_timer_get_time();
  uint8_t rawA = (uint8_t)digitalRead(self->pinA);
  uint8_t rawB = (uint8_t)digitalRead(self->pinB);

  portENTER_CRITICAL(&self->mux);
  uint8_t a = debounce_update(&self->debA, rawA);
  uint8_t b = debounce_update(&self->debB, rawB);
  quadrature_update(&self->q, (uint8_t)((a << 1) | b));  // mesmo estado = 0

  // Estatística (custa ~1 us): duração desta amostra e espaço desde a anterior.
  EncoderStats *st = &self->stats;
  uint32_t dur = (uint32_t)(esp_timer_get_time() - t0);
  if (self->ultima_amostra_us > 0) {
    uint32_t intervalo = (uint32_t)(t0 - self->ultima_amostra_us);
    if (intervalo > st->max_intervalo_us) st->max_intervalo_us = intervalo;
  }
  self->ultima_amostra_us = t0;
  st->amostras++;
  st->soma_us += dur;
  if (dur > st->max_us) st->max_us = dur;
  st->core = (int8_t)xPortGetCoreID();
  portEXIT_CRITICAL(&self->mux);
}

void encoder_begin(Encoder *enc, uint8_t pinA, uint8_t pinB, bool pullup) {
  enc->pinA = pinA;
  enc->pinB = pinB;
  portMUX_INITIALIZE(&enc->mux);
  pinMode(enc->pinA, pullup ? INPUT_PULLUP : INPUT);
  pinMode(enc->pinB, pullup ? INPUT_PULLUP : INPUT);

  // Parte do estado real dos pinos, já considerado estável.
  uint8_t a = (uint8_t)digitalRead(enc->pinA);
  uint8_t b = (uint8_t)digitalRead(enc->pinB);
  debounce_init(&enc->debA, a, ENCODER_DEBOUNCE_SAMPLES);
  debounce_init(&enc->debB, b, ENCODER_DEBOUNCE_SAMPLES);
  enc->q.errors = 0;
  quadrature_reset(&enc->q, (uint8_t)((a << 1) | b), 0);
  encoder_stats_reset(enc);

  // Um timer por encoder; o ponteiro vai como argumento, assim
  // dá para ter 3 encoders com o mesmo código.
  const esp_timer_create_args_t args = {
      .callback = onSample,
      .arg = enc,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "encoder",
      .skip_unhandled_events = true,  // atrasou? não faz amostras em rajada
  };
  ESP_ERROR_CHECK(esp_timer_create(&args, &enc->timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(enc->timer, ENCODER_SAMPLE_PERIOD_US));
}

int32_t encoder_count(Encoder *enc) {
  portENTER_CRITICAL(&enc->mux);
  int32_t c = enc->q.count;
  portEXIT_CRITICAL(&enc->mux);
  return c;
}

uint32_t encoder_errors(Encoder *enc) {
  portENTER_CRITICAL(&enc->mux);
  uint32_t e = enc->q.errors;
  portEXIT_CRITICAL(&enc->mux);
  return e;
}

void encoder_stop(Encoder *enc) {
  esp_timer_stop(enc->timer);  // erro só se já estava parado: ignorado
}

void encoder_start(Encoder *enc) {
  portENTER_CRITICAL(&enc->mux);
  enc->ultima_amostra_us = 0;  // não conta a pausa como intervalo
  portEXIT_CRITICAL(&enc->mux);
  esp_timer_start_periodic(enc->timer, ENCODER_SAMPLE_PERIOD_US);
}

void encoder_stats(Encoder *enc, EncoderStats *out) {
  portENTER_CRITICAL(&enc->mux);
  *out = enc->stats;
  portEXIT_CRITICAL(&enc->mux);
}

void encoder_stats_reset(Encoder *enc) {
  portENTER_CRITICAL(&enc->mux);
  enc->stats.amostras = 0;
  enc->stats.soma_us = 0;
  enc->stats.max_us = 0;
  enc->stats.max_intervalo_us = 0;
  enc->stats.core = -1;
  enc->ultima_amostra_us = 0;
  portEXIT_CRITICAL(&enc->mux);
}

void encoder_reset(Encoder *enc, int32_t value) {
  // Usa o nível LIMPO (não o pino cru): é ele que a quadratura vê.
  portENTER_CRITICAL(&enc->mux);
  uint8_t ab = (uint8_t)((debounce_level(&enc->debA) << 1) | debounce_level(&enc->debB));
  quadrature_reset(&enc->q, ab, value);
  portEXIT_CRITICAL(&enc->mux);
}
