#include "debounce.h"

void debounce_init(Debounce *d, uint8_t level, uint8_t required) {
  d->state = DEB_ESTAVEL;
  d->level = level ? 1 : 0;
  d->count = 0;
  d->required = required ? required : 1;
}

// Máquina de estados desenhada em debounce.h.
uint8_t debounce_update(Debounce *d, uint8_t raw) {
  raw = raw ? 1 : 0;

  switch (d->state) {
    case DEB_ESTAVEL:
      if (raw == d->level) break;  // nada mudou
      // Amostra diferente: começa a confirmar, e esta já é a 1ª da contagem.
      d->state = DEB_CONFIRMANDO;
      d->count = 0;
      // Segue direto para CONFIRMANDO para contar esta amostra
      // (com required = 1, já aceita aqui).
      __attribute__((fallthrough));

    case DEB_CONFIRMANDO:
      if (raw == d->level) {
        // Voltou ao nível antigo antes de confirmar: era repique.
        d->state = DEB_ESTAVEL;
        d->count = 0;
      } else if (++d->count >= d->required) {
        // Ficou 'required' amostras seguidas no nível novo: aceita.
        d->level = raw;
        d->state = DEB_ESTAVEL;
        d->count = 0;
      }
      break;
  }

  return d->level;
}

uint8_t debounce_level(const Debounce *d) {
  return d->level;
}
