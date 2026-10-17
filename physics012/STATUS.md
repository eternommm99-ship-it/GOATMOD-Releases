# GOATMOD PHYSICS 0.12 — OFFLINE GEOMETRY

Estado de esta rama:

- Base de trabajo: PHYSICS 0.11 queda intacta.
- Implementado aquí: geometría limpia de banda/tronera para integrar sobre 0.11.
- Incluye: límites jugables compensados por radio de bola, primer impacto, punto físico de contacto, reflexión ideal, boca útil de tronera basada en medida de boca, corredor de obstáculos y ruta de una banda por espejo geométrico.
- La entrada por boca se considera posibilidad geométrica, no embocada garantizada.
- No implementa todavía potencia, fricción, spin/throw, Autoplay, AutoQueue ni panel.
- Las pruebas host cubren reflexión horizontal/vertical/oblicua, radio, boca lateral, obstrucción, degenerados, one-bank, rechazo de rebote dentro de boca y simetría izquierda/derecha.

Criterio para cerrar la etapa offline: compilar con C++17 y pasar `goatmod_physics012_geometry` sin regresiones antes de integrar en Android.
