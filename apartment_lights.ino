#include <RotaryEncoder.h>
#include <FastLED.h>
#include <MSGEQ7.h>

#define BTN_DEBOUNCE_MILLIS 200
#define COLOR_CHANGE_MILLIS 250
#define BRIGHTNESS_TICK_SCALE 0.02
#define NUM_STATES 5
#define NUM_AMBIENT_COLORS 11
#define NUM_LEDS 30
#define LED_DATA_PIN 6

#define MSG_DATA_PIN 9
#define MSG_RESET_PIN 10
#define MSG_STROBE_PIN 8
#define MSG_INTERVAL ReadsPerSecond(50)
#define MSG_SMOOTH 191


enum State {
    OFF,
    AMBIENT,
    GRADIENT,
    MUSIC,
    BASS
};

struct Knob {
    RotaryEncoder* encoder;
    long prev_pos;
    uint8_t pin_a;
    uint8_t pin_b;
    void (*update_isr)(void);
};

struct Button {
    unsigned long prev_time;
    bool prev_state;
    uint8_t pin;
};


CRGB ambient_colors[NUM_AMBIENT_COLORS] = {
    {192, 192, 192},  // White
    {255, 20, 0}, // Red
    {255, 140, 0}, // Orange
    {255, 255, 12}, // Yellow
    {124, 252, 0}, // Light green
    {10, 255, 10}, // Green
    {0, 206, 125}, // Blueish green
    {10, 20, 225}, // Blue
    {138, 43, 226}, // Purplish blue
    {199, 21, 133}, // Purple
    {255, 20, 100} // Pink
};

float msg_scale_factors[7] = {
  8.5,
  8.5,
  8.5,
  8.5,
  8.5,
  8.5,
  8.5
};

/* External objects */
CMSGEQ7<MSG_SMOOTH, MSG_RESET_PIN, MSG_STROBE_PIN, MSG_DATA_PIN> msg;
Knob state_knob;
Knob brightness_knob;

Button state_btn;
Button brightness_btn;

/* Device state*/
State state;
float brightness_factor;
uint8_t ambient_color_idx = 0;
uint8_t spectrum_color_offset = 0;
uint8_t msg_vals[7];
bool state_changed;
CRGB true_color[NUM_LEDS];
CRGB leds[NUM_LEDS];


/* ISR function declarations */
void state_btn_pressed();
void update_state_knob();
void update_brightness_knob();


/* Helper function declarations */
void init_knob(Knob*);
void handle_state_knob();
void handle_brightness_knob();


void init_btn(Button*);
bool was_btn_pressed(Button*);

void filter_and_scale_msg();
void set_color(CRGB color);
void write_to_leds();
unsigned int draw_band(unsigned int center_offset, unsigned int size);
void handle_music();
void handle_color_change();
inline CRGB get_spectrum_color(uint8_t spectrum_idx);


void setup() {

    Serial.begin(9600);

    state = OFF;
    brightness_factor = 1.0;
    state_changed = false;

    // state change button init
    state_btn.pin = 4;
    init_btn(&state_btn);

    // TBD button
    brightness_btn.pin = 5;
    init_btn(&brightness_btn);
    

    // state knob init
    state_knob.pin_a = 1;
    state_knob.pin_b = 2;
    state_knob.update_isr = update_state_knob;
    init_knob(&state_knob);

    // brightness knob init
    brightness_knob.pin_a = 3;
    brightness_knob.pin_b = 7;
    brightness_knob.update_isr = update_brightness_knob;
    init_knob(&brightness_knob);

    FastLED.addLeds<WS2811, LED_DATA_PIN, BRG>(leds, NUM_LEDS);

    msg.begin();
}





void loop() {

    // control logic
    state_changed = was_btn_pressed(&state_btn);
    if (state_changed) {
        state = (state + 1) % NUM_STATES;
        Serial.print("state changed: ");
        Serial.println(state);
    }

    handle_state_knob();
    handle_brightness_knob();


    // state machine loop

    switch (state) {
      case OFF:
        if (state_changed) {
          set_color(CRGB::Black);
        }
        break;

      case AMBIENT:
        if (state_changed) {
          set_color(ambient_colors[ambient_color_idx]);
        }
        break;

      case GRADIENT:
      if (state_changed) {
          set_color(CRGB::Black); // TODO
        }
        break;

      case MUSIC:
        handle_music();
        break;

      case BASS:
        break;
    }

}


/* LED helpers */

void handle_music() {

  static uint8_t prev_bass = false;

  if (!msg.read(MSG_INTERVAL)) {
    return;
  }

  filter_and_scale_msg();
  handle_color_change();

  for (int i = 0; i < NUM_LEDS; ++i) {
    true_color[i] = CRGB::Black;
  }

  // Draw higher frequencies
  unsigned int offset = 0;
  for (int i = 6; i >= 2; --i) {
    offset = draw_band(offset, msg_vals[i], get_spectrum_color(i));
  }

  // Draw bass frequency
  for (int i = 0; i < msg_vals[0] && i < NUM_LEDS; ++i) {
    true_color[i] = get_spectrum_color(0);
  }

  for (int i = NUM_LEDS - 1; i >= NUM_LEDS - 1 - msg_vals[0] && i >= 0; --i) {
    true_color[i] = get_spectrum_color(0);
  }

  // Draw 2nd bass frequency
  for (int i = msg_vals[0]; i < msg_vals[0] + msg_vals[1] && i < NUM_LEDS; ++i) {
    true_color[i] = get_spectrum_color(1);
  }

  for (int i = NUM_LEDS - 1 - msg_vals[0]; i >= NUM_LEDS - 1 - msg_vals[0] - msg_vals[1] && i >= 0; --i) {
    true_color[i] = get_spectrum_color(1);
  }

   write_to_leds();
}

void handle_color_change() {
    static unsigned long fall_time = 0;
    static uint8_t prev_vol = 0;

    uint8_t vol = msg.getVolume();
    unsigned long time = millis();

    if (prev_vol && !vol) {
        if (time - fall_time > COLOR_CHANGE_MILLIS) {
            spectrum_color_offset = (spectrum_color_offset + 1) % NUM_AMBIENT_COLORS;
        }
    } else {
        fall_time = time;
    }   

    prev_vol = vol;
}

void filter_and_scale_msg() {
  for (int i = 0; i < 7; ++i) {
    msg_vals[i] = mapNoise(msg.get(i)) / msg_scale_factors[i];
  }
}

void set_color(CRGB color) {
  for (int i = 0; i < NUM_LEDS; ++i) {
    true_color[i] = color;
  }

  write_to_leds();
}

inline CRGB get_spectrum_color(uint8_t spectrum_idx) {
  Serial.println(spectrum_idx + spectrum_color_offset);
  return ambient_colors[(spectrum_idx + spectrum_color_offset) % NUM_AMBIENT_COLORS];
}


unsigned int draw_band(unsigned int center_offset, unsigned int size, CRGB color) {

    // start is out of bounds
    if ((NUM_LEDS / 2) + center_offset >= NUM_LEDS) {
        return center_offset;
    }

    // clip size if out-of-bounds
    if ((NUM_LEDS / 2) + center_offset + size > NUM_LEDS) {
        size = (NUM_LEDS / 2) - center_offset;
    }   

    // positive band
    for (unsigned int i = (NUM_LEDS / 2) + center_offset; i < (NUM_LEDS / 2) + center_offset + size; ++i) {
        true_color[i] = color;
    }

    // negative band
    for (unsigned int i = (NUM_LEDS / 2) + center_offset; i > (NUM_LEDS / 2) + center_offset - size; --i) {
        true_color[i] = color;
    }

    return size + center_offset;
}

void write_to_leds() {

  for (int i = 0; i < NUM_LEDS; ++i) {
    leds[i].setRGB(true_color[i].red * brightness_factor, 
                   true_color[i].green * brightness_factor,
                   true_color[i].blue * brightness_factor);
  }

  FastLED.show();
}


/* Knob functions */

void init_knob(Knob* knob) {
    knob->encoder = new RotaryEncoder(knob->pin_a, knob->pin_b, 
                            RotaryEncoder::LatchMode::TWO03);
    knob->prev_pos = 0;
    knob->encoder->setPosition(knob->prev_pos);
    attachInterrupt(digitalPinToInterrupt(knob->pin_a), knob->update_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(knob->pin_b), knob->update_isr, CHANGE);
}


void handle_state_knob() {

    long cur_pos = state_knob.encoder->getPosition();

    if (state_knob.prev_pos == cur_pos) {
        return;
    }

    long delta = cur_pos - state_knob.prev_pos;
    state_knob.prev_pos = cur_pos;

    // Encoder counts twice per tick
    if (cur_pos % 2 != 0) {
      return;
    }


    switch (state) {
        case AMBIENT:
            // change color
            if (delta > 0) {
              ambient_color_idx = (ambient_color_idx + 1) % NUM_AMBIENT_COLORS;
            } else {
              ambient_color_idx = (ambient_color_idx + NUM_AMBIENT_COLORS - 1) % NUM_AMBIENT_COLORS;
            }
            
            set_color(ambient_colors[ambient_color_idx]);
            Serial.println(ambient_color_idx);
            break;
        
        case MUSIC:
            // ?
            break;

        case BASS:
            // Change bass threshold
            break;
    }

}

void handle_brightness_knob() {

    long cur_pos = brightness_knob.encoder->getPosition();

    if (brightness_knob.prev_pos == cur_pos) {
        return;
    }
    
    float delta = BRIGHTNESS_TICK_SCALE * (cur_pos - brightness_knob.prev_pos);
    brightness_knob.prev_pos = cur_pos;

    if (cur_pos % 2 != 0) {
      return;
    }

  
    brightness_factor += delta;

    if (brightness_factor > 1.0) {
        brightness_factor = 1.0;
    } else if (brightness_factor < 0.0) {
        brightness_factor = 0.0;
    }


    write_to_leds();

    Serial.print("Brightness: ");
    Serial.print(brightness_factor);
    Serial.print(" Delta: ");
    Serial.println(delta);
}


/* Button functions */

void init_btn(Button* btn) {
    pinMode(btn->pin, INPUT_PULLUP);
    btn->prev_time = millis();
    btn->prev_state = digitalRead(btn->pin);
}

bool was_btn_pressed(Button* btn) {
    unsigned long cur_time = millis();
    uint8_t cur_state = digitalRead(btn->pin);
    bool was_pressed = false;
    
    if (cur_state != btn->prev_state) {
        
        was_pressed = (cur_time - btn->prev_time > BTN_DEBOUNCE_MILLIS)
                        && !cur_state;

        btn->prev_time = cur_time;
        btn->prev_state = cur_state;
    }

    return was_pressed;
}




/* ISRs defintions */

void update_state_knob() {
    state_knob.encoder->tick();
}

void update_brightness_knob() {
    brightness_knob.encoder->tick();
}






