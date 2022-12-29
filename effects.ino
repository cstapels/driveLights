//for fire
#define BRIGHTNESS 200

bool gReverseDirection = false;
#define COOLING 55
#define SPARKING 120
extern uint8_t gHue;

/*
1 fire
2 rainbow
3 rainbowWithGlitter
4 confetti,
5 sinelon,
6  juggle, 
7  bpm
8 RGBLoop
*/

void Fire2012() {
  // Array of temperature readings at each simulation cell
  static uint8_t heat[NUM_LEDS];

  // Step 1.  Cool down every cell a little
  for (int i = 0; i < NUM_LEDS; i++) {
    heat[i] = qsub8(heat[i], random8(0, ((COOLING * 10) / NUM_LEDS) + 2));
  }

  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for (int k = NUM_LEDS - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }

  // Step 3.  Randomly ignite new 'sparks' of heat near the bottom
  if (random8() < SPARKING) {
    int y = random8(7);
    heat[y] = qadd8(heat[y], random8(160, 255));
  }

  // Step 4.  Map from heat cells to LED colors
  for (int j = 0; j < NUM_LEDS; j++) {
    CRGB color = HeatColor(heat[j]);
    int pixelnumber;
    if (gReverseDirection) {
      pixelnumber = (NUM_LEDS - 1) - j;
    } else {
      pixelnumber = j;
    }
    leds[pixelnumber] = color;
  }
}

void rainbow() {
  // FastLED's built-in rainbow generator
  fill_rainbow(leds, NUM_LEDS, gHue, 7);
}

void rainbowWithGlitter() {
  // built-in FastLED rainbow, plus some random sparkly glitter
  rainbow();
  addGlitter(80);
}

void addGlitter(fract8 chanceOfGlitter) {
  if (random8() < chanceOfGlitter) {
    leds[random16(NUM_LEDS)] += CRGB::White;
  }
}

void confetti() {
  // random colored speckles that blink in and fade smoothly
  fadeToBlackBy(leds, NUM_LEDS, 10);
  int pos = random16(NUM_LEDS);
  leds[pos] += CHSV(gHue + random8(64), 200, 255);
}

void sinelon() {
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy(leds, NUM_LEDS, 20);
  int pos = beatsin16(13, 0, NUM_LEDS - 1);
  leds[pos] += CHSV(gHue, 255, 192);
}

void bpm() {
  // colored stripes pulsing at a defined Beats-Per-Minute (BPM)
  uint8_t BeatsPerMinute = 62;
  CRGBPalette16 palette = PartyColors_p;
  uint8_t beat = beatsin8(BeatsPerMinute, 64, 255);
  for (int i = 0; i < NUM_LEDS; i++) {  //9948
    leds[i] = ColorFromPalette(palette, gHue + (i * 2), beat - gHue + (i * 10));
  }
}

void juggle() {
  // eight colored dots, weaving in and out of sync with each other
  fadeToBlackBy(leds, NUM_LEDS, 20);
  uint8_t dothue = 0;
  for (int i = 0; i < 8; i++) {
    leds[beatsin16(i + 7, 0, NUM_LEDS - 1)] |= CHSV(dothue, 200, 255);
    dothue += 32;
  }
}

void RGBLoop(int step) {
  //1536 steps
  if (step < 256) {
    fill_solid(leds, NUM_LEDS, CRGB(step, 0, 0));
  }
  if (step > 256 && step < 512) {
    fill_solid(leds, NUM_LEDS, CRGB(256 - step, 0, 0));
  }
  if (step > 512 && step < 768) {
    fill_solid(leds, NUM_LEDS, CRGB(0, step - 512, 0));
  }
  if (step > 768 && step < 1024) {
    fill_solid(leds, NUM_LEDS, CRGB(0, 1024 - step, 0));
  }
  if (step > 1024 && step < 1280) {
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, step - 1280));
  }
  if (step > 1280 && step < 1536) {
    fill_solid(leds, NUM_LEDS, CRGB(0, 0, 1536 - step));
  }
}


/*
 * Send up a flare
 * 
 */

// void flare() {

//   flarePos = 0;
//   float flareVel = float(random16(50, 90)) / 100; // trial and error to get reasonable range
//   float brightness = 1;

//   // initialize launch sparks
//   for (int i = 0; i < 5; i++) {
//     sparkPos[i] = 0;
//     sparkVel[i] = (float(random8()) / 255) * (flareVel / 5);
//     // random around 20% of flare velocity
//     sparkCol[i] = sparkVel[i] * 1000;
//     sparkCol[i] = constrain(sparkCol[i], 0, 255);
//   }
//   // launch
//   FastLED.clear();
//   while (flareVel >= -.2) {
//     // sparks
//     for (int i = 0; i < 5; i++) {
//       sparkPos[i] += sparkVel[i];
//       sparkPos[i] = constrain(sparkPos[i], 0, 120);
//       sparkVel[i] += gravity;
//       sparkCol[i] += -.8;
//       sparkCol[i] = constrain(sparkCol[i], 0, 255);
//       leds[int(sparkPos[i])] = HeatColor(sparkCol[i]);
//       leds[int(sparkPos[i])] %= 50; // reduce brightness to 50/255
//     }

//     // flare
//     leds[int(flarePos)] = CHSV(0, 0, int(brightness * 255));
//     FastLED.show();
//     FastLED.clear();
//     flarePos += flareVel;
//     flareVel += gravity;
//     brightness *= .985;
//   }
// }

// /*
//  * Explode!
//  *
//  * Explosion happens where the flare ended.
//  * Size is proportional to the height.
//  */
// void explodeLoop() {
//   int nSparks = flarePos / 2; // works out to look about right

//   // initialize sparks
//   for (int i = 0; i < nSparks; i++) {
//     sparkPos[i] = flarePos; sparkVel[i] = (float(random16(0, 20000)) / 10000.0) - 1.0; // from -1 to 1
//     sparkCol[i] = abs(sparkVel[i]) * 500; // set colors before scaling velocity to keep them bright
//     sparkCol[i] = constrain(sparkCol[i], 0, 255);
//     sparkVel[i] *= flarePos / NUM_LEDS; // proportional to height
//   }
//   sparkCol[0] = 255; // this will be our known spark
//   float dying_gravity = gravity;
//   float c1 = 120;
//   float c2 = 50;
//   while(sparkCol[0] > c2/128) { // as long as our known spark is lit, work with all the sparks
//     FastLED.clear();
//     for (int i = 0; i < nSparks; i++) {
//       sparkPos[i] += sparkVel[i];
//       sparkPos[i] = constrain(sparkPos[i], 0, NUM_LEDS);
//       sparkVel[i] += dying_gravity;
//       sparkCol[i] *= .99;
//       sparkCol[i] = constrain(sparkCol[i], 0, 255); // red cross dissolve
//       if(sparkCol[i] > c1) { // fade white to yellow
//         leds[int(sparkPos[i])] = CRGB(255, 255, (255 * (sparkCol[i] - c1)) / (255 - c1));
//       }
//       else if (sparkCol[i] < c2) { // fade from red to black
//         leds[int(sparkPos[i])] = CRGB((255 * sparkCol[i]) / c2, 0, 0);
//       }
//       else { // fade from yellow to red
//         leds[int(sparkPos[i])] = CRGB(255, (255 * (sparkCol[i] - c2)) / (c1 - c2), 0);
//       }
//     }
//     dying_gravity *= .995; // as sparks burn out they fall slower
//     FastLED.show();
//   }
//   FastLED.clear();
//   FastLED.show();
// }



/*
  Aurora effect
*/
/*

#define W_MAX_COUNT 20  //Number of simultaneous waves
#define W_MAX_SPEED 6     //Higher number, higher speed
#define W_WIDTH_FACTOR 6  //Higher number, smaller waves

//24 bytes
class AuroraWave {
private:
  uint16_t ttl;
  CRGB basecolor;
  float basealpha;
  uint16_t age;
  uint16_t width;
  float center;
  bool goingleft;
  float speed_factor;
  bool alive = true;

public:
  void init(uint32_t segment_length, CRGB color) {
    ttl = random(500, 1501);
    basecolor = color;
    basealpha = random(60, 101) / (float)100;
    age = 0;
    width = random(segment_length / 20, segment_length / W_WIDTH_FACTOR);  //half of width to make math easier
    if (!width) width = 1;
    center = random(101) / (float)100 * segment_length;
    goingleft = random(0, 2) == 0;
    speed_factor = (random(10, 31) / (float)100 * W_MAX_SPEED / 255);
    alive = true;
  }

  CRGB getColorForLED(int ledIndex) {
    if (ledIndex < center - width || ledIndex > center + width) return 0;  //Position out of range of this wave

    CRGB rgb;

    //Offset of this led from center of wave
    //The further away from the center, the dimmer the LED
    float offset = ledIndex - center;
    if (offset < 0) offset = -offset;
    float offsetFactor = offset / width;

    //The age of the wave determines it brightness.
    //At half its maximum age it will be the brightest.
    float ageFactor = 0.1;
    if ((float)age / ttl < 0.5) {
      ageFactor = (float)age / (ttl / 2);
    } else {
      ageFactor = (float)(ttl - age) / ((float)ttl * 0.5);
    }

    //Calculate color based on above factors and basealpha value
    float factor = (1 - offsetFactor) * ageFactor * basealpha;
    rgb.r = basecolor.r * factor;
    rgb.g = basecolor.g * factor;
    rgb.b = basecolor.b * factor;

    return rgb;
  };

  //Change position and age of wave
  //Determine if its sill "alive"
  void update(uint32_t segment_length, uint32_t speed) {
    if (goingleft) {
      center -= speed_factor * speed;
    } else {
      center += speed_factor * speed;
    }

    age++;

    if (age > ttl) {
      alive = false;
    } else {
      if (goingleft) {
        if (center + width < 0) {
          alive = false;
        }
      } else {
        if (center - width > segment_length) {
          alive = false;
        }
      }
    }
  };

  bool stillAlive() {
    return alive;
  };
};

//uint16_t WS2812FX::mode_aurora(void) {
  uint16_t mode_aurora(void) {
  //aux1 = Wavecount
  //aux2 = Intensity in last loop

  AuroraWave* waves;

  if (SEGENV.aux0 != SEGMENT.intensity || SEGENV.call == 0) {
    //Intensity slider changed or first call
    SEGENV.aux1 = map(SEGMENT.intensity, 0, 255, 2, W_MAX_COUNT);
    SEGENV.aux0 = SEGMENT.intensity;

    // if (!SEGENV.allocateData(sizeof(AuroraWave) * SEGENV.aux1)) {  // 26 on 32 segment ESP32, 9 on 16 segment ESP8266
    //   return mode_static();                                        //allocation failed
    //}

    waves = reinterpret_cast<AuroraWave*>(SEGENV.data);

    for (int i = 0; i < SEGENV.aux1; i++) {
      waves[i].init(SEGLEN, col_to_crgb(color_from_palette(random8(), false, false, random(0, 3))));
    }
  } else {
    waves = reinterpret_cast<AuroraWave*>(SEGENV.data);
  }

  for (int i = 0; i < SEGENV.aux1; i++) {
    //Update values of wave
    waves[i].update(SEGLEN, SEGMENT.speed);

    if (!(waves[i].stillAlive())) {
      //If a wave dies, reinitialize it starts over.
      waves[i].init(SEGLEN, col_to_crgb(color_from_palette(random8(), false, false, random(0, 3))));
    }
  }

  uint8_t backlight = 1;  //dimmer backlight if less active colors
  if (SEGCOLOR(0)) backlight++;
  if (SEGCOLOR(1)) backlight++;
  if (SEGCOLOR(2)) backlight++;
  //Loop through LEDs to determine color
  for (int i = 0; i < SEGLEN; i++) {
    CRGB mixedRgb = CRGB(backlight, backlight, backlight);

    //For each LED we must check each wave if it is "active" at this position.
    //If there are multiple waves active on a LED we multiply their values.
    for (int j = 0; j < SEGENV.aux1; j++) {
      CRGB rgb = waves[j].getColorForLED(i);

      if (rgb != CRGB(0)) {
        mixedRgb += rgb;
      }
    }

    setPixelColor(i, mixedRgb[0], mixedRgb[1], mixedRgb[2]);
  }

  return FRAMETIME;
}
*/

void candyCane(int numColors){
  CRGB color1 = CRGB::White;  // color used between color 2 (and 3 if used)
  CRGB color2 = CRGB::Red;
  //CRGB color3 = CHSV(0,170,255);  //optional 3rd color
  CRGB color3 = CRGB(0,255,0);  //optional 3rd color
  //uint16_t travelSpeed = 1000-9*myData.lightSpeed;
  int shiftBy = 1;  //shiftBy can be positive or negative (to change direction)
  //static uint8_t numColors = 3;  // Can be either 2 or 3
  static uint8_t stripeLength = 5;  //number of pixels per color
  static int offset;

  EVERY_N_SECONDS(4) {
    numColors = random8(2,4);  //picks either 2 or 3
    stripeLength = random8(3,6);  //picks random length
   // Serial.println("travelspeed " + String(travelSpeed));
  }

  //EVERY_N_MILLISECONDS(travelSpeed) {
    if (numColors==2) {
      for (uint8_t i=0; i<NUM_LEDS; i++){
        if ( (i+offset)%((numColors)*stripeLength)<stripeLength ) {
          leds[i] = color2;
        } else {
          leds[i] = color1;
        }
      }
    }

    if (numColors==3) {
      for (uint8_t i=0; i<NUM_LEDS; i++){
        if ( (i+offset)%((numColors+1)*stripeLength)<stripeLength ) {
          leds[i] = color2;
        }
        else if ( (i+offset+(2*stripeLength))%((numColors+1)*stripeLength)<stripeLength ) {
          leds[i] = color3;
        } else {
          leds[i] = color1;
        }
      }
    }

    //FastLED.show();

    offset = offset + shiftBy;
    if (shiftBy>0) {  //for positive shiftBy
      if (offset>=NUM_LEDS) offset = 0;
    } else {  //for negitive shiftBy
      if (offset<0) offset = NUM_LEDS;
    }

 // }//end EVERY_N
}//end candyCane
