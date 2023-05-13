// Polar basics demo for the 
// FastLED Podcast #2
// https://www.youtube.com/watch?v=KKjFRZFBUrQ
//
// VO.1 preview version
// by Stefan Petrick 2023
// This code is licenced under a 
// Creative Commons Attribution 
// License CC BY-NC 3.0

///
//This code was modified by Clark Reinholtz to support 3d pixel mapping
//

#include <FastLED.h>
#include <FLOAT.h>


#define NUM_LEDS 50


float ledMap[NUM_LEDS][3] = {

    // X       Y       Z
    // -LR+ -INOUT+ -DOWNUP+

    { 7.5 , 0, 13},
    { 7.25 , 0, 9},
    { 7.25 , 0, 5.5},
    { 7.5 , 0, 1.5},

    { 10 , 0, 0},
    { 11 , 0, 4},
    { 10.5 , 0, 7.5},
    { 10.5 , 0, 11},
    { 11 , 0, 14.5},

    { 11.5 , 2, 13},
    { 11.5 , 2, 9},
    { 11.5 , 2, 5},
    { 11.5 , 2, 1.5},

    { 11.5 , 4, 0},
    { 11.5 , 4.5, 4},
    { 12.5 , 4.25, 7.5},
    { 12 , 4, 11},
    { 11.5 , 5, 14.5},

    { 9 , 6, 12.5},
    { 9.5 , 6, 9},
    { 10 , 6, 5},
    { 9 , 6, 1.5},

    { 6.5 , 6, 1.5},
    { 7 , 7, 5}, 
    { 7.5 , 6.5, 9},
    { 7 , 6, 12.5},

    { 6 , 6, 14.5},
    { 4 , 6.5, 12.5},
    { 4.5 , 6, 9},
    { 5 , 6, 5},
    { 4 , 6, 1.5},

    { 1 , 6, 1.5},
    { 1.5 , 7, 5}, 
    { 1.5 , 6, 9},
    { 1 , 6, 12.5},

    { 0 , 5, 14},
    { 0 , 4, 12},
    { 0 , 4.5, 9},
    { 0 , 5, 5},
    { 0 , 4, 0},

    { -.25 , .75, 1.5},
    { -1 , 1, 5.5},
    { -.5 , 1, 9.5},
    { 0 , 1.25, 13},

    { 1, 0, 14},
    { 2, 0, 11.5},
    { 2, 0, 8},
    { 1.75, 0, 4},
    { 2.25, 0, 0},
    { 4.5, 0, 4}
    
};






float runtime;                          // elapse ms since startup
float newdist, newphi, newtheta;                // parameters for image reconstruction
float offset_x, offset_y, offset_z;               // wanna shift the cartesians during runtime?
float scale_x, scale_y, scale_z;                 // cartesian scaling in 2 dimensions
float dist, theta, phi;                 // the actual polar coordinates

float x, y, z;                               // the cartesian coordiantes
//int num_x = WIDTH;                      // horizontal pixel count
//int num_y = HEIGHT;                     // vertical pixel count

// Background for setting the following 2 numbers: the FastLED inoise16() function returns
// raw values ranging from 0-65535. In order to improve contrast we filter this output and
// stretch the remains. In histogram (photography) terms this means setting a blackpoint and
// a whitepoint. low_limit MUST be smaller than high_limit.

uint16_t low_limit  = 30000;            // everything lower drawns in black
                                        // higher numer = more black & more contrast present
uint16_t high_limit = 50000;            // everything higher gets maximum brightness & bleeds out
                                        // lower number = the result will be more bright & shiny

float center_x = 6;     // the reference point for polar coordinates
float center_y = 3;     // (can also be outside of the actual xy matrix)
float center_z = 7;     // (can also be outside of the actual xy matrix)

//float center_x = 20;                  // the reference point for polar coordinates
//float center_y = 20;                

CRGB leds[NUM_LEDS];               // framebuffer

float lookup_theta   [NUM_LEDS];          // look-up table for all angles
float lookup_phi   [NUM_LEDS];          // look-up table for all angles
float lookup_distance[NUM_LEDS];          // look-up table for all distances
float vignette[NUM_LEDS];
float inverse_vignette[NUM_LEDS];

float spd;                            // can be used for animation speed manipulation during runtime

float show1, show2, show3, show4, show5; // to save the rendered values of all animation layers
float red, green, blue;                  // for the final RGB results after the colormapping

float c, d, e, f;                                                   // factors for oscillators
float linear_c, linear_d, linear_e, linear_f;                       // linear offsets
float angle_c, angle_d, angle_e, angle_f;                           // angle offsets
float noise_angle_c, noise_angle_d, noise_angle_e, noise_angle_f;   // angles based on linear noise travel
float dir_c, dir_d, dir_e, dir_f;                                   // direction multiplicators



void setup() {

    Serial.begin(115200);                 // check serial monitor for current fps count
    
    // Teensy users: make sure to use the hardware SPI pins 11 & 13
    // for best performance
    
    //FastLED.addLeds<APA102, 11, 13, BGR, DATA_RATE_MHZ(12)>(leds, NUM_LEDS); //teensy
    
    FastLED.addLeds<WS2813, 2, GRB>(leds, NUM_LEDS); // esp??  //teen
    

    
    // FastLED.addLeds<NEOPIXEL, 13>(leds, NUM_LEDS);   
   
    render_polar_lookup_table();          // precalculate all polar coordinates 
                                          // to improve the framerate
 }


void loop() {

    // set speedratios for the offsets & oscillators
    
    spd = 0.01  ;
    c   = 0.1  ;
    d   = 0.231   ;
    e   = 0.30137  ;
    f   = 0.49098  ;

    calculate_oscillators();     // get linear offsets and oscillators going
    
    // ...and now let's generate a frame 

    for (int n = 0; n < NUM_LEDS; n++) {

        // pick polar coordinates from look the up table
        // this is the actual position of the pixel
        dist  = lookup_distance [n];
        theta = lookup_theta    [n];
        phi = lookup_phi    [n];

        // Generation of one layer. Explore the parameters and what they do.

        //1/////////////////////////////////////////////////////
        scale_x  = 5000;                       // smaller value = zoom in, bigger structures, less detail
        scale_y  = 5000;                       // higher = zoom out, more pixelated, more detail
        scale_z  = 5000;
        newtheta = theta;   //swirlLR
        newphi = phi; // swirl DU
        newdist  = dist;
        offset_x = linear_d*5.0;                        // must be >=0
        offset_y = 0;                        // must be >=0
        offset_z = 0;                        // must be >=0


        //only thing changes is x offset
        show1 = render_pixel();


        //2///////////////////////////////////////
        scale_x  = 5000;                       // smaller value = zoom in, bigger structures, less detail
        scale_y  = 5000;                       // higher = zoom out, more pixelated, more detail
        scale_z  = 5000;
        newtheta = theta;   //swirlLR
        newphi = phi; // swirl DU
        newdist  = dist;
        offset_x = 0;                        // must be >=0
        offset_y = linear_e*.5;                        // must be >=0
        offset_z = 1000;                        // must be >=0      

        //only thing changes is y offset
        show2 = render_pixel();


        //3///////////////////////////////////////
        scale_x  = 5000;                       // smaller value = zoom in, bigger structures, less detail
        scale_y  = 5000;                       // higher = zoom out, more pixelated, more detail
        scale_z  = 5000;
        newtheta = theta + angle_f;   //swirlLR
        newphi = phi; // swirl DU
        newdist  = dist;
        offset_x = linear_c;                        // must be >=0
        offset_y = 0;                        // must be >=0
        offset_z = 1000;                        // must be >=0    

        //swirl and X scroll 
        show3 = render_pixel();
        
                
        // Colormapping - Assign rendered values to colors 
        red   = 0;
        red   = show1;  // comment this line to hide
        
        green = 0;
        //green = show3;  // comment this line to hide
        
        blue  = 0;
        //blue  = show2;  // comment this line to hide


/*
float linear_c, linear_d, linear_e, linear_f;                       // linear offsets
float angle_c, angle_d, angle_e, angle_f;                           // angle offsets
float noise_angle_c, noise_angle_d, noise_angle_e, noise_angle_f;   // angles based on linear noise travel
float dir_c, dir_d, dir_e, dir_f;                                   // direction multiplicators

 */
        // Check the final results.
        // Discard faulty RGB values & write the valid results into the framebuffer.
        
        write_pixel_to_framebuffer(n);

    }

    // BRING IT ON! SHOW WHAT YOU GOT!
    FastLED.show();

    // check serial monitor for current performance data
    EVERY_N_MILLIS(500) report_performance();

} 
//-----------------------------------------------------------------------------------end main loop --------------------

void calculate_oscillators() {
    
    runtime = millis();                          // save elapsed ms since start up

    runtime = runtime * spd;                     // global anaimation speed

    linear_c = runtime * c;                      // some linear rising offsets 0 to max
    linear_d = runtime * d;
    linear_e = runtime * e;
    linear_f = runtime * f;

    angle_c = fmodf(linear_c, 2 * PI);           // some cyclic angle offsets  0 to 2*PI
    angle_d = fmodf(linear_d, 2 * PI);
    angle_e = fmodf(linear_e, 2 * PI);
    angle_f = fmodf(linear_f, 2 * PI);

    dir_c = sinf(angle_c);                       // some direction oscillators -1 to 1
    dir_d = sinf(angle_d);
    dir_e = sinf(angle_e);
    dir_f = sinf(angle_f);

    uint16_t noi;
    noi =  inoise16(10000 + linear_c * 100000);    // some noise controlled angular offsets
    noise_angle_c = map_float(noi, 0, 65535 , 0, 4*PI);
    noi =  inoise16(20000 + linear_d * 100000);
    noise_angle_d = map_float(noi, 0, 65535 , 0, 4*PI);
    noi =  inoise16(30000 + linear_e * 100000);
    noise_angle_e = map_float(noi, 0, 65535 , 0, 4*PI);
    noi =  inoise16(40000 + linear_f * 100000);
    noise_angle_f = map_float(noi, 0, 65535 , 0, 4*PI);
}


// given a static polar origin we can precalculate 
// all the (expensive) polar coordinates

void render_polar_lookup_table() {

    for (int n = 0; n < NUM_LEDS; n++) {

        float dx = ledMap[n][0] - center_x;
        float dy = ledMap[n][1] - center_y;
        float dz = ledMap[n][2] - center_z;

        lookup_distance[n] = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        lookup_theta[n] = atan2f(dy,dx);
        lookup_phi[n] = acosf(dz / lookup_distance[n]);

    }
}


// convert polar coordinates back to cartesian
// & render noise value there

float render_pixel() {

    // convert polar coordinates back to cartesian ones

    float newx = (offset_x + center_x - (newdist * sinf(newphi) * cosf(newtheta))) * scale_x;
    float newy = (offset_y + center_y - (newdist * sinf(newphi) * sinf(newtheta))) * scale_y;
    float newz = (offset_z + center_z - (newdist * cosf(newphi))) * scale_z;

    // render noisevalue at this new cartesian point

    //uint16_t raw_noise_field_value = inoise16(newx, newy, z);
    uint16_t raw_noise_field_value =inoise16(newx, newy, newz);

    // a lot is happening here, namely
    // A) enhance histogram (improve contrast) by setting the black and white point
    // B) scale the result to a 0-255 range
    // it's the contrast boosting & the "colormapping" (technically brightness mapping)

    if (raw_noise_field_value < low_limit)  raw_noise_field_value =  low_limit;
    if (raw_noise_field_value > high_limit) raw_noise_field_value = high_limit;

    float scaled_noise_value = map_float(raw_noise_field_value, low_limit, high_limit, 0, 255);

    return scaled_noise_value;

    // done, we've just rendered one color value for one single pixel
}


// float mapping maintaining 32 bit precision
// we keep values with high resolution for potential later usage

float map_float(float x, float in_min, float in_max, float out_min, float out_max) { 
  
  float result = (x-in_min) * (out_max-out_min) / (in_max-in_min) + out_min;
  if (result < out_min) result = out_min;
  if( result > out_max) result = out_max;

  return result; 
}


// Avoid any possible color flicker by forcing the raw RGB values to be 0-255.
// This enables to play freely with random equations for the colormapping
// without causing flicker by accidentally missing the valid target range.

void rgb_sanity_check() {

      // rescue data if possible: when negative return absolute value
      if (red < 0)     red = abs(red);
      if (green < 0) green = abs(green);
      if (blue < 0)   blue = abs(blue);
      
      // discard everything above the valid 0-255 range
      int maxBright=128;
      red = map(red , 0, 255, 0, maxBright);
      green = map(green , 0, 255, 0, maxBright);
      blue = map(blue , 0, 255, 0, maxBright);

      //probably uneeded
      if (red   > maxBright)   red = maxBright;
      if (green > maxBright) green = maxBright;
      if (blue  > maxBright)  blue = maxBright;
   
}


// check result after colormapping and store the newly rendered rgb data

void write_pixel_to_framebuffer(int n) {
  
      // the final color values shall not exceed 255 (to avoid flickering pixels caused by >255 = black...)
      // negative values * -1 

      rgb_sanity_check();

      CRGB finalcolor = CRGB(red, green, blue);
     
      // write the rendered pixel into the framebutter
      leds[n] = finalcolor;
}

// make it look nicer - expand low brightness values and compress high brightness values,
// basically we perform gamma curve bending for all 3 color chanels,
// making more detail visible which otherwise tends to get lost in brightness

void adjust_gamma() {
  for (uint16_t i = 0; i < NUM_LEDS; i++)
  {
    leds[i].r = dim8_video(leds[i].r);
    leds[i].g = dim8_video(leds[i].g);
    leds[i].b = dim8_video(leds[i].b);   //CPR 
  }
}


// show current framerate and rendered pixels per second

void report_performance() {
 
  int fps = FastLED.getFPS();           // frames per second
  int kpps = (fps * NUM_LEDS) / 1000;   // kilopixel per second

  Serial.print(kpps); Serial.print(" kpps ... ");
  Serial.print(fps); Serial.print(" fps @ ");
  Serial.print(1.0/float(fps)); Serial.print(" period @ ");
  Serial.print(NUM_LEDS); Serial.println(" LEDs ... ");
}
