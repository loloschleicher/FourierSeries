#include <U8glib.h>
#include "arduinoFFT.h"


// Crear objeto pantalla
U8GLIB_ST7920_128X64 u8g(13, 11, 12, U8G_PIN_NONE);
//ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear prescaler bit
//ADCSRA |= bit (ADPS0) | bit (ADPS1) | bit (ADPS2);   // 128

#define MUESTRAS 128             // Potencia de 2
#define FRECUENCIA_SAMPLEO 1000 //Hz


arduinoFFT FFT = arduinoFFT();

unsigned int periodo;
unsigned long tiempo;
/*#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03*/

double vReal[MUESTRAS];
double vImag[MUESTRAS];

void draw(double* datos)
{
  int index = 0;
  for (int i = 1; i < 128; i += 5)
  {
    if(datos[index] > 127)
    {
      //(x, y, ancho, alto)
      u8g.drawBox(i, 1, 2, 50);
    }
    else{
      u8g.drawBox(i, 1, 2, (int)datos[index]);
      
    }
    ++index;
  }
}

void setup(void)
{
  Serial.begin(9600);
  periodo = round(1000000 * (1.0 / FRECUENCIA_SAMPLEO));
}

void loop(void) {
  Serial.println(analogRead(0));
  u8g.firstPage();
  //obtener muestras
  for (int i = 0; i < MUESTRAS; i++)
  {
    tiempo = micros();

    vReal[i] = analogRead(0);
    vImag[i] = 0;


    while (micros() < (tiempo + periodo)) {
    }
    tiempo += periodo;
  }

  // realizar fft
  FFT.Windowing(vReal, MUESTRAS, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, MUESTRAS, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, MUESTRAS);

  //graficar
  do {
    draw(vReal);

  } while ( u8g.nextPage() );
 // delay(200);

 //PrintVector(vReal, (MUESTRAS >> 1), SCL_FREQUENCY);
}

/*void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 10; i < 40; i++)
  {
    double abscissa;
   
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / FRECUENCIA_SAMPLEO);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * FRECUENCIA_SAMPLEO) / MUESTRAS);
  break;
    }
    Serial.print(abscissa, 6);
    if(scaleType==SCL_FREQUENCY)
      Serial.print("Hz");
    Serial.print(" ");
    Serial.println(vData[i], 4);
  }
  Serial.println();
}*/
