// ************************************************************************
// *                                                                      *
// *  Disciplina: Fundamentos de Termodinâmica e Calor                    *
// *                                                                      *
// *  Professor: Diego Moro                                               *  
// *                                                                      *
// *  Alunos: Lucas de Carvalho Santos, Antônio Victor Gonçalves Dias,    *
// *  Jhenifer Cristina Pereira, Taís Eduarda Vieira Miranda              *
// *                                                                      *
// ************************************************************************

// IMPORTAÇÃO DAS BIBLIOTECAS
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <PID_v1.h>

// DEFINIÇÃO DE CONSTANTES PRÉ-COMPILADAS
#define PIN_tempSensor 25
#define PIN_controle 16
#define Min_PWN 4095
#define Max_PWN 1200

// CONSTRUTOR DA BIBLIOTECA DO SENSOR UTILIZADO
OneWire oneWire(PIN_tempSensor);
DallasTemperature sensor(&oneWire);

// VARIÁVEIS UTILIZADAS NO PID
double entrada, saida, setPoint;
float erro = 0;
double ultimoErro, erroAcumulado;

// CONSTANTES DO PID
double Kp=6, Ki=1.6, Kd=0.2;

// CONSTANTES DO PWM
const int freq = 5000;
const int MVChannel = 0;
const int resolution = 12;

// CONSTRUTOR DA BIBLIOTECA PID
PID controladorPID(&entrada, &saida, &setPoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  setPoint = 4;
  Serial.begin(115200);
  while (!Serial);
  pinMode(PIN_tempSensor, INPUT);
  pinMode(PIN_controle, OUTPUT);

  ledcSetup(MVChannel, freq, resolution);
  ledcAttachPin(PIN_controle, MVChannel);

  controladorPID.SetMode(AUTOMATIC);
  analogReadResolution(12);
  sensor.begin();
  controladorPID.SetOutputLimits(0, 100);

  // INICIA A BOMBA PARA VENCER A INÉRCIA
  for (int i = 1200; i <= 4095; i++)
  {
      Serial.println(i);
      ledcWrite(MVChannel, i);
      if (i == 4095)
      {
        delay(4800);
      }
  }
  
  // LENGENDA PARA O PLOT DAS VARIÁVEIS
  Serial.println("min, max, SetPoitn, temperatura, saidaPID");
}

void loop()
{
  // PARA INDICAÇÃO DE ESCALA MÍNIMA E MÁXIMA NO PLOTTER SERIAL (ARDUINO)
  Serial.print(0);
  Serial.print(",");
  Serial.print(4095);
  Serial.print(",");
  Serial.print(setPoint);
  Serial.print(",");

  // CAPTURA A INDICAÇÃO DO SENSOR DE TEMPERATURA
  sensor.requestTemperatures();
  entrada = sensor.getTempCByIndex(0);

  // CALCULA A SAÍDA DO PID
  controladorPID.Compute();

  // MOSTRA NA TELA
  Serial.print(entrada);
  Serial.print(",");
  Serial.print(saida);
  Serial.println();  
  // MAPEIA A VARIÁVEL MANIPULADA, CAPTURA A SAÍDA DA MV E CONVERTE PARA A RESOLUÇÃO ANALÓGICA DO PWM QUE SERÁ APLICADA NA BOMBA
  ledcWrite(MVChannel, map(saida, 0, 100, 1200, 4095));
  delay(15);
}
