//________________INCLUSÃO DE BIBLIOTECAS________________   Drone
#include <SPI.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Servo.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <RF24_config.h>

//________________INSTANCIANDO OBJETOS________________
RF24 radio (7, 8);                        //Instanciando o nRF24L01
const int MPU = 0x68;                     //Define o endereço da MPU
SoftwareSerial serialGPS(3, 2);           //Instanciando o RX, TX do gps
TinyGPS gps;                              //Instanciando o GPS
Servo motor1;                             //Instanciando o motor 1
Servo motor2;                             //Instanciando o motor 2
Servo motor3;                             //Instanciando o motor 3
Servo motor4;                             //Instanciando o motor 4

//________________MAPEAMENTO DE HARDWARE________________
#define bateria A0                        //Definindo o pino do sensor de tensão
#define clk     A1                        //Definindo o pino clock do 74HC585    
#define letch   A2                        //Definindo o pino letch do 74HC585
#define data    A3                        //Definindo o pino data  do 74HC585

//________________VARIAVEIS DA TELIMETRIA________________
byte enderecos[][6] = {"1node", "2node"}; //Definindo os dois tubos de dados que serão utilizados
bool estadoAnteriorRadio, alertaRadio;   
unsigned long timePerda, timeAlerta;      //Armazenamento de valores de tempo
struct estruturaR {
  int throttleValue;                      //Armazenamento de Throttle
  int yawValue;                           //Armazenamento de Yaw
  int pitchValue;                         //Armazenamento de Pitch
  int rollValue;                          //Armazenamento de Roll
  bool travamentoValue;                   //Armazenamento da alavanca de travamento
  bool modoValue;                         //Armazenamento da alavanca de modo de veiculo
};
struct estruturaE {
  int battery;                            //Armazenamento da bateria do drone
  float latitudeGPS;                      //Armazenamento da latitude do GPS
  float longitudeGPS;                     //Armazenamento da longitude do GPS
  unsigned short satelitesGPS;            //Armazenamento do numero de satelites conectados ao GPS
};
typedef estruturaE dadosE;                //Definindo a estrutura1 como um tipo de dado
typedef estruturaR dadosR;                //Definindo a estrutura2 como um tipo de dado
dadosR dadosRecebidos;                    //Criando uma estrutura para enviar os dados
dadosE dadosEnviados;                     //Criando uma estrutura para receber os dados   

//________________VARIAVEIS PARA ESTABILIZAÇÃO________________
const int n = 15;                         //Constante de colunas para media movel
int valueAcgy[7][n];                      //Valores armazenados do MPU
float PIDx, PIDy, errorX, errorY, previous_errorX, previous_errorY, timed;
float  px = 0,      py = 0,
       ix = 0,      iy = 0,
       dx = 0,      dy = 0;
double kpx = 1.0,   kpy = 1.0,
       kix = 0.0,   kiy = 0.0,
       kdx = 0.0,   kdy = 0.0;
float setpointX = 0.0;                    //Valor para o drone ser estabilizado no eixo X
float setpointY = 0.0;                    //Valor para o drone ser estabilizado no euxo Y
int motorA, motorB, motorC, motorD;
int conjunto1, conjunto2, conjunto3, conjunto4;

void setup() {
  //________________INICIANDO PERIFÉRICOS________________
  Serial.begin(9600);                     //Inicializando o monitor 
  radio.begin();                          //Inicializando o nRF24l01
  radio.openWritingPipe(enderecos[1]);    //Definindo o tubo de escrita do nRF24l01
  radio.openReadingPipe(1, enderecos[0]); //Definindo o tubo de leitura do nRF24l01
  radio.startListening();                 //Pondo o radio em modo escuta
  Wire.begin();                           //Inicializa a comunicação I2C
  Wire.beginTransmission(MPU);            //Inicializa a comunicação I2C com o MPU
  Wire.write(0x6B);                       //|
  Wire.write(0);                          //Inicializa o modulo MPU
  Wire.endTransmission(true);             //|
  serialGPS.begin(9600);                  //Inicio do GPS

  //________________SETANDO PINOS________________
  pinMode(bateria, INPUT);                //Definindo o pino do sensor de tensão como entrada
  pinMode(clk,    OUTPUT);                //Definindo o pino clk   do 74HC595
  pinMode(letch,  OUTPUT);                //Definindo o pino letch do 74HC595
  pinMode(data,   OUTPUT);                //Definindo o pino data  do 74HC595
}

void loop() {
  //________________COLORAÇÃO DOS LEDS________________
  if (alertaRadio) {
    if ((millis() - timeAlerta) < 500) {
      shiftOut(data, clk, LSBFIRST, 0x66);
      attShift();
    }
    if ((millis() - timeAlerta) > 500) {
      shiftOut(data, clk, LSBFIRST, 0xD0);
      attShift();
    }
    if ((millis() - timeAlerta) >= 1000) {
      timeAlerta = millis();
    }
  } else {
    if ((millis() - timeAlerta) < 250) {
      shiftOut(data, clk, LSBFIRST, 0xA8);
      attShift();
    }
    if ((millis() - timeAlerta) > 250) {
      shiftOut(data, clk, LSBFIRST, 0xE0);
      attShift();
    } 
    if ((millis() - timeAlerta) >= 1000) {
      timeAlerta = millis();
      attShift();
    }
  }
  
  //________________TRABALHANDO COM DADOS DO GPS________________
  bool gpsRecebido = false;
  while (serialGPS.available()) {
    char gpsIN = serialGPS.read();
    gpsRecebido = gps.encode(gpsIN);
  }  
  
  unsigned long idadeInformacaoGPS;
  gps.f_get_position(&dadosEnviados.latitudeGPS, &dadosEnviados.longitudeGPS, idadeInformacaoGPS);
  dadosEnviados.satelitesGPS = gps.satellites();
  
  //________________MEDINDO E FAZENDO A MEDIA MOVEL DA MPU________________
  for (int i = 0; i < 7; i++) {
    for (int ii = n - 1; ii > 0; ii--) {
      valueAcgy[i][ii] = valueAcgy[i][ii - 1];
    };
  }
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);
  valueAcgy[0][0] = Wire.read()<<8|Wire.read();
  valueAcgy[1][0] = Wire.read()<<8|Wire.read();
  valueAcgy[2][0] = Wire.read()<<8|Wire.read();
  valueAcgy[3][0] = Wire.read()<<8|Wire.read();
  valueAcgy[4][0] = Wire.read()<<8|Wire.read();
  valueAcgy[5][0] = Wire.read()<<8|Wire.read();
  valueAcgy[6][0] = Wire.read()<<8|Wire.read();
  float moving_avarage[7] = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 7; i++) {
    for (int ii = 0; ii < n; ii++) {
      moving_avarage[i] += valueAcgy[i][ii];
    }
  }
  for(int i = 0; i < 7; i++) moving_avarage[i] = moving_avarage[i] / n;
  moving_avarage[3] = moving_avarage[3] / 340.00 + 36.53;
  double pitch = atan(moving_avarage[0] / sqrt(moving_avarage[1] * moving_avarage[1] + moving_avarage[2] * moving_avarage[2])) * RAD_TO_DEG;
  double roll  = atan(moving_avarage[1] / sqrt(moving_avarage[0] * moving_avarage[0] + moving_avarage[2] * moving_avarage[2])) * RAD_TO_DEG;
  
  

  //________________CALCULO PID DOS MOTORES________________
  double errorX = setpointX - pitch;
  double errorY = setpointY - roll;
  float delta = (millis() - timed) / 1000.0;
  timed = millis();
  py = kpy * errorY;
  px = kpx * errorX;
  iy += (kiy * errorY) * delta;
  ix += (kix * errorX) * delta;
  dy = errorY * kdy / delta;
  dx = errorX * kdx / delta;
  PIDx = px + ix + dx;
  PIDy = py + iy + dy;
  motorA = dadosRecebidos.throttleValue + PIDx + PIDy;
  motorB = dadosRecebidos.throttleValue - PIDx + PIDy;
  motorC = dadosRecebidos.throttleValue + PIDx - PIDy;
  motorD = dadosRecebidos.throttleValue - PIDx - PIDy;

  //Serial.print(" PITCH: "); Serial.print(pitch); Serial.print("\t\tROLL: "); Serial.println(roll);
  //Serial.print(" MOTOR A: "); Serial.print(motorA); Serial.print("\t MOTOR B: "); Serial.print(motorB);Serial.print("\t MOTOR C: "); Serial.print(motorC); Serial.print("\t MOTOR D: "); Serial.println(motorD);
  Serial.print(" LATITUDE: "); Serial.print(dadosEnviados.latitudeGPS, 7); Serial.print("\t LONGITUDE: "); Serial.print(dadosEnviados.longitudeGPS, 7); Serial.print("\t SATELITES CONECTADOS: "); Serial.println(dadosEnviados.satelitesGPS);
  
  //________________ESTRUTURA PRINCIPAL________________
  radio.startListening();
  if (radio.available()) {
    radio.read(&dadosRecebidos, sizeof(dadosR));
    estadoAnteriorRadio = HIGH;
  } else {
    if (estadoAnteriorRadio == HIGH) {
      estadoAnteriorRadio = LOW;
      timePerda = millis();
    }
  }
  if (((millis() - timePerda) > 3000) && (estadoAnteriorRadio == LOW)) {
    alertaRadio = 1;
    return 0; 
  } else {
    alertaRadio = 0;
  }
  delay(21);
  radio.stopListening(); 
  radio.write(&dadosEnviados, sizeof(dadosE));
  delay(21);
}

void attShift() {
  digitalWrite(letch, LOW); 
  digitalWrite(letch, HIGH);
  digitalWrite(letch, LOW);
  return 0;
}
