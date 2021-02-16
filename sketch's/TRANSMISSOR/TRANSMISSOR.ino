//________________INCLUSÃO DE BIBLIOTECAS________________   Transmissor
#include <RF24_config.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <SPI.h>

//________________INSTANCIANDO OBJETOS________________
RF24 radio (7, 8);                       //Instanciando o nRF24L01

//________________MAPEAMENTO DE HARDWARE________________
#define throttle   A4                    // Esquerdo Y
#define yaw        A5                    // Esquerdo X
#define pitch      A2                    // Direito  Y
#define roll       A3                    // Direito  X
#define travamento 3                     // Alavanca de travamento
#define modo       4                     // Alavanca de Modo de Veiculo

//________________VARIAVEIS DA TELIMETRIA________________
byte enderecos[][6] = {"1node", "2node"};//Definindo os dois tubos de dados que serão utilizados
bool estadoAnteriorRadio, alertaRadio;
unsigned long timePerda, timeAlerta;     //Armazenamento de valores de tempo
struct estruturaE {
  int throttleValue;                     //Armazenamento de Throttle
  int yawValue;                          //Armazenamento de Yaw
  int pitchValue;                        //Armazenamento de Pitch
  int rollValue;                         //Armazenamento de Roll
  bool travamentoValue;                  //Armazenamento da alavanca de travamento
  bool modoValue;                        //Armazenamento da alavanca de modo de veiculo
};
struct estruturaR {
  int battery;                           //Armazenamento da bateria do drone
  float latitudeGPS;                     //Armazenamento da latitude do GPS
  float longitudeGPS;                    //Armazenamento da longitude do GPS
  float satelitesGPS;                    //Armazenamento do numero de satelites conectados ao GPS
};
typedef estruturaE dadosE;               //Definindo a estrutura1 como um tipo de dado
typedef estruturaR dadosR;               //Definindo a estrutura2 como um tipo de dado
dadosR dadosRecebidos;                   //Criando uma estrutura para enviar os dados
dadosE dadosEnviados;                    //Criando uma estrutura para receber os dados
void setup() {
  //________________INICIANDO PERIFÉRICOS________________
  Serial.begin(9600);                    //Inicializando o monitor serial para testes
  radio.begin();                         //Inicializandoo o nRF24l01
  radio.openWritingPipe(enderecos[0]);   //Definindo o tubo de escrita do nRF24l01
  radio.openReadingPipe(1, enderecos[1]);//Definindo o tubo de leitura do nRF24l01
  radio.startListening();                //Pondo o radio em modo escuta
}

void loop() {
  if (alertaRadio) {
    
  }
  //________________COLETANDO DADOS________________
  //dadosEnviados.throttleValue   = analogRead(throttle);    //Armazenando o valor de throttle na estrutura
  dadosEnviados.throttleValue = 50;
  dadosEnviados.yawValue        = analogRead(yaw);         //Armazenando o valor de yaw na estrutura
  dadosEnviados.pitchValue      = analogRead(pitch);       //Armazenando o valor de pitch na estrutura
  dadosEnviados.rollValue       = analogRead(roll);        //Armazenando o valor de roll na estrutura
  dadosEnviados.travamentoValue = digitalRead(travamento); //Armazenando o valor da alavanda de travamento na estrutura
  dadosEnviados.modoValue       = digitalRead(modo);       //Armazenando o valor da alavanca de modo de veiculo na estrutura
  
  radio.stopListening();
  radio.write(&dadosEnviados, sizeof(dadosE));
  delay(20);
  radio.startListening();
  if (radio.available()) {
    radio.read(&dadosRecebidos, sizeof(dadosR));
    Serial.println("DADOS RECEBIDOS");
    estadoAnteriorRadio = HIGH;
  } else {
    Serial.println("DADOS PERDIDOS"); 
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
  delay(20);
}
