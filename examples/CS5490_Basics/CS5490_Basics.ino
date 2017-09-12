

#include<CS5490.h>


CS5490 line(MCLK_default,14,12);


void setup() {
  //Inicializa a comunicação com o CS5490
  //600 é a velocidade default do CI para comunicação serial
  line.begin(600);
  //Inicializa a comunicação com o PC para debug
  Serial.begin(115200);

  //Send 21 instruct
  //Seta Conversão p/ continuo
  line.CC();
}

void loop() {

  //Obtem informação da configuração de calibração do ganho de corrente
  line.getInstantV();

  //Printa essa informação no Serial Monitor
  Serial.println("Valor instantâneo é: ");
  Serial.println(line.data[0],HEX);
  Serial.println(line.data[1],HEX);
  Serial.println(line.data[2],HEX);
  Serial.println("");

  //Obtem informação da configuração de calibração do ganho de corrente
  line.getPeakI();

  //Printa essa informação no Serial Monitor
  Serial.println("Valor de pico é: ");
  Serial.println(line.data[0],HEX);
  Serial.println(line.data[1],HEX);
  Serial.println(line.data[2],HEX);
  Serial.println("");



  delay(1000);
}
