

#include<CS5490.h>


CS5490 line(MCLK_default,14,12);


void setup() {
  //Inicializa a comunicação com o CS5490
  //600 é a velocidade default do CI para comunicação serial
  line.begin(600);
  //Inicializa a comunicação com o PC para debug
  Serial.begin(9600);
}

void loop() {
 
  //Obtem informação da configuração de calibração do ganho de corrente
  line.getGainI();


  //Printa essa informação no Serial Monitor
  Serial.println("A informação coletada é: ");
  Serial.println(line.data[0]);
  Serial.println(line.data[1]);
  Serial.println(line.data[2]);
  Serial.println("\n\n");

  delay(5000);
}



