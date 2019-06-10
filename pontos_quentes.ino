/*
 PROJETO PONTOS QUENTES

 MCU: ARDUINO NANO

 AUTOR: ISAAC GONDIM GERALDO      DATA:2018
 REFERENCIA:http://www.artilhariadigital.com/2014/11/Medir-temperatura-com-sensor-infravermelho-MLX90614-e-Arduino.html
 */

//BIBLIOTECAS
#include <SoftwareSerial.h>
#include <i2cmaster.h>

//MAPEMENTO DE HARDWARE 

#define TX_PIN 10 //Porta do Arduino onde o pino RX do Módulo GSM esta conectado
 

#define RX_PIN 11 //Porta do Arduino onde o pino TX do Módulo GSM esta conectado
 

SoftwareSerial serialGSM(RX_PIN, TX_PIN); //Cria comunicacao serial via software nas portas digitais definidas acima

//DECLARAAÇÃO DE VARIAVEIS GLOBAIS
unsigned long temp;   
int x=1;
int* xp=&x;
float celcius;
float* temperatura= &celcius;
int c=0;
int* cont=&c;

//JOHNSON
char inData[20]; // Allocate some space for the string
char inChar; // Where to store the character read
byte index = 0; // Index into array; where to store the character
String umamsg;

// DECLARAÇÃO DE FUNÇÕES AUXILIARES
void Desligar();
void Ligar();
void tempmlx();
void Config();
void Data();
void SeriPot();

//=======================================================================================================================================================================================================

void setup() 
{
  
  Serial.begin(9600); //Begin serial comunication with Arduino and Arduino IDE (Serial Monitor)
  while(!Serial);
   
  
  serialGSM.listen(); //Inicia comunicacao serial com o GSM
  serialGSM.begin(9600);
  delay(1000);
  
  i2c_init(); //Inicializa a comunicação I2C
  PORTC = (1 << PORTC4) | (1 << PORTC5);// Habilita os pullups



  temp=millis();// define que a variavel  igual ao tempo usado pelo sistema at executar essa linha

  Serial.println("Setup Completo!");

  //Config();
}

//=======================================================================================================================================================================================================

void loop() 
{
  //SeriPot(); //executa a função SeriPot para testes pela serial
  
  //if ((millis() - temp) > 209980)//inicia se tiver passado o tempo para o envio da mensagem(5mim contando com a executação do codigo)
  if ((millis() - temp) > 10000)//inicia se tiver passado o tempo para o envio da mensagem(10mim contando com a executação do codigo)
  {                                                                                                                                                                                                 
    Ligar();//liga o modulo
    //Serial.println("porta serial fechada");//indica que a porta serial no pode mais sr utilizada pelo usuario e que começar a enviar a mensagem
    delay(2000);
    tempmlx();
    delay(1000);
    Data(); //realiza o envio dos dados
    delay(1000);
    Serial.println("Feito"); //indica que a mensagem deve ter sido enviada com sucesso e que o usuario pode mandar comandos para o modulo
    temp = millis();//define temp para o horario que termina de executar o envio da mensagem
    Desligar();//desliga o modulo
  }
}

//=======================================================================================================================================================================================================

void SeriPot() //função para comunicação serial
{
    if(Serial.available())
    {
        while(index < 1)   
        {
          inChar = Serial.read(); // Read a character
          //Serial.println(inChar);
          inData[index] = inChar; // Store it
          index++; // Increment where to write next
          //inData[index] = '\0'; // Null terminate the string
        } // One less than the size of the array
        index=0;
        serialGSM.write(inChar);
      
        
        /*while(inData[index] != '\0')
        {
          Serial.println(inData[index++]);
        }*/

    }
    
   
    
    if(serialGSM.available())
    {
       //serialGSM.write(Serial.read());

       
        while(index < 1)   
        {
          inChar = serialGSM.read(); // Read a character
          //Serial.println(inChar);
          inData[index] = inChar; // Store it
          index++; // Increment where to write next
          //inData[index] = '\0'; // Null terminate the string
        } // One less than the size of the array
        index=0;        
        Serial.write(inChar);

        //umamsg[0]="0";
        umamsg=String(umamsg+inChar);
        //Serial.print("<"+umamsg+">" );
        if(umamsg == "AT" or umamsg.substring(3)== "OK" or umamsg.substring(2)== "AT")
        {
          Serial.print("ENTENDIDO");
          umamsg="";
        }
      
        
        /*while(inData[index] != '\0')
        {
          Serial.println(inData[index++]);
        }*/

    }
    /*     
    if(serialGSM.available())     //Le as mensagens vindas do GSM para escrevê-las no monitor serial
    {
      char c = serialGSM.read();     
      Serial.write(c);
    }    
     */
}    

//=======================================================================================================================================================================================================

void Data()
{   Serial.println(*temperatura);
    delay(500);
    serialGSM.write("AT+CIPMUX=0\r\n");//define uma conexão com apenas um servidor
    delay(3000);
    serialGSM.write("AT+CIPSHUT\r\n");//Deactivate GPRS PDP Context
    delay(5000);
    //serialGSM.write("AT+CIICR=0\r\n");//Bring Up Wireless Connection with GPRS or CSD
    //delay(3000);
    serialGSM.write("AT+CGREG=2\r\n");//faz uma conexão em uma rede GPRS
    delay(3000);
    serialGSM.write("AT+CIPSTART=TCP,54.94.245.185,3000\r\n");//inicia uma conexão tcp com o seguinte servidor
    delay(60000);
    serialGSM.write("AT+CIPSEND\r\n");//inicia a mensagem
    delay(3000);
    char texto[20];//define o tamanho do corpo da mensagem
    int primeiro= (int)*temperatura;//primeira variavel para a temperatura(antes da virgula)
    int segundo= (*temperatura-primeiro)*100;//segunda variavel para a temperatura(depois da virgula)
    
    //sprintf(texto,"CEA001%.3d%.3d%d",primeiro,segundo,*cont);//controle da variavel na porta serial
    sprintf(texto,"CEA001%.3d%.3d",primeiro,segundo);//controle da variavel na porta serial
    
    delay(3000);
    Serial,serialGSM.write(texto);//envia texto(o corpo da mensagem) para o servidor 
    delay(3000);
    Serial,serialGSM.write(char(26));//comando para encerrar a mensagem
    //serialGSM.write("AT+CIPCLOSE\r\n");
    //c++;
}

//=======================================================================================================================================================================================================

void Config()
{   Serial.println(*temperatura);
    delay(500);
    serialGSM.write("AT+CIPMUX=0\r\n");//define uma conexão com apenas um servidor
    delay(3000);
    serialGSM.write("AT+CIPSHUT\r\n");//Deactivate GPRS PDP Context
    delay(5000);
    //serialGSM.write("AT+CIICR=0\r\n");//Bring Up Wireless Connection with GPRS or CSD
    //delay(3000);
    serialGSM.write("AT+CGREG=2\r\n");//faz uma conexão em uma rede GPRS
    delay(3000);
    serialGSM.write("AT+CIPSTART=TCP,0000.0000.0000.0000,3000\r\n");//inicia uma conexão tcp com o seguinte servidor <substituir pelo id do servidor utilizado>
    delay(60000);
}

//=======================================================================================================================================================================================================

void tempmlx()
{
  int dev = 0x5A<<1;
  int data_low = 0;
  int data_high = 0;
  int pec = 0;
  
  i2c_start_wait(dev+I2C_WRITE);
  i2c_write(0x07);
  
  i2c_rep_start(dev+I2C_READ);
  data_low = i2c_readAck(); //Faz a leitura de 1 byte e depois envia ack
  data_high = i2c_readAck(); //Faz a leitura de 1 byte e depois envia ack
  pec = i2c_readNak();
  i2c_stop();
  
  //Converte os bytes high e low juntos e processa a temperatura, MSB é um bit de erro que é ignorado para temperaturas
  double tempFactor = 0.02; // 0.02 graus por LSB (resolução do MLX90614)
  double tempData = 0x0000; // Zera os dados 
  int frac; // dados apos o ponto decimal
  
  // Mascara o bit de erro do high byte, e depois move para esquerda 8 bits e soma o low byte.
  tempData = (double)(((data_high & 0x007F) << 8) + data_low);
  tempData = (tempData * tempFactor)-0.01;
  
  celcius = tempData - 273.15;

  delay(1000); // Espera 1 segundo antes de continuar o código. 
}

//=======================================================================================================================================================================================================

void Ligar()
{
  //digitalWrite(2,HIGH);//liga usando pino digital e chave de transistor
  Serial,serialGSM.write("AT+CFUN=1");//liga usando comando at para deixar em funcionalidade total
  delay(18000);  
}
//=======================================================================================================================================================================================================

void Desligar()
{
  //digitalWrite(2,LOW);//desliga usando pino digital e chave de transistor
  Serial,serialGSM.write("AT+CFUN=0");//deixa em funcionalidade minima com o comando at
  delay(1000);
}

//pedir horio pra rede/AT+CCLK
//https://forum.arduino.cc/index.php?topic=152683.0
