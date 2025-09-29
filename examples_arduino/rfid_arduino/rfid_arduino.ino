#include <SPI.h> //INCLUSÃO DE BIBLIOTECA
#include <MFRC522.h> //INCLUSÃO DE BIBLIOTECA

#define SS_PIN 10 //PINO SDA
#define RST_PIN 9 //PINO DE RESET

/*
TAG 1 = EC:A9:77:0A
TAG 2 = AA:62:EA:87
TAG 3 = 26:59:62:AF
CARTAO = 25:0B:CF:11
*/

MFRC522 rfid(SS_PIN, RST_PIN); //PASSAGEM DE PARÂMETROS REFERENTE AOS PINOS

void setup() {
  Serial.begin(9600); //INICIALIZA A SERIAL
  SPI.begin(); //INICIALIZA O BARRAMENTO SPI
  rfid.PCD_Init(); //INICIALIZA MFRC522
}

void loop() {
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) //VERIFICA SE O CARTÃO PRESENTE NO LEITOR É DIFERENTE DO ÚLTIMO CARTÃO LIDO. CASO NÃO SEJA, FAZ
    return; //RETORNA PARA LER NOVAMENTE

  /***INICIO BLOCO DE CÓDIGO RESPONSÁVEL POR GERAR A TAG RFID LIDA***/
  String strID = "";
  for (byte i = 0; i < 4; i++) {
    strID +=
    (rfid.uid.uidByte[i] < 0x10 ? "0" : "") +
    String(rfid.uid.uidByte[i], HEX) +
    (i!=3 ? ":" : "");
  }
  strID.toUpperCase();
  /***FIM DO BLOCO DE CÓDIGO RESPONSÁVEL POR GERAR A TAG RFID LIDA***/

  Serial.print("Identificador (UID) da tag: "); //IMPRIME O TEXTO NA SERIAL
  Serial.println(strID); //IMPRIME NA SERIAL O UID DA TAG RFID

  rfid.PICC_HaltA(); //PARADA DA LEITURA DO CARTÃO
  rfid.PCD_StopCrypto1(); //PARADA DA CRIPTOGRAFIA NO PCD
}