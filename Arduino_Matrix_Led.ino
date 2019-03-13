#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <QueueList.h>
//#include "caracteres.h"
#include "fonts/TinyFont.h"
//#include "fonts/small_font.h"
//#include "fonts/Sinclair_S.h"
//#include "fonts/Sinclair_Inverted_S.h"

#define CANT_VALUV 20
#define CORR_VALUV 110
#define PIN1_RED1 GPIO_NUM_32
#define PIN1_RED2 GPIO_NUM_33
#define PIN2_RED1 GPIO_NUM_15
#define PIN2_RED2 GPIO_NUM_23
#define PIN3_RED1 GPIO_NUM_27
#define PIN3_RED2 GPIO_NUM_14
#define PIN4_RED1 GPIO_NUM_12
#define PIN4_RED2 GPIO_NUM_13
#define PIN_BLK GPIO_NUM_5
#define PIN_RED_STR GPIO_NUM_18
#define PIN_RED_CLK GPIO_NUM_19
#define PIN_GRN_STR GPIO_NUM_17
#define PIN_GRN_CLK GPIO_NUM_16
#define PIN_DHT11 GPIO_NUM_21
#define SDA_PIN GPIO_NUM_22
#define SCL_PIN GPIO_NUM_23
#define Mat_Alt 8
#define Mat_Larg 32
#define SDELAY 170
//#define T IT_2
#define DELAY_INFO 100
#define DELAY_BANNER 3000
#define TXT_DEBUG 0
#define CaracteresArray CaracteresArray2
#define MQTT_SERVER "10.110.254.121"  //you MQTT IP Address
#define MQTT_PORT 1883  //you MQTT IP Address
#define MQTT_USER "ha"  //you MQTT IP Address
#define MQTT_PASS "4ut0m4t1c0"  //you MQTT IP Address

const char* ssid = "10110";
const char* password = "T3cn0C10r";
static int taskCore = 0;
int analog_value = 0;
static int Wconectado = 0;
String TopicBase = "cior";
String TopicDev = "dev-display-01";
String Tdisplay1 = "display/lab/don1";
String Tdisplay2 = "display/lab/dsp1";
String txt="";
uint8_t  txtcolor=0;
uint16_t txttiempo=0;
uint8_t  txtpermanente=0;
uint8_t  txtidperm=0;
uint8_t  txtindex=0;

WiFiClient espClient;
PubSubClient client(espClient);
TaskHandle_t tempTaskHandle = NULL;

char buf[100];
int col=1;
int l=0;
//int l = rand() % 32;
uint8_t dat=((2<<2) | 0x02);
uint16_t uv=0;
uint16_t uvc=0;
uint8_t uvi=0;
uint8_t prom=0;
uint8_t leds=0;

typedef struct strMSG {
  String  texto;
  uint8_t  color;
  uint16_t tiempo;
  uint8_t  permanente;
  uint8_t  idperm;
} MSG;

MSG tabla[6]={String("Laboratorio de Electronica y Tecnologia del CIOR"), 2,3000,1,0 };

QueueList <MSG> queue;
MSG MsgTXT;
uint32_t Matriz[]={
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};

uint32_t MatrizROSARIO[]={
0b00000000000000000000000000000000,
0b11100011000110001100111001001100,
0b10010100101000010010100101010010,
0b11100100100110011110111001010010,
0b10100100100001010010101001010010,
0b10010011000110010010100101001100,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
uint32_t MatrizSMART[]={
0b00000000000000000000000000000000,
0b00000110011011001100111001110000,
0b01101000010101010010100100100000,
0b00000110010001011110111000100000,
0b01100001010001010010101000100000,
0b00000110010001010010100100100000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
uint32_t MatrizCIOR[]={
0b00000000000000000000000000000000,
0b00000000110010011000111000000000,
0b00111001001010100100100100111000,
0b00000001000010100100111000000000,
0b00111001001010100100101000111000,
0b00000000110010011000100100000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};

void WiFiEvent(WiFiEvent_t event)
{
    Serial.printf("[WiFi-event] event: %d\n", event);

    switch (event)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      Wconectado=1;
      ArduinoOTA.begin(); // OTA initialization
      delay(400);
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection.  Attempting to reconnect...");
      //WiFi.reconnect();
      Wconectado=0;
      break;
    case SYSTEM_EVENT_STA_START:
      Serial.println("ESP32 station start");
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.println("ESP32 station connected to AP");
      break;
    default:      
      Serial.println("Unhandled WiFi Event raised.");
      break;
    }
}
void Wifi_init(){ 
  WiFi.disconnect(true);
  delay(1000);
  WiFi.onEvent(WiFiEvent);
  WiFi.begin(ssid, password);
}

int suscribetopics_mqtt(){
  int rsus,rsus1;
  String Topic1=TopicBase + "/" + Tdisplay1;
  rsus=client.subscribe(string2char(Topic1));
  Serial.print( "[SUBSCRIBE] Topic1" );
  Serial.println(Topic1);
  Serial.println(rsus);
  Topic1=TopicBase + "/" + Tdisplay2;
  rsus1=client.subscribe(string2char(Topic1));
  Serial.print( "[SUBSCRIBE] Topic2" );
  Serial.println(Topic1);
  Serial.println(rsus1);
  return (rsus && rsus1);
}

char* string2char(String command){
    if(command.length()!=0){
        char *p = const_cast<char*>(command.c_str());
        return p;
    }
}

void on_message(char* topic, byte* payload, unsigned int length) {
  Serial.println("Mensaje Recibido");
  String topicStr = String(topic);
  Serial.print("Topic: ");
  Serial.println(topicStr);
  Serial.print("Payload: ");
  Serial.println((char)payload[0]);
  Serial.print("length: ");
  Serial.println(length);
  char cpayload[length + 1];
  strncpy (cpayload, (char*)payload, length);
  cpayload[length] = '\0';
  String msg=cpayload;
  String Topic1=TopicBase + "/" + Tdisplay1;
  String Topic1C=TopicBase + "/" + Tdisplay1 + "c";
  
  if (topicStr == Topic1){
    if((cpayload[0] == '1') || (cpayload[0] == '0')){
       Serial.print("Topic1 Payload : ");
       Serial.println(msg.toInt());
       //digitalWrite(RLY_P1, msg.toInt());
       pubicatopic_mqtt(Topic1C, msg);
    }
  }else{
    Serial.println("Topic1 No coincide : ");
    Serial.println(topicStr);
    Serial.println(Topic1);
  }
  
  StaticJsonBuffer<400> jsonBuffer;
  String Topic2=TopicBase + "/" + Tdisplay2;
  String Topic2C=TopicBase + "/" + Tdisplay2 + "c";
  
  if (topicStr == Topic2){

    JsonObject& root=jsonBuffer.parseObject((char*)cpayload);
    if (!root.success()) {
      Serial.println("parseObject() failed");
      return;
    }
    Serial.println("Cpayload:");
    root.printTo(Serial);
    Serial.println("----");
    MSG ResMsg;
    if(root.containsKey("texto") && root.containsKey("color") && root.containsKey("tiempo") && root.containsKey("permanente") && root.containsKey("idperm")){
      ResMsg.texto=(const char*)root["texto"];
      ResMsg.color=root["color"];
      ResMsg.tiempo=root["tiempo"];
      ResMsg.permanente=root["permanente"];
      ResMsg.idperm=root["idperm"];
      if(ResMsg.permanente==0){
         queue.push (ResMsg);
      }else{
        if(ResMsg.idperm<6){
          if(ResMsg.tiempo==0)
             ResMsg.permanente=0;
          tabla[ResMsg.idperm]=ResMsg;
        }
     }
    }
  }else{
    Serial.println("Topic2 No coincide : ");
    Serial.println(topicStr);
    Serial.println(Topic2);
  }
}

int pubicatopic_mqtt(String topic, String msg){
  int rsus;
  rsus=client.publish(string2char(topic), msg.c_str());
  Serial.print( "Publish : ");
  Serial.println(rsus);
  Serial.print( "Topic : ");
  Serial.println(topic);
  Serial.print( "Valor :");
  Serial.println(msg);
  return rsus;
}

void coreTask( void * pvParameters ){
 
    String taskMessage = "Corriendo en core ";
    taskMessage = taskMessage + xPortGetCoreID();
    Serial.println(taskMessage);
    int rsus;
    
    while(true){
      if(Wconectado == 1){
          if(!client.connected()) {
            Serial.print("Conectando ThingsBoard node ...");
            if ( client.connect((char*) TopicDev.c_str(), MQTT_USER, MQTT_PASS)) {
              Serial.println( "[DONE]" );
              suscribetopics_mqtt();
            } else {
              Serial.print( "[FAILED] [ rc = " );
              Serial.print( client.state() );
            }
          }
          //Serial.print("LOOP ");
          if(! client.loop()){
            Serial.println("Error en Loop.");  
          }
          //Serial.println(taskMessage);
      }else{
      Serial.print("No conectado wifi:");
      Serial.println(Wconectado);
    }
   delay(300);
   ArduinoOTA.handle();
   }
}

uint16_t ValoresUV[CANT_VALUV]={0};
static uint8_t ivaloresuv=0;
static uint8_t bvaloresuv=0;
//static uint8_t temprature_sens_read(); 
//static uint32_t hall_sens_read();
//void TempPres(double sali[]);

uint8_t Promedio_UV(void){
  int num=0;
  uint32_t sum=0;
  uint8_t prom=0;
  uint8_t vals=ivaloresuv;
  if (bvaloresuv==1)
     vals=CANT_VALUV;
  for (int v=0;v<vals;v++){
     num++;
     sum=sum+ValoresUV[v];
  }
  if ( num >0 ){
    prom=sum/num;
    sprintf(buf,"Promedio: %d Suma: %d Numero: %d\n",prom,sum,num);
    Serial.print (buf);
  }else{
    sprintf(buf,"Sin Promedio: no hay valores. Suma: %d Numero: %d\n",sum,num);
    Serial.print (buf);
  }
  return prom;
}

void AgregaVal_UV(uint8_t valor){
      ValoresUV[ivaloresuv]=valor;
      if(ivaloresuv<(CANT_VALUV-1)){
       ivaloresuv++;
      }else{
       ivaloresuv=0;
       bvaloresuv=1;
      }
    sprintf(buf,"Agrega Val: %d ival: %d\n",valor,ivaloresuv);
    Serial.print (buf);
}
void binary(uint32_t numero){
  for (int i = 31; i >= 0; i--){
    sprintf(buf,"%u",((numero >> i) & 1));
    Serial.print (buf);
  }
  Serial.print("\n");
  
}
void GenBarra_Mat(uint32_t Mat[],int indice){
  uint32_t val=0;
  for (int j=0;j<indice;j++){
    val=val | (1<<j);
  }
  if (TXT_DEBUG >0) 
    binary(val);
  for (int i=0;i<Mat_Alt;i++){
    Mat[i]=val;
  }
}
void Blanc_Mat(uint32_t Mat[]){
  for (int i=0;i<Mat_Alt;i++){
    Mat[i]=0b00000000000000000000000000000000;
  }
}
void Pone_Car_Mat(const uint8_t Caract[], int posicion,uint32_t Mat[]){
  for (int i=0;i<Mat_Alt;i++){
    Mat[i]=Mat[i] | (uint32_t)(Caract[i] << posicion);    
  }
}

void Agrega_Car_Mat(const uint8_t Caract[], int inicio,uint32_t Mat[]){
  for (int i=0;i<Mat_Alt;i++){
    if(inicio==0){
      Mat[i]=(Mat[i] << (sizeof(Caract[i])*8))| (uint32_t)(Caract[i]);
    }else{
      Mat[i]=(Mat[i] >> (sizeof(Caract[i])*8))| (uint32_t)(Caract[i]<< (Mat_Larg-(sizeof(Caract[i])*8)));    
    }
  }
}

void Grafica_Car_Mat(const uint8_t Caract[], int inicio,uint32_t Mat[],uint8_t color){
    if(inicio==0){
      for (int j=0;j<8;j++){
        for (int i=0;i<Mat_Alt;i++){
          Mat[i]=(uint32_t)(((Mat[i] << 1))| (uint32_t)(Caract[i]>>(8-j)));
        }
        Grafica_Mat(Mat,32,color,1);
        if(SDELAY>0)
          delayMicroseconds(SDELAY);
     }
   }
}
      
void Imprime_Mat(uint32_t Mat[]){
  Serial.print("Matriz :\n");
  for (int i=0;i<Mat_Alt;i++){
    binary(Mat[i]);
  }
}
void Grafica_Mat(uint32_t Matriz[],int largo,int color,int borra){
  int val;
  for (int i=0;i<largo;i++){
    val=(Matriz[0]>> i) & 1;
    digitalWrite(PIN1_RED1, val);
    val=(Matriz[1]>> i) & 1;
                digitalWrite(PIN1_RED2, val);
    val=(Matriz[2]>> i) & 1;
                digitalWrite(PIN2_RED1, val);
    val=(Matriz[3]>> i) & 1;
                digitalWrite(PIN2_RED2, val);
    val=(Matriz[4]>> i) & 1;
                digitalWrite(PIN3_RED1, val);
    val=(Matriz[5]>> i) & 1;
                digitalWrite(PIN3_RED2, val);
    val=(Matriz[6]>> i) & 1;
                digitalWrite(PIN4_RED1, val);
    val=(Matriz[7]>> i) & 1;
                digitalWrite(PIN4_RED2, val);
    if(SDELAY>0)
       delayMicroseconds(SDELAY);
    switch (color){
    case 0:
      digitalWrite(PIN_RED_CLK, 1);
      if(TXT_DEBUG>0)
                     //Serial.print("Clock RED.\n");
                     Serial.print(".");
      if(SDELAY>0)
          delayMicroseconds(SDELAY);
      digitalWrite(PIN_RED_CLK, 0);
      if(SDELAY>0)
          delayMicroseconds(SDELAY);
      if(borra==1){
          digitalWrite(PIN1_RED1, 0);
          digitalWrite(PIN1_RED2, 0);
          digitalWrite(PIN2_RED1, 0);
          digitalWrite(PIN2_RED2, 0);
          digitalWrite(PIN3_RED1, 0);
          digitalWrite(PIN3_RED2, 0);
          digitalWrite(PIN4_RED1, 0);
          digitalWrite(PIN4_RED2, 0);
          if(SDELAY>0)
             delayMicroseconds(SDELAY);
          digitalWrite(PIN_GRN_CLK, 1);
          if(TXT_DEBUG>0)
              //Serial.print("Clock GREEN BLANQUEO.\n");
            Serial.print("-");
          if(SDELAY>0)
            delayMicroseconds(SDELAY);
          digitalWrite(PIN_GRN_CLK, 0);
          if(SDELAY>0)
            delayMicroseconds(SDELAY);
      }
      break;
    case 1:
      digitalWrite(PIN_GRN_CLK, 1);
      if(TXT_DEBUG>0)
        //Serial.print("Clock GREEN.\n");
        Serial.print(".");
       if(SDELAY>0)
         delayMicroseconds(SDELAY);
       digitalWrite(PIN_GRN_CLK, 0);
       if(SDELAY>0)
        delayMicroseconds(SDELAY);
      if(borra==1){
          digitalWrite(PIN1_RED1, 0);
          digitalWrite(PIN1_RED2, 0);
          digitalWrite(PIN2_RED1, 0);
          digitalWrite(PIN2_RED2, 0);
          digitalWrite(PIN3_RED1, 0);
          digitalWrite(PIN3_RED2, 0);
          digitalWrite(PIN4_RED1, 0);
          digitalWrite(PIN4_RED2, 0);
          if(SDELAY>0)
            delayMicroseconds(SDELAY);
          digitalWrite(PIN_RED_CLK, 1);
          if(TXT_DEBUG>0)
              //Serial.print("Clock RED BLANQUEO.\n");
              Serial.print("-");
          if(SDELAY>0)
             delayMicroseconds(SDELAY);
          digitalWrite(PIN_RED_CLK, 0);
          if(SDELAY>0)
            delayMicroseconds(SDELAY);
      }
      break;
    case 2:
      digitalWrite(PIN_RED_CLK, 1);
      digitalWrite(PIN_GRN_CLK, 1);
      if(TXT_DEBUG>0)
         //Serial.print("Clock RED-GREEN.\n");
         Serial.print(".");
      if(SDELAY>0)
        delayMicroseconds(SDELAY*2);
      digitalWrite(PIN_RED_CLK, 0);
      digitalWrite(PIN_GRN_CLK, 0);
      if(SDELAY>0)
        delayMicroseconds(SDELAY*3);
      break;
    default:
      digitalWrite(PIN_RED_CLK, 1);
      if(TXT_DEBUG>0)
          //Serial.print("Clock RED.\n");
          Serial.print(".");
      if(SDELAY>0)
         delayMicroseconds(SDELAY);
      digitalWrite(PIN_RED_CLK, 0);
    }   
      
      
  }
  switch (color){
    case 0:
      if(SDELAY>0)
         delayMicroseconds(SDELAY);
      digitalWrite(PIN_RED_STR, 1);
      if(TXT_DEBUG>0)
          Serial.print("STROB RED.\n");
      if(SDELAY>0)
          delayMicroseconds(SDELAY);
      digitalWrite(PIN_RED_STR, 0);
      if(borra==1){
          if(SDELAY>0)
             delayMicroseconds(SDELAY);
          digitalWrite(PIN_GRN_STR, 1);
          if(TXT_DEBUG>0)
             Serial.print("STROB GREEN BORRA.\n");
          if(SDELAY>0)
             delayMicroseconds(SDELAY);
          digitalWrite(PIN_GRN_STR, 0);
      }
      break;
    case 1:
        if(SDELAY>0)
            delayMicroseconds(SDELAY);
        digitalWrite(PIN_GRN_STR, 1);
        if(TXT_DEBUG>0)
          Serial.print("STROB GREEN.\n");
        if(SDELAY>0)
          delayMicroseconds(SDELAY);
        digitalWrite(PIN_GRN_STR, 0);
        if(borra==1){
           if(SDELAY>0)
              delayMicroseconds(SDELAY);
           digitalWrite(PIN_RED_STR, 1);
           if(TXT_DEBUG>0)
              Serial.print("STROB RED BORRA.\n");
           if(SDELAY>0)
              delayMicroseconds(SDELAY);
           digitalWrite(PIN_RED_STR, 0);
      }
      break;
    case 2:
      if(SDELAY>0)
           delayMicroseconds(SDELAY);
      digitalWrite(PIN_RED_STR, 1);
      digitalWrite(PIN_GRN_STR, 1);
      if(TXT_DEBUG>0)
            Serial.print("STROB RED-GREEN.\n");
      if(SDELAY>0)
         delayMicroseconds(SDELAY);
      digitalWrite(PIN_RED_STR, 0);
      digitalWrite(PIN_GRN_STR, 0);
      break;
    default:
     if(SDELAY>0)
        delayMicroseconds(SDELAY);
     digitalWrite(PIN_RED_STR, 1);
     if(TXT_DEBUG>0)
        Serial.print("STROB RED.\n");
     if(SDELAY>0)
       delayMicroseconds(SDELAY);
     digitalWrite(PIN_RED_STR, 0);
        }
  if(TXT_DEBUG>0)
    Serial.print("BLK .\n");
  digitalWrite(PIN_BLK, 0);
}
    
void Grafica_Matriz_Desplaza(uint32_t Matriz[],int largo,int color,int borra,int direccion){
  
uint32_t MatrizTemp[]={
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
  for(int i=0;i<=largo;i++){
    if(direccion==0){
    MatrizTemp[0]=(uint32_t)((Matriz[0]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[1]=(uint32_t)((Matriz[1]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[2]=(uint32_t)((Matriz[2]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[3]=(uint32_t)((Matriz[3]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[4]=(uint32_t)((Matriz[4]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[5]=(uint32_t)((Matriz[5]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[6]=(uint32_t)((Matriz[6]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[7]=(uint32_t)((Matriz[7]<< (largo - i)) & 0b11111111111111111111111111111111);
    }else{
    MatrizTemp[0]=(uint32_t)((Matriz[0]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[1]=(uint32_t)((Matriz[1]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[2]=(uint32_t)((Matriz[2]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[3]=(uint32_t)((Matriz[3]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[4]=(uint32_t)((Matriz[4]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[5]=(uint32_t)((Matriz[5]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[6]=(uint32_t)((Matriz[6]>> (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[7]=(uint32_t)((Matriz[7]<< (largo - i)) & 0b11111111111111111111111111111111);
    }
    if(TXT_DEBUG>0)
      Imprime_Mat(MatrizTemp);
    Grafica_Mat(MatrizTemp,largo,color,borra);
  }
}

void Grafica_Matriz_DesplazayFunde(uint32_t Matriz[],uint32_t Matrizf[],int largo,int color,int borra){
  
uint32_t MatrizTemp[]={
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
  for(int i=1;i<=largo;i++){
    MatrizTemp[0]=(uint32_t)((Matriz[0]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[1]=(uint32_t)((Matriz[1]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[2]=(uint32_t)((Matriz[2]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[3]=(uint32_t)((Matriz[3]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[4]=(uint32_t)((Matriz[4]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[5]=(uint32_t)((Matriz[5]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[6]=(uint32_t)((Matriz[6]<< (largo - i)) & 0b11111111111111111111111111111111);
    MatrizTemp[7]=(uint32_t)((Matriz[7]<< (largo - i)) & 0b11111111111111111111111111111111);
    if(TXT_DEBUG>0)
      Imprime_Mat(MatrizTemp);
    Grafica_Mat(MatrizTemp,largo,color,borra);
  }
        if(DELAY_BANNER>0)
          delay(DELAY_BANNER / portTICK_RATE_MS);
  for(int i=1;i<largo;i++){
    MatrizTemp[0]=(uint32_t)(((Matriz[0]<< i) | (Matrizf[0]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[1]=(uint32_t)(((Matriz[1]<< i) | (Matrizf[1]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[2]=(uint32_t)(((Matriz[2]<< i) | (Matrizf[2]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[3]=(uint32_t)(((Matriz[3]<< i) | (Matrizf[3]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[4]=(uint32_t)(((Matriz[4]<< i) | (Matrizf[4]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[5]=(uint32_t)(((Matriz[5]<< i) | (Matrizf[5]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[6]=(uint32_t)(((Matriz[6]<< i) | (Matrizf[6]>> (largo - i))) & 0b11111111111111111111111111111111);
    MatrizTemp[7]=(uint32_t)(((Matriz[7]<< i) | (Matrizf[7]>> (largo - i))) & 0b11111111111111111111111111111111);
    if(TXT_DEBUG>0)
      Imprime_Mat(MatrizTemp);
    Grafica_Mat(MatrizTemp,largo,color,borra);
  }
        if(DELAY_BANNER>0)
          delay(DELAY_BANNER / portTICK_RATE_MS);
}
void Grafica_Banner(void){
  //Blanc_Mat(Matriz);
  //Grafica_Mat(Matriz,32,2,0);
  Grafica_Matriz_DesplazayFunde(MatrizROSARIO,MatrizSMART,32,2,0);
  Blanc_Mat(Matriz);
  Grafica_Mat(Matriz,32,2,0);
}


void setup() {
 Serial.begin(115200);
  
 pinMode(PIN1_RED1, OUTPUT);
 pinMode(PIN1_RED2, OUTPUT);
 pinMode(PIN2_RED1, OUTPUT);
 pinMode(PIN2_RED2, OUTPUT);
 pinMode(PIN3_RED1, OUTPUT);
 pinMode(PIN3_RED2, OUTPUT);
 pinMode(PIN4_RED1, OUTPUT);
 pinMode(PIN4_RED2, OUTPUT);
 pinMode(PIN_BLK, OUTPUT);
 pinMode(PIN_RED_STR, OUTPUT);
 pinMode(PIN_RED_CLK, OUTPUT);
 pinMode(PIN_GRN_STR, OUTPUT);
 pinMode(PIN_GRN_CLK, OUTPUT);
 
 ArduinoOTA.setHostname((const char*) TopicDev.c_str()); // A name given to your ESP8266 module when discovering it as a port in ARDUINO IDE
 delay(400);
 queue.setPrinter (Serial);

 client.setServer(MQTT_SERVER, MQTT_PORT);
 client.setCallback(on_message);
 xTaskCreatePinnedToCore(coreTask, "coreTask", 10000, NULL, 0, NULL, taskCore);      
 delay(400);
 
 if(TXT_DEBUG>0)
    Serial.print("BLK 1.\n");
        delay(500 / portTICK_PERIOD_MS);
 if(TXT_DEBUG>0)
          Serial.print("BLK 0.\n");
        delay(200 / portTICK_PERIOD_MS);
  int val=0;
  Blanc_Mat(Matriz);
  Grafica_Mat(Matriz,32,2,0);
  Grafica_Mat(MatrizCIOR,32,2,0);
  if(DELAY_BANNER>0)
     delay(DELAY_BANNER / portTICK_RATE_MS);
  Grafica_Banner();
  Blanc_Mat(Matriz);
  txtindex=0;
  
}
 
void loop(){
    //val=getTemp();
    if (Wconectado == 0){
      Serial.println("Error No conectado wifi Wifi_init.");
      Wifi_init();
    }
    if(!queue.isEmpty ()){
        Serial.println("COLA# No esta vacia. Saca de la cola..");
        MsgTXT=queue.pop();
        
    }else{
        Serial.println("COLA# Esta vacia. Saca de Estatico..");
        Serial.print("TXTIndex:");
        Serial.println(txtindex);
        while(txtindex<6){
            MsgTXT=tabla[txtindex];
            txtindex++;
            Serial.print("While - tiempo: ");
            Serial.println(MsgTXT.tiempo);
            if(MsgTXT.tiempo>0){
               break;    
            }
                
       }
       if(txtindex>5)
          txtindex=0;
    }
    txt=MsgTXT.texto;
    txtcolor=MsgTXT.color;
    txttiempo=MsgTXT.tiempo;
    txtpermanente=MsgTXT.permanente;
    txtidperm=MsgTXT.idperm;
     Serial.print("TXT: ");
     Serial.print(txt);
     Serial.print("   COLOR: ");
     Serial.print(txtcolor);
     Serial.print("   TIEMPO: ");
     Serial.print(txttiempo);
     Serial.print("   PERM: ");
     Serial.print(txtpermanente);
     Serial.print("   IDP: ");
     Serial.println(txtidperm);
    
    int k=0;
    Blanc_Mat(Matriz);
    int letra=0;
    if(txttiempo>0){
      Serial.print("SIZEOF(TXT): ");
      Serial.println(sizeof(txt)-1);
      Serial.print("length(TXT): ");
      Serial.println(txt.length());
      for(int i=0;i<(txt.length());i++){
          letra=(int)txt[i];
          sprintf(buf,"Letra: %c int: %d \n",txt[i],letra);
          Serial.print (buf);
          //Pone_Car_Mat(TinyFont[letra-32],k,Matriz);
          Grafica_Car_Mat(TinyFont[letra-32], 0,Matriz,txtcolor);
          //Agrega_Car_Mat(small_font[letra-32], 0,Matriz);
          //Agrega_Car_Mat(Sinclair_S[letra-32], 0,Matriz);
          //Agrega_Car_Mat(Sinclair_Inverted_S[letra-32], 0,Matriz);
          k=k+8;
          if(TXT_DEBUG>0)
            Imprime_Mat(Matriz);
      }
      if(txttiempo>0)
            delay(txttiempo / portTICK_RATE_MS);
      if(TXT_DEBUG>0)
            Imprime_Mat(Matriz);
      Blanc_Mat(Matriz);
    }
 }
