//#include "caracteres.h"
#include "fonts/TinyFont.h"
//#include "fonts/small_font.h"
#include "fonts/Sinclair_S.h"
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
#define SDELAY 600
//#define T IT_2
#define DELAY_INFO 100
#define DELAY_BANNER 3000
#define TXT_DEBUG 0
#define CaracteresArray CaracteresArray2

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

void Grafica_Car_Mat(const uint8_t Caract[], int inicio,uint32_t Mat[]){
  
    if(inicio==0){
      for (int j=0;j<8;j++){
        for (int i=0;i<Mat_Alt;i++){
          Mat[i]=(uint32_t)(((Mat[i] << 1))| (uint32_t)(Caract[i]>>(8-j)));
          
      }
      Grafica_Mat(Mat,32,2,0);
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
                    delayMicroseconds(SDELAY);
                  digitalWrite(PIN_RED_CLK, 0);
                  digitalWrite(PIN_GRN_CLK, 0);
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
  //nvs_flash_init();
    
  //setDHTPin(PIN_DHT11);
  //setPinsVeml6070(SDA_PIN,SCL_PIN);
  
  //sprintf(buf,"PINS %llu\n",GPIO_OUTPUT_PINS);
  //Serial.print (buf);
  //gpio_config_t io_conf;
  //io_conf.intr_type = GPIO_PIN_INTR_DISABLE;  //disable interrupt
  //io_conf.mode = GPIO_MODE_OUTPUT; //set as output mode
  //io_conf.pin_bit_mask = GPIO_OUTPUT_PINS; //bit mask of the pins that you want to set,e.g.GPIO18/19
  //io_conf.pull_down_en = 0; //disable pull-down mode
  //io_conf.pull_up_en = 0; //disable pull-up mode
  //gpio_config(&io_conf);  //configure GPIO with the given settings
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
        
  //digitalWrite(PIN_BLK, 0);
  //      digitalWrite(PIN_RED_CLK, 0);
  //      digitalWrite(PIN_GRN_CLK, 0);
  //     
  //i2c_master_init();
  
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
  
}
 
  void loop(){
    //val=getTemp();
    char texto[]="Laboratorio de Electronica y Tecnologia del CIOR";
        //printf("Temperatura %d\n",val);
    //int itemp = (int)roundf(val);
          //printf("iTemp %d\n",itemp);
    int k=0;
    
    Blanc_Mat(Matriz);
    int letra=0;
    
     //   Pone_Car_Mat(TinyFont[10],k,Matriz);
    //k=k+8;
    for(int i=0;i<(sizeof(texto)-1);i++){
    //  letra=(int)texto[i];
//    while (itemp > 0)
//    { 
      //    int digit = itemp%10;
  //        itemp /= 10;
   //   Pone_Car_Mat(TinyFont[letra],k,Matriz);
     letra=(int)texto[i];
     sprintf(buf,"Letra: %c int: %d \n",texto[i],letra);
     Serial.print (buf);
     //Pone_Car_Mat(TinyFont[letra-32],k,Matriz);
     Grafica_Car_Mat(TinyFont[letra-32], 0,Matriz);
     //Agrega_Car_Mat(small_font[letra-32], 0,Matriz);
     //Agrega_Car_Mat(Sinclair_S[letra-32], 0,Matriz);
     //Agrega_Car_Mat(Sinclair_Inverted_S[letra-32], 0,Matriz);
     
     k=k+8;
//          printf("Digit : %d  Posicion : %d", digit,k);
   // }
    if(TXT_DEBUG>0)
       Imprime_Mat(Matriz);
    //Grafica_Mat(Matriz,32,2,0);
    //      if(DELAY_INFO>0)
    //  delay(DELAY_INFO / portTICK_RATE_MS);
    }
    if(DELAY_BANNER>0)
          delay(DELAY_BANNER / portTICK_RATE_MS);
//    val=getHumidity();
//        printf("Humedad %d\n",val);
//    itemp = (int)roundf(val);
//          printf("iHumed %d\n",itemp);
//    k=0;
//    Blanc_Mat(Matriz);
//    Pone_Car_Mat(CaracteresArray[11],k,Matriz);
//    k=k+8;
//    while (itemp > 0)
//    { 
 //         int digit = itemp%10;
//          itemp /= 10;
//      Pone_Car_Mat(CaracteresArray[digit],k,Matriz);
//      k=k+8;
//          printf("Digit : %d  Posicion : %d", digit,k);
//    }
//    if(TXT_DEBUG>0)
//       Imprime_Mat(Matriz);
//    Grafica_Mat(Matriz,32,2,1);
 //         if(DELAY_INFO>0)
//      delay(DELAY_INFO / portTICK_RATE_MS);
    
    if (uv!=65534){
      uvc= (int)roundf(uv*CORR_VALUV/100);  
      //uvi=i2c_veml6070_indexuv(uvc,T);
      AgregaVal_UV(uvi);
      prom=Promedio_UV();
      l= (int)roundf(prom*11/11);
      sprintf(buf,"UV: %d UVC: %d  UVI: %d IBAR: %d\n",uv,uvc,uvi,l);
      Serial.print (buf);
      if(l>10){
        leds=32;
      }else if(l>7){
        leds=27;
      }else if(l>5){
        leds=22;
      }else if(l>2){
        leds=17;
      }else if(l>1){
        leds=12;
      }else if(l>0){
        leds=6;
      }else if(l==0){
        leds=1;
      }
    
    }else{
      sprintf(buf,"Error de lectura uv: %d \n",uv);
      Serial.print (buf);
      leds=0;
    }
      GenBarra_Mat(Matriz,leds);
      if(TXT_DEBUG>0)
              Imprime_Mat(Matriz);
      if(leds>12){
        col=2;
      }
      if(leds>22){
        col=0;
      }
            Grafica_Mat(Matriz,32,col,1);
          if(DELAY_INFO>0)
      delay(DELAY_INFO / portTICK_RATE_MS);
    Blanc_Mat(Matriz);
  //  Grafica_Banner();
}
