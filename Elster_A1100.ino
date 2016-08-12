/*
  Updated to use DIG3: INT1
  
  ****************
  
  It will print to to the serial just when it detects a change
  in Total a1100_imports, Total a1100_exports or a change in direction (0=Importing , 1=Exporting)
  
  I have tried some IR sensors so far the only one working at the moment is RPM7138-R
  
  Based on Dave's code to read an elter a100c for more info on that vist:
  http://www.rotwang.co.uk/projects/meter.html
  Thanks Dave.
*/

#define BIT_PERIOD 860 // us
#define BUFF_SIZE 64
volatile long data[BUFF_SIZE];
volatile uint8_t in;
volatile uint8_t out;
volatile unsigned long last_us;
uint8_t a1100_dbug = 0;

ISR(INT1_vect) {
   unsigned long us = micros();
   unsigned long diff = us - last_us;
   if (diff > 20 ) {
      last_us = us;
      int next = in + 1;
      if (next >= BUFF_SIZE) next = 0;
      data[in] = diff;
      in = next;
   }
}


void setup() {
    in = out = 0;
    Serial.begin(115200);
    
    //http://www.protostack.com/blog/2010/09/external-interrupts-on-an-atmega168/
    EICRA |= 3;    // Attach RISING edge interrupt
    EIMSK |= 2;    // Interrupt Maks Register: DIG3 = IRQ1 . Change to 1 for Dig2 INT0
    last_us = micros();
}

unsigned int a1100_statua1100_sFlag;
float a1100_imports;
float a1100_exports;

void loop() {
  int rd = a1100_decode_buff();
  if (!rd) return;
  if (rd==3) {
   rd=4;
   Serial.println("");
   Serial.print(a1100_imports);    Serial.print("\t");
   Serial.print(a1100_exports);    Serial.print("\t");
   Serial.print(a1100_statua1100_sFlag); Serial.println("");
  }

}

float a1100_last_data;
uint8_t a1100_sFlag;
float a1100_imps;
float a1100_exps;
uint16_t idx=0;
uint8_t byt_msg = 0;
uint8_t bit_left = 0;
uint8_t bit_shft = 0;
uint8_t pSum = 0;
uint16_t BCC = 0;
uint8_t eom = 1;

static int a1100_decode_buff(void) {
   if (in == out) return 0;
   int next = out + 1;
   if (next >= BUFF_SIZE) next = 0;
   int p = (((data[out]) + (BIT_PERIOD/2)) / BIT_PERIOD);
   if (a1100_dbug) { Serial.print(data[out]); Serial.print(" "); if (p>500) Serial.println("<-"); }
   if (p>500) {
     idx = BCC = eom = a1100_imps = a1100_exps = a1100_sFlag = 0;
     out = next;
     return 0;
   }
   bit_left = (4 - (pSum % 5));
   bit_shft = (bit_left<p)? bit_left : p;
   pSum = (pSum==10)? p : ((pSum+p>10)? 10: pSum + p);
   if (eom==2 && pSum>=7) {
      pSum=pSum==7?11:10;
      eom=0;
   }

   if (bit_shft>0) {
      byt_msg >>= bit_shft;
      if (p==2) byt_msg += 0x40<<(p-bit_shft);
      if (p==3) byt_msg += 0x60<<(p-bit_shft);
      if (p==4) byt_msg += 0x70<<(p-bit_shft);
      if (p>=5) byt_msg += 0xF0;
    }
//    Serial.print(p); Serial.print(" ");Serial.print(pSum);Serial.print(" ");
//    Serial.print(bit_left);Serial.print(" ");Serial.print(bit_shft);Serial.print(" ");
//    Serial.println(byt_msg,BIN);
    if (pSum >= 10) {
       idx++;
       if (idx!=328) BCC=(BCC+byt_msg)&255;
//       if (a1100_dbug){Serial.print("[");Serial.print(idx);Serial.print(":");Serial.print(byt_msg,HEX); Serial.print("]");}
       if (idx>=95 && idx<=101)
          a1100_imps += ((float)byt_msg-48) * pow(10 , (101 - idx));
       if (idx==103)
          a1100_imps += ((float)byt_msg-48) / 10;
       if (idx>=114 && idx<=120)
          a1100_exps += ((float)byt_msg-48) * pow(10 , (120-idx));
       if (idx==122)
          a1100_exps += ((float)byt_msg-48) / 10;
       if (idx==210)
          a1100_sFlag = (byt_msg-48)>>3; //1=Exporting ; 0=Importing
       if (byt_msg == 3) eom=2;
       if (idx==328) {
          if ((byt_msg>>(pSum==10?(((~BCC)&0b1000000)?0:1):2))==((~BCC)&0x7F)) {
             if (a1100_last_data != (a1100_imps + a1100_exps + a1100_sFlag)) {
                a1100_imports=a1100_imps;
                a1100_exports=a1100_exps;
                a1100_statua1100_sFlag=a1100_sFlag;
                a1100_last_data = a1100_imps + a1100_exps + a1100_sFlag;
                out = next;
                return 3;
             }
          }
          if (a1100_dbug) {
             Serial.println(""); Serial.print("---->>>>");
             Serial.print(a1100_imps); Serial.print("\t");
             Serial.print(a1100_exps); Serial.print("\t");
             Serial.print(a1100_sFlag); Serial.print("\t");
             Serial.print(pSum); Serial.print("\t");
             Serial.print(byt_msg>>(pSum==10?1:2),BIN); Serial.print("\t"); //BCC read
             Serial.print((~BCC)&0x7F,BIN); Serial.print("\t"); //BCC calculated

          }
       }
    }
    out = next;
    return 0;
}
