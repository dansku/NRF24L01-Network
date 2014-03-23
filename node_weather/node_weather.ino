#include <DHT.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <stdarg.h>

DHT dht(2, DHT22);

RF24 radio(9,10); // CE, CS. CE at pin A0, CSN at pin 8
RF24Network network(radio);

static uint16_t this_node = 3; // 001
short node_prime = 79; // 83, 89, 97
unsigned long iterations=0;
unsigned long errors=0;
unsigned int loss=0;
unsigned long p_sent=0;
unsigned long p_recv=0;
// Variables for the 32bit unsigned long Microsecond rollover handling
static unsigned long microRollovers=0; // variable that permanently holds the number of rollovers since startup
static unsigned long halfwayMicros = 2147483647; // this is halfway to the max unsigned long value of 4294967296
static boolean readyToRoll = false; // tracks whether we've made it halfway to rollover

const short max_active_nodes = 10;
uint16_t active_nodes[max_active_nodes];
short num_active_nodes = 0;
short next_ping_node_index = 0;
const unsigned long interval = 5000;
unsigned long last_time_sent;
unsigned long updates = 0;
void add_node(uint16_t node);
boolean send_T(uint16_t to);
void handle_T(RF24NetworkHeader& header);
void handle_B(RF24NetworkHeader& header);
void p(char *fmt, ... );

void setup(void){
  dht.begin();
  Serial.begin(115200);
  delay(128);
  SPI.begin();
  radio.begin();
  //Slower, better distance
   radio.setDataRate(RF24_250KBPS);
  // The amplifier gain can be set to RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  radio.setPALevel(RF24_PA_LOW); // transmitter gain value (see above)
  network.begin(/*fixed radio channel: */ 16, /*node address: */ this_node );
  Serial.print("This_node [DEC:");Serial.print(this_node,DEC);Serial.print("/OCT:");Serial.print(this_node,OCT);Serial.println("]");
  p("%010ld: Starting up\n", millis());
  
  pinMode(4,OUTPUT);
}



void loop(void){
  network.update();
  updates++;
  
  while ( network.available() ){
    
    RF24NetworkHeader header;
    network.peek(header); // preview the header, but don't advance nor flush the packet
    
    handle_B(header);
  
    unsigned long time;
    network.read(header,&time,sizeof(time));
 }
  
  unsigned long now = millis();
  unsigned long nowM = micros();
  if ( now - last_time_sent >= interval ) // non-blocking
  {
    updates = 0;
    last_time_sent = now;
    uint16_t to = 00;
    bool ok = 0;
    if ( to != this_node)
    {
      unsigned long nowM = micros();
      ok = send_T(to);
      p(" in %ld us.\n", (micros()-nowM) );
      if (ok){
        p_sent++;
      }
      if (!ok)
      {
        errors++;
        //last_time_sent -= node_prime; // random awesomeness to stop packets from colliding (at least it tries to)
        p("%010ld: Timout while sending. Can I haz bugfix? \n", millis()); // An error occured, need to stahp!
      }
      iterations++;
      Serial.print("loop ");
      Serial.println(iterations);
      Serial.print("errors: ");
      Serial.println(errors);     
      Serial.print("send error in %: ");
      Serial.println(errors*100/iterations);
      Serial.print("packet sent    : ");
      Serial.println(p_sent);
      Serial.print("packet received: ");
      Serial.println(p_recv);
      Serial.print("reply in %  : ");
      Serial.println(p_recv*100/(p_sent-1));
    }
  }   
}
/*
 * T send own time
 * B send back the just received time
 */
boolean send_T(uint16_t to) // Send out this nodes' time -> Timesync!
{
   RF24NetworkHeader header(to,'OK');
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  header.temperature = t;
  header.humidity = h;
  
  p("%010ld: Sent Temperature Humidity  %05o", millis(),to);
 
  unsigned long time = micros();
  return network.write(header,&time,sizeof(time));
}

// Arduino version of the printf()-funcition in C 
void p(char *fmt, ... ){
  char tmp[128]; // resulting string limited to 128 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, 128, fmt, args);
  va_end (args);
  Serial.print(tmp);
}

unsigned long microsRollover() { //based on Rob Faludi's (rob.faludi.com) milli wrapper

  // This would work even if the function were only run once every 35 minutes, though typically,
  // the function should be called as frequently as possible to capture the actual moment of rollover.
  // The rollover counter is good for over 584000 years of runtime. 
  //  --Alex Shure
  
  unsigned long nowMicros = micros(); // the time right now

  if (nowMicros > halfwayMicros) { // as long as the value is greater than halfway to the max
    readyToRoll = true; // we are in the second half of the current period and ready to roll over
  }

  if (readyToRoll == true && nowMicros < halfwayMicros) {
    // if we've previously made it to halfway
    // and the current millis() value is now _less_ than the halfway mark
    // then we have rolled over
    microRollovers++; // add one to the count of rollovers (approx 71 minutes)
    readyToRoll = false; // we're no longer past halfway, reset!
  } 
  return microRollovers;
}

void handle_B(RF24NetworkHeader& header){
  p_recv++;
  unsigned long ref_time;
  network.read(header,&ref_time,sizeof(ref_time));
  //p("%010ld: Received 'B' from %05o -> %ldus round trip\n", millis(), header.from_node, micros()-ref_time);
  Serial.print("Recebi ");
  Serial.println(header.type);
  if(header.type == 79){ digitalWrite(4,HIGH); }
  else{digitalWrite(4,LOW);}
}