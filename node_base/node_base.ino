
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>
#include <stdarg.h>

RF24 radio(9,10); // CE, CS. CE at pin A0, CSN at pin 8
RF24Network network(radio);

static uint16_t this_node = 0; // 001
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

void setup(void)
{
  Serial.begin(115200);
  delay(128);
  SPI.begin();
  radio.begin();
  // The amplifier gain can be set to RF24_PA_MIN=-18dBm, RF24_PA_LOW=-12dBm, RF24_PA_MED=-6dBM, and RF24_PA_HIGH=0dBm.
  radio.setPALevel(RF24_PA_MIN); // transmitter gain value (see above)
  radio.setDataRate(RF24_250KBPS);
  network.begin(/*fixed radio channel: */ 16, /*node address: */ this_node );
  p("%010ld: Starting up... We are Node %05d \n", millis(),this_node);
}

void loop(void)
{
  network.update();
  updates++;
  while ( network.available() ) // while there is some shit filling our pipe
  {
    RF24NetworkHeader header;
    network.peek(header); // preview the header, but don't advance nor flush the packet
    Serial.print("Kind: ");Serial.println(header.module);
    //Messages Received
    if(header.module == 1){
      p("%010ld: ID: %05d - From %05o - To: %05o",millis(),header.id,header.from_node, header.to_node);
      Serial.print(" - Temperature: ");Serial.print(header.temperature);
      Serial.print(" - Humidity: ");Serial.println(header.humidity);
    }
    else {
    p("%010ld: ID: %05d - From %05o - To: %05o\n",millis(),header.id,header.from_node, header.to_node);
    }
    
    //Handle Receibed and Answer Them
    handle_T(header);
    
//    if(digitalRead(2)==HIGH){
//      handle_DO(header,3,"O");
//    }
//   

    };
  
  
  unsigned long now = millis();
  unsigned long nowM = micros();
  if ( now - last_time_sent >= interval ) // non-blocking
  {
    p("%010ld: %ld estimated updates/s\n",millis(),updates*1000/interval);
    updates = 0;
    last_time_sent = now;
    uint16_t to = 00;
    bool ok = 0;
  }   
}

void handle_DO(RF24NetworkHeader& header, int to_node, char *){
  unsigned long time;
    RF24NetworkHeader header2(to_node, 'M'); //Here we can send O (On) F (Off)
    if(network.write(header2,&time,sizeof(time)))
      p("%010ld: Sending MSG to\n", millis(),header.from_node);

}

void handle_T(RF24NetworkHeader& header){
  unsigned long time;
  network.read(header,&time,sizeof(time));
  //p("%010ld: Recv 'T' from %05o:%010ld\n", millis(), header.from_node, time);
  add_node(header.from_node);  
  if(header.from_node != this_node){
    if(header.module == 1){
      if(digitalRead(2)==HIGH){
        RF24NetworkHeader header2(header.from_node/*header.from_node*/,'O');
        if(network.write(header2,&time,sizeof(time)))
         p("%010ld: Answering 'B' to %05o, confirming message received TURN LIGHT ON!\n", millis(),header.from_node);
      }
      else{
        RF24NetworkHeader header2(header.from_node/*header.from_node*/,'F');
        if(network.write(header2,&time,sizeof(time)))
        p("%010ld: Answering 'B' to %05o, confirming message received!TURN LIGHT OFF \n", millis(),header.from_node);
      }

    }
    else{
      RF24NetworkHeader header2(header.from_node/*header.from_node*/,'B');
      if(network.write(header2,&time,sizeof(time)))
      p("%010ld: Answering 'B' to %05o, confirming message received!\n", millis(),header.from_node);
    }
  }
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

void add_node(uint16_t node)
{
  short i = num_active_nodes;
  while (i--)
    if ( active_nodes[i] == node ) break; // Do we already know about this node?
  if ( i == -1 && num_active_nodes < max_active_nodes )  // If not and there is enough place, add it to the table
  {
    active_nodes[num_active_nodes++] = node; 
    p("%010ld: Add new node: %05o\n", millis(), node);
  }
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
