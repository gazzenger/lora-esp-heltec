/* 
  Check the new incoming messages, and print via serialin 115200 baud rate.
  
  by Aaron.Lee from HelTec AutoMation, ChengDu, China
  成都惠利特自动化科技有限公司
  www.heltec.cn
  
  this project also realess in GitHub:
  https://github.com/Heltec-Aaron-Lee/WiFi_Kit_series
*/

/*
 * Originally taken from
 * https://lastminuteengineers.com/creating-esp32-web-server-arduino-ide/
 */
#include <WiFi.h>
#include <WebServer.h>

/*
 * Based on the link
 * https://randomnerdtutorials.com/esp32-web-server-spiffs-spi-flash-file-system/
 * https://github.com/me-no-dev/ESPAsyncWebServer
 * https://github.com/me-no-dev/AsyncTCP
 */
//#include "ESPAsyncWebServer.h"

/*
 * The SPIFFS library is used for accessing the binary files uploaded to the ESP
 * https://techtutorialsx.com/2018/09/11/esp32-arduino-web-server-serving-jquery/
 */
#include "SPIFFS.h"

#include <SPI.h>
#include <LoRa.h>
#include <math.h>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;



/*
 * Files are uploaded through the "data" directory
 * The plugin is found here
 * https://github.com/me-no-dev/arduino-esp32fs-plugin
 */


/* Put your SSID & Password */
const char* ssid = "ESP32";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

//AsyncWebServer server(80);  // Object of WebServer(HTTP port, 80 is defult)
WebServer server(80);  // Object of WebServer(HTTP port, 80 is defult)

#include "HardwareSerial.h"
#include <TinyGPS.h> 
float lat = 28.5458,lon = 77.1703, elv = 0;  // create variable for latitude and longitude object 


//UART   RX IO   TX IO   CTS   RTS
//UART0   GPIO3   GPIO1   N/A   N/A
//UART1   GPIO9   GPIO10  GPIO6   GPIO11
//UART2   GPIO16  GPIO17  GPIO8   GPIO7
HardwareSerial gpsSerial(0);//rx,tx 
TinyGPS gps; // create gps object 


int counter = 0;

static void smartdelay(unsigned long ms);
static void print_date(TinyGPS &gps);
static void print_int(unsigned long val, unsigned long invalid, int len);


#define MAX_WORLD_COUNT 5
#define MIN_WORLD_COUNT 2
char *Words[MAX_WORLD_COUNT];


struct coord
{
  float lat;
  float lon;
  float elv;
};

typedef struct coord Coord;

Coord initial;
Coord final;



struct point
{
  float x;
  float y;
  float z;
  float radius;
  float nx;
  float ny;
  float nz;
};

typedef struct point Point;


#include "SSD1306.h" // alias for `#include "SSD1306Wire.h"`

//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16

 SSD1306  display(0x3c, 4, 15);


// Pin definetion of WIFI LoRa 32
#define SCK     5    // GPIO5  -- SX1278's SCK
#define MISO    19   // GPIO19 -- SX1278's MISO
#define MOSI    27   // GPIO27 -- SX1278's MOSI
#define SS      18   // GPIO18 -- SX1278's CS
#define RST     14   // GPIO14 -- SX1278's RESET
#define DI0     26   // GPIO26 -- SX1278's IRQ(Interrupt Request)

#define BAND    915E6 //you can set band here directly,e.g. 868E6,915E6
#define PABOOST true

//app defines
//resolution1000msec -> 30sec
#define TIME_OUT_RCV_PACKET  50
//#define TIME_OUT_RCV_PACKET  10

void init_diplay()
{
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH); // while OLED is running, must set GPIO16 in high
  
  display.init();

  // display.flipScreenVertically();

  display.setContrast(255);
}


void setup() {

  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

/*
 * Based on the link
 * https://techtutorialsx.com/2018/09/11/esp32-arduino-web-server-serving-jquery/
 */
  if(!SPIFFS.begin()){
       Serial.println("An Error has occurred while mounting SPIFFS");
       return;
  }

/*
 * Based on the link
 * https://tttapa.github.io/ESP8266/Chap11%20-%20SPIFFS.html
 */
  server.onNotFound([]() {                              // If the client requests any URI
    if (!handleFileRead(server.uri()))                  // send it if it exists
      server.send(404, "text/plain", "404: Not Found"); // otherwise, respond with a 404 (Not Found) error
  });

/*
 * Taken from the link
 * https://electropeak.com/learn/create-a-web-server-w-esp32/
 */
//  server.on("/", handle_root);

/*
 * Based on the link
 * https://techtutorialsx.com/2018/09/11/esp32-arduino-web-server-serving-jquery/
 */
//  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
//      request->send(SPIFFS, "/index.html", "text/html");
//  });
//  server.on("/jquery-3.6.0.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/jquery-3.6.0.min.js", "text/javascript");
//  });
//  server.on("/bootstrap.bundle.min.js", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/bootstrap.bundle.min.js", "text/javascript");
//  });
//  server.on("/bootstrap.min.css", HTTP_GET, [](AsyncWebServerRequest *request){
//    request->send(SPIFFS, "/bootstrap.min.css", "text/css");
//  });
//  server.onNotFound(handle_NotFound);

  server.begin();
  delay(100); 

  
  init_diplay();
  display.clear();
  //display.setTextSize(1);
  //display.setColor(WHITE);
  //display.setCursor(0,0);
  display.setLogBuffer(5, 30);
  display.println("LoRa Receiver");
  display.drawLogBuffer(0, 0);
  display.display();
#if 0
  // Initialize the log buffer
  // allocate memory to store 8 lines of text and 30 chars per line.
  display.setLogBuffer(5, 30);
  // Print to the screen
  display.println("LoRa Receiver:");
  // Draw it to the internal screen buffer
  display.drawLogBuffer(0, 0);
  // Display it on the screen
  display.display();
#endif
  Serial.begin(9600);
  gpsSerial.begin(9600);
  while (!Serial); //test this program,you must connect this board to a computer
  Serial.println("LoRa Receiver");
  SPI.begin(SCK,MISO,MOSI,SS);
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND )) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setSignalBandwidth(250e3);
  LoRa.setSpreadingFactor(12);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.enableCrc();

  SerialBT.begin("ESP32test"); //Bluetooth device name

  
}

int time_out =0;

void loop() {

  server.handleClient();

  // bluetooth serial read
  char ch;
  while (SerialBT.available()) {
      display.clear();
      ch=(char)SerialBT.read();
      display.print((char)ch);
  }






char lorastr[30];

byte i=0;

  int packetSize = LoRa.parsePacket();
  if (packetSize) {

    // read packet
    while (LoRa.available()) { 
      lorastr[i] = (char)LoRa.read();
      i++;
    }
    
      lorastr[i] = '\0';


        split_message(lorastr);

      Serial.println(Words[0]);
      Serial.println(Words[1]);
      Serial.println(Words[2]);

//    lcd.setCursor(0,0);


      initial.lat = (float)strtod(Words[0],NULL);
      initial.lon = (float)strtod(Words[1],NULL);
      initial.elv = (float)strtod(Words[2],NULL);

//      lcd.print("Dist: ");
//      lcd.print(Calculate()*1000,2);
//      lcd.print("m");



//     lcd.setCursor(0,1);
//     lcd.print("RSSI: ");
//     lcd.print(LoRa.packetRssi());
//     lcd.print("dbm");    


  display.clear();
  display.printf("Dist: %.2f\n",Calculate()*1000);
  display.printf("RSSI: %dr dbm\n",LoRa.packetRssi());
     
  }





  



  //GPS Stuff
  gps.f_get_position(&lat,&lon); // get latitude and longitude 
  elv = gps.f_altitude();
  Serial.print("Position: "); 
  Serial.print("Latitude:"); 
  Serial.print(lat,6); 
  Serial.print(";"); 
  Serial.print("Longitude:"); 
  Serial.println(lon,6);  
  Serial.println(elv,1);
  print_date(gps);  

  final.lat = lat;
  final.lon = lon;
  final.elv = elv;

  //display.clear();
  //display.printf("%.3f %.3f %.1f\n",lat,lon,elv);

  // final functions
  //refresh the display
  display.drawLogBuffer(0, 0);
  display.display();
  //run the smart delay
  smartdelay(1000);



  
//  // try to parse packet
//  char ch;
//  delay(1000);
//  int packetSize = LoRa.parsePacket();
//  if (packetSize) {
//    time_out =0;
//    // received a packet
//    display.clear();
//    display.print("Rcv:");
//    Serial.print("Received packet '");
//    SerialBT.print("Received packet '");
//    int hTab=0;
//    // read packet
//    while (LoRa.available()) {
//      ch=(char)LoRa.read();
//      display.print((char)ch);
//      Serial.print(ch);
//      SerialBT.print(ch);
//      hTab++;
//      //add new line <CR>
//      if (hTab>=21) //accepts 23 also .Lowered to show tes message beter
//      {
//        display.println(" ");
//        hTab=0;
//      }
//    }
//    // print RSSI of packet
//    display.print("' with RSSI ");
//    display.println(LoRa.packetRssi());
//    display.drawLogBuffer(0, 0);
//    display.display();
//    Serial.print("' with RSSI ");
//    Serial.println(LoRa.packetRssi());
//    SerialBT.print("' with RSSI ");
//    SerialBT.println(LoRa.packetRssi());
//  }
//  else
//  {
//    time_out++;
//    if (time_out> TIME_OUT_RCV_PACKET)
//    {
//      display.clear();
//      display.println("RCV_ TIMEOUT !!!");
//      display.drawLogBuffer(0, 0);
//      display.display();
//    }
//    //Serial.println("no lora packet");
//  }
}


/*
 * Taken from the link
 * https://electropeak.com/learn/create-a-web-server-w-esp32/
 */
//String HTML = "<!DOCTYPE html>\
//<html>\
//<body>\
//<h1>My First Web Server with ESP32 - Station Mode &#128522;</h1>\
//</body>\
//</html>";
//
//// Handle root url (/)
//void handle_root() {
//  server.send(200, "text/html", HTML);
//}
//
//
//void handle_NotFound(){
//  server.send(404, "text/plain", "Not found");
//}





/*
 * Based on the link
 * https://tttapa.github.io/ESP8266/Chap11%20-%20SPIFFS.html
 */
String getContentType(String filename) { // convert the file extension to the MIME type
  if (filename.endsWith(".html")) return "text/html";
  else if (filename.endsWith(".css")) return "text/css";
  else if (filename.endsWith(".js")) return "application/javascript";
  else if (filename.endsWith(".ico")) return "image/x-icon";
  return "text/plain";
}

bool handleFileRead(String path) { // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  if (path.endsWith("/")) path += "index.html";         // If a folder is requested, send the index file
  String contentType = getContentType(path);            // Get the MIME type
  if (SPIFFS.exists(path)) {                            // If the file exists
    File file = SPIFFS.open(path, "r");                 // Open it
    size_t sent = server.streamFile(file, contentType); // And send it to the client
    file.close();                                       // Then close the file again
    return true;
  }
  Serial.println("\tFile Not Found");
  return false;                                         // If the file doesn't exist, return false
}




////////// ////////// ////////// ////////// ////////// ////////// //////////
// Split string into individual words and store each word into an array
// this function return word_count (number of words)
////////// ////////// ////////// ////////// ////////// ////////// //////////
byte split_message(char* str) {
  byte word_count = 0; //number of words
  char * item = strtok (str, " ,"); //getting first word (uses space & comma as delimeter)

  while (item != NULL) {
    if (word_count >= MAX_WORLD_COUNT) {
      break;
    }
    Words[word_count] = item;
    item = strtok (NULL, " ,"); //getting subsequence word
    word_count++;
  }
  return  word_count;
}








static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}




static void print_date(TinyGPS &gps)
{
  int year;
  byte month, day, hour, minute, second, hundredths;
  unsigned long age;
  gps.crack_datetime(&year, &month, &day, &hour, &minute, &second, &hundredths, &age);
  if (age == TinyGPS::GPS_INVALID_AGE)
    Serial.print("********** ******** ");
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d %02d:%02d:%02d ",
        month, day, year, hour, minute, second);
    Serial.print(sz);
  }
  print_int(age, TinyGPS::GPS_INVALID_AGE, 5);
  smartdelay(0);
}



static void print_int(unsigned long val, unsigned long invalid, int len)
{
  char sz[32];
  if (val == invalid)
    strcpy(sz, "*******");
  else
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartdelay(0);
}






//GPS Distance calculations

    float EarthRadiusInMeters(float latitudeRadians)
    {
        // latitudeRadians is geodetic, i.e. that reported by GPS.
        // http://en.wikipedia.org/wiki/Earth_radius
        float a = 6378137.0;  // equatorial radius in meters
        float b = 6356752.3;  // polar radius in meters
        float cosrad = cos(latitudeRadians);
        float sinrad = sin(latitudeRadians);
        float t1 = a * a * cosrad;
        float t2 = b * b * sinrad;
        float t3 = a * cosrad;
        float t4 = b * sinrad;
        return sqrt((t1*t1 + t2*t2) / (t3*t3 + t4*t4));
    }

    float  GeocentricLatitude(float latitude)
    {
        // Convert geodetic latitude 'lat' to a geocentric latitude 'clat'.
        // Geodetic latitude is the latitude as given by GPS.
        // Geocentric latitude is the angle measured from center of Earth between a point and the equator.
        // https://en.wikipedia.org/wiki/Latitude#Geocentric_latitude
        float e2 = 0.00669437999014;
        float clat = atan((1.0 - e2) * tan(latitude));
        return clat;
    }

Point LocationToPoint(Coord c)
    {
        // Convert (lat, lon, elv) to (x, y, z).
        float latitude = c.lat * PI / 180.0;
        float longitude = c.lon * PI / 180.0;
        float radius = EarthRadiusInMeters(latitude);
        float clat   = GeocentricLatitude(latitude);

        float cosLon = cos(longitude);
        float sinLon = sin(longitude);
        float cosLat = cos(clat);
        float sinLat = sin(clat);
        float x = radius * cosLon * cosLat;
        float y = radius * sinLon * cosLat;
        float z = radius * sinLat;

        // We used geocentric latitude to calculate (x,y,z) on the Earth's ellipsoid.
        // Now we use geodetic latitude to calculate normal vector from the surface, to correct for elevation.
        float cosGlat = cos(latitude);
        float sinGlat = sin(latitude);

        float nx = cosGlat * cosLon;
        float ny = cosGlat * sinLon;
        float nz = sinGlat;

        x += c.elv * nx;
        y += c.elv * ny;
        z += c.elv * nz;

        Point point;
        point.x = x;
        point.y = y;
        point.z = z;
        point.radius = radius;
        point.nx = nx;
        point.ny = ny;
        point.nz = nz;

        return point;
    }

    float Distance (Point ap,Point bp)
    {
        float dx = ap.x - bp.x;
        float dy = ap.y - bp.y;
        float dz = ap.z - bp.z;
        return sqrt (dx*dx + dy*dy + dz*dz);
    }

    Point RotateGlobe (Coord b, Coord a,float bradius,float aradius)
    {
        // Get modified coordinates of 'b' by rotating the globe so that 'a' is at lat=0, lon=0.
        Coord br;
        br.lat = b.lat;
        br.lon= (b.lon - a.lon);
        br.elv=b.elv;
        
        Point brp = LocationToPoint(br);

        // Rotate brp cartesian coordinates around the z-axis by a.lon degrees,
        // then around the y-axis by a.lat degrees.
        // Though we are decreasing by a.lat degrees, as seen above the y-axis,
        // this is a positive (counterclockwise) rotation (if B's longitude is east of A's).
        // However, from this point of view the x-axis is pointing left.
        // So we will look the other way making the x-axis pointing right, the z-axis
        // pointing up, and the rotation treated as negative.

        float alat = GeocentricLatitude(-a.lat * PI / 180.0);
        float acosrad = cos(alat);
        float asinrad = sin(alat);

        float bx = (brp.x * acosrad) - (brp.z * asinrad);
        float by = brp.y;
        float bz = (brp.x * asinrad) + (brp.z * acosrad);

        Point point;
        point.x = bx;
        point.y = by;
        point.z = bz;
        point.radius = bradius;

        return point;
    }

Point NormalizeVectorDiff(Point b,Point a)
    {
      Point point;      
        // Calculate norm(b-a), where norm divides a vector by its length to produce a unit vector.
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float dz = b.z - a.z;
        float dist2 = dx*dx + dy*dy + dz*dz;
        if (dist2 == 0) {
            return point;
        }
        float dist = sqrt(dist2);
        point.x = (dx/dist);
        point.y = (dy/dist);
        point.z = (dz/dist);
        point.radius = 1.0;
        return point;
    }



    float Calculate()
    {

                Point ap = LocationToPoint(initial);
                Point bp = LocationToPoint(final);
                float distKm = 0.001 * Distance(ap,bp);
                return distKm;
    }
