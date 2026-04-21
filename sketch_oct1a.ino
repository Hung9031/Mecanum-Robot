#include <WiFi.h>
#include <PubSubClient.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

// Left Front Motor
#define M1_PWM 26
#define M1_IN1 33
#define M1_IN2 25

// Right Front Motor
#define M2_PWM 19
#define M2_IN1 32
#define M2_IN2 23
 
// Right Rear Motor
#define M3_PWM 27
#define M3_IN1 12
#define M3_IN2 14
 
// Left Rear Motor
#define M4_PWM 4
#define M4_IN1 0//13
#define M4_IN2 15//2
 
//Time stick
unsigned long previousTime = 0;
unsigned long previousTime_Encoder = 0;
unsigned long previousTime_Motor = 0;
//const unsigned long interval_Motor = 100;
const unsigned long interval_Encoder = 50; // 10 ms
unsigned long motorStartTime = 0;  // thời điểm bắt đầu chạy motor
bool motorRunning = false;         // trạng thái đang chạy hay không
unsigned long currentTime;
unsigned long lastReconnectAttempt = 0; // Lưu thời điểm thử kết nối gần nhất
const unsigned long RECONNECT_INTERVAL = 5000; // Thử lại sau mỗi 5s
unsigned long lastMsgTime = 0; // Lưu thời điểm nhận lệnh cuối cùng (cho Safety)
const unsigned long SAFETY_TIMEOUT = 2000; // Xe tự dừng nếu mất sóng quá 2s

// Define some preset motor speeds (0 - 255, adjust as desired)
int speed_slow = 100;
int speed_fast = 255;
float setpoint1 = 0, setpoint2 = 0, setpoint3 = 0, setpoint4 = 0;

// timedelay_1s
int timeDelay = 1000;

//PWM Frequency 1Khz
const int PWMFreq = 15000;
//PWM Resolution
const int PWMResolution = 8;

//Define PWM channels for each motor
const int M1pwmchannel = 0;
const int M2pwmchannel = 1;
const int M3pwmchannel = 2;
const int M4pwmchannel = 3;

//Define PWM Motor Speed Variables
static int M1_Speed = 0;
static int M2_Speed = 0;
static int M3_Speed = 0;
static int M4_Speed = 0;

//Define BYTE for Motor Direction
//B7 = M1_IN1, B6 = M1_IN2, B5 = M2_IN1, B4 = M2_IN2, B3 = M3_IN1, B2 = M3_IN2, B1 = M4_IN1, B0 = M4_IN2  
//STOP
const byte STOP  = B00000000;
//Đi thẳng/lùi
const byte STRAIGHT_FORWARD  = B10101010;
const byte STRAIGHT_BACKWARD = B01010101;
//Đi ngang trái/phải
const byte SIDEWAYS_RIGHT = B10011001;
const byte SIDEWAYS_LEFT = B01100110;
//Đi theo góc 45, 135, 225, 315
const byte DIAGONAL_45  = B10001000;
const byte DIAGONAL_135 = B00100010;
const byte DIAGONAL_225 = B01000100;
const byte DIAGONAL_315 = B00010001;
//Trục đứng tiến/lùi + rẽ phải; tiến/lùi + rẽ trái
const byte PIVOT_RIGHT_FORWARD  = B10000010;
const byte PIVOT_RIGHT_BACKWARD = B01000001;
const byte PIVOT_LEFT_FORWARD   = B00101000;
const byte PIVOT_LEFT_BACKWARD  = B00010100;
//Xoay tròn theo chiều kim đồng hồ và ngược chiều kim đồng hồ
const byte ROTATE_CLOCKWISE        = B10010110;
const byte ROTATE_COUNTERCLOCKWISE = B01101001;
//Trục ngang tiến/lùi + rẽ phải; tiến/lùi + rẽ trái
const byte PIVOT_SIDEWAYS_FRONT_RIGHT = B10010000;
const byte PIVOT_SIDEWAYS_REAR_RIGHT  = B00001001;
const byte PIVOT_SIDEWAYS_FRONT_LEFT  = B01100000;
const byte PIVOT_SIDEWAYS_REAR_LEFT   = B00000110;

byte STATION = STOP;

void setup_motor() { 
  // Set up Serial Monitor 
  Serial.begin(115200); 

  // Set all connections as outputs 
  pinMode(M1_PWM, OUTPUT); pinMode(M1_IN1, OUTPUT); 
  pinMode(M1_IN2, OUTPUT); pinMode(M2_PWM, OUTPUT); 
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT); 
  pinMode(M3_PWM, OUTPUT); pinMode(M3_IN1, OUTPUT); 
  pinMode(M3_IN2, OUTPUT); pinMode(M4_PWM, OUTPUT); 
  pinMode(M4_IN1, OUTPUT); pinMode(M4_IN2, OUTPUT); 

  //Set up PWM channels with frequency and resolution 
  ledcSetup(M1pwmchannel, PWMFreq, PWMResolution); 
  ledcSetup(M2pwmchannel, PWMFreq, PWMResolution); 
  ledcSetup(M3pwmchannel, PWMFreq, PWMResolution); 
  ledcSetup(M4pwmchannel, PWMFreq, PWMResolution); 
  
  // Attach channels to PWM output pins 
  ledcAttachPin(M1_PWM, M1pwmchannel); 
  ledcAttachPin(M2_PWM, M2pwmchannel); 
  ledcAttachPin(M3_PWM, M3pwmchannel); 
  ledcAttachPin(M4_PWM, M4pwmchannel); 

  // Test speed for all motors (change as desired) 
  M1_Speed = 255; 
  M2_Speed = 255; 
  M3_Speed = 255; 
  M4_Speed = 255; 
}

void moveMotor(int speed_M1, int speed_M2, int speed_M3, int speed_M4, byte dircontrol){
  //M1
  digitalWrite(M1_IN1, bitRead(dircontrol, 7));
  digitalWrite(M1_IN2, bitRead(dircontrol, 6));
  ledcWrite(M1pwmchannel, abs(speed_M1));

  //M2
  digitalWrite(M2_IN1, bitRead(dircontrol, 5));
  digitalWrite(M2_IN2, bitRead(dircontrol, 4));
  ledcWrite(M2pwmchannel, abs(speed_M2));
 
  //M3
  digitalWrite(M3_IN1, bitRead(dircontrol, 3));
  digitalWrite(M3_IN2, bitRead(dircontrol, 2));
  ledcWrite(M3pwmchannel, abs(speed_M3));
 
  //M4
  digitalWrite(M4_IN1, bitRead(dircontrol, 1));
  digitalWrite(M4_IN2, bitRead(dircontrol, 0));
  ledcWrite(M4pwmchannel, abs(speed_M4));

  motorStartTime = millis(); // ghi lại lúc bắt đầu chạy
  motorRunning = true;
}

void stopMotors() {
  // Stops all motors and motor controllers
  ledcWrite(M1pwmchannel, 0);
  ledcWrite(M2pwmchannel, 0);
  ledcWrite(M3pwmchannel, 0);
  ledcWrite(M4pwmchannel, 0);
 
  digitalWrite(M1_IN1, 0);
  digitalWrite(M1_IN2, 0);
  digitalWrite(M2_IN1, 0);
  digitalWrite(M2_IN2, 0);
  digitalWrite(M3_IN1, 0);
  digitalWrite(M3_IN2, 0);
  digitalWrite(M4_IN1, 0);
  digitalWrite(M4_IN2, 0);

  STATION = B00000000;
  motorRunning = false;
}


//MQTT
const char* ssid = "PIFLab_M5";//"ESP32_MQTT";
const char* password = "khonghoisaocopass";//"12345678";
const char* mqtt_server = "192.168.1.159";
String mqttCommand = "";


//IP
IPAddress local_IP(192, 168, 1, 50);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   // optional
IPAddress secondaryDNS(8, 8, 4, 4); // optional


WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE	(50)
char msg[MSG_BUFFER_SIZE];
int value = 0;
String message = "";

void setup_wifi() {

  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
  mqttCommand = "";
  for (int i = 0; i < length; i++) mqttCommand += (char)payload[i];
  
  Serial.print("Message arrived ["); 
  Serial.print(topic); Serial.print("] "); 
  Serial.println(mqttCommand);
}
/*
void handleCommand(String cmd) {
  cmd.trim();

  // ===== 1. Nếu là lệnh tốc độ: "50 50 50 50"
  int spaceCount = 0;
  for (int i = 0; i < cmd.length(); i++) {
    if (cmd[i] == ' ') spaceCount++;
  }

  if (spaceCount == 3) {
    float s1, s2, s3, s4;
    sscanf(cmd.c_str(), "%f %f %f %f", &s1, &s2, &s3, &s4);

    setpoint1 = s1;
    setpoint2 = s2;
    setpoint3 = s3;
    setpoint4 = s4;

    Serial.printf("Set speed: %.1f %.1f %.1f %.1f\n",
                  setpoint1, setpoint2, setpoint3, setpoint4);
    return;   // 👈 chỉ lưu tốc độ, KHÔNG chạy xe
  }

  // ===== 2. Nếu là lệnh điều hướng: w a s d p
  if (cmd == "w") {
    STATION = STRAIGHT_FORWARD;
  } else if (cmd == "s") {
    STATION = STRAIGHT_BACKWARD;
  } else if (cmd == "a") {
    STATION = SIDEWAYS_LEFT;
  } else if (cmd == "d") {
    STATION = SIDEWAYS_RIGHT;
  } else if (cmd == "r") {
    STATION = ROTATE_CLOCKWISE;
  } else if (cmd == "p") {
    STATION = STOP;
    setpoint1 = 0;
    setpoint2 = 0;
    setpoint3 = 0;
    setpoint4 = 0;
    return;
  } else {
    Serial.println("Unknown command");
    return;
  }

  // ===== 3. Có hướng thì chạy với setpoint đã lưu
  moveMotor(M1_Speed, M2_Speed, M3_Speed, M4_Speed, STATION);
}
*/

void handleCommand(String cmd) {
  // Chon trạng thái cho xe
  String cmd_motor = cmd.substring(0, 2);
  if (cmd_motor == "w") {
    Serial.println("Straight Forward");
    STATION = STRAIGHT_FORWARD;
    setpoint1 = 50;  // rpm mong muốn
    setpoint2 = 50;
    setpoint3 = 50;
    setpoint4 = 50;
  } else if (cmd_motor == "s") {
    Serial.println("Straight Backward");  
    STATION = STRAIGHT_BACKWARD;
    setpoint1 = 50;  // rpm mong muốn
    setpoint2 = 50;
    setpoint3 = 50;
    setpoint4 = 50;
  } else if (cmd_motor == "a") {
    Serial.println("Sideways Left");  
    STATION = SIDEWAYS_LEFT;
    setpoint1 = 60;  // rpm mong muốn
    setpoint2 = 60;
    setpoint3 = 50;
    setpoint4 = 50;
  } else if (cmd_motor == "d") {
    Serial.println("Sideways Right");  
    STATION = SIDEWAYS_RIGHT;
    setpoint1 = 52;  // rpm mong muốn
    setpoint2 = 52;
    setpoint3 = 50;
    setpoint4 = 50;
  } else if (cmd == "sq") {
    Serial.println("DIAGONAL_135");  
    STATION = DIAGONAL_135;
    setpoint1 = 0;  // rpm mong muốn
    setpoint2 = 75;
    setpoint3 = 0;
    setpoint4 = 65;
  } else if (cmd == "se") {
    Serial.println("DIAGONAL_45");  
    STATION =DIAGONAL_45;
    setpoint1 = 75;  // rpm mong muốn
    setpoint2 = 0;
    setpoint3 = 65;
    setpoint4 = 0;
  } else if (cmd == "wa") {
    Serial.println("DIAGONAL_225");  
    STATION = DIAGONAL_225;
    setpoint1 = 75;  // rpm mong muốn
    setpoint2 = 0;
    setpoint3 = 65;
    setpoint4 = 0;
  } else if (cmd == "wd") {
    Serial.println("DIAGONAL_315");  
    STATION = DIAGONAL_315;
    setpoint1 = 0;  // rpm mong muốn
    setpoint2 = 75;
    setpoint3 = 0;
    setpoint4 = 65;
  } else if (cmd == "de") {
    Serial.println("PIVOT_RIGHT_FORWARD");  
    STATION = PIVOT_RIGHT_FORWARD;
    setpoint1 = 60;  // rpm mong muốn
    setpoint2 = 0;
    setpoint3 = 0;
    setpoint4 = 50;
  } else if (cmd == "ed") {
    Serial.println("PIVOT_RIGHT_BACKWARD");  
    STATION = PIVOT_RIGHT_BACKWARD;
    setpoint1 = 60;  // rpm mong muốn
    setpoint2 = 0;
    setpoint3 = 0;
    setpoint4 = 50;
  } else if (cmd == "aq") {
    Serial.println("PIVOT_LEFT_FORWARD");  
    STATION = PIVOT_LEFT_FORWARD;
    setpoint1 = 0;  // rpm mong muốn
    setpoint2 = 60;
    setpoint3 = 50;
    setpoint4 = 0;
  } else if (cmd == "qa") {
    Serial.println("PIVOT_LEFT_BACKWARD");  
    STATION = PIVOT_LEFT_BACKWARD;
    setpoint1 = 0;  // rpm mong muốn
    setpoint2 = 60;
    setpoint3 = 50;
    setpoint4 = 0;
  } else if (cmd == "r") {
    Serial.println("ROTATE_CLOCKWISE");  
    STATION = ROTATE_CLOCKWISE;
    setpoint1 = 60;  // rpm mong muốn
    setpoint2 = 60;
    setpoint3 = 50;
    setpoint4 = 50;
  } else if (cmd == "rr") {
    Serial.println("ROTATE_COUNTERCLOCKWISE");  
    STATION = ROTATE_COUNTERCLOCKWISE;
    setpoint1 = 60;  // rpm mong muốn
    setpoint2 = 60;
    setpoint3 = 50;
    setpoint4 = 50;
  } else if (cmd == "we") {
    Serial.println("PIVOT_SIDEWAYS_FRONT_RIGHT");  
    STATION = ROTATE_CLOCKWISE;
    setpoint1 = 50;  // rpm mong muốn
    setpoint2 = 50;
    setpoint3 = 0;
    setpoint4 = 0;
  } else if (cmd == "wq") {
    Serial.println("PIVOT_SIDEWAYS_REAR_RIGHT");  
    STATION = PIVOT_SIDEWAYS_REAR_RIGHT;
    setpoint1 = 0;  // rpm mong muốn
    setpoint2 = 0;
    setpoint3 = 40;
    setpoint4 = 40;
  } else if (cmd == "xc") {
    Serial.println("PIVOT_SIDEWAYS_FRONT_LEFT");  
    STATION = PIVOT_SIDEWAYS_FRONT_LEFT;
    setpoint1 = 50;  // rpm mong muốn
    setpoint2 = 50;
    setpoint3 = 0;
    setpoint4 = 0;
  } else if (cmd == "xz") {
    Serial.println("PIVOT_SIDEWAYS_REAR_LEFT");  
    STATION = PIVOT_SIDEWAYS_REAR_LEFT;
    setpoint1 = 0;  // rpm mong muốn
    setpoint2 = 0;
    setpoint3 = 40;
    setpoint4 = 40;
  } else if (cmd == "p") {
    Serial.println("STOP");  
    STATION = STOP;
    setpoint1 = setpoint2 = setpoint3 = setpoint4 = 0;
  }

  moveMotor(M1_Speed, M2_Speed, M3_Speed, M4_Speed, STATION);
}

boolean reconnect() {
  Serial.print("Attempting MQTT connection...");
  // Tạo random ID để tránh trùng
  String clientId = "ESP32Client-";
  clientId += String(random(0xffff), HEX);
  
  // Thử kết nối
  if (client.connect(clientId.c_str())) {
    Serial.println("connected");
    // Publish và Subscribe lại
    client.publish("Kival/Output", "Reconnected!");
    client.subscribe("Kival/Input");
    return true;
  } else {
    Serial.print("failed, rc=");
    Serial.print(client.state());
    Serial.println(" (will try again later)");
    return false;
  }
}
/*
void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("Kival/Output", "Hello everyone! My name is Kival :)");
      
      // ... and resubscribe
      client.subscribe("Kival/Input");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}
*/


//OTA
const char* host = "esp32";

WebServer server(80);


/*
 * Login page
 */

const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";


//Wifi AP
//const char* ssid = "ESP32-Wifi-AP-Mode";
//const char* password = "12345678";
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup_MQTT() {
  //MQTT
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void setup_OTA() {
  //OTA
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
}
/*
void setup_Wifi_AP() {
  //Wifi AP
  Serial.print("Setting AP mode");
  WiFi.softAP(ssid, password);
 
  IPAddress IP = WiFi.softAPIP(); //mặc định là 192.168.4.1
  Serial.print("AP IP address: ");
  Serial.println(IP);
  //khởi tạo webserver
  webServer.begin();
}
*/

void setup_PC13() {
  //PC13 test
  pinMode(13, OUTPUT);
}
  



/*
void loop_Wifi_AP() {
  WiFiClient webClient = webServer.available();
 
  if(webClient)
  {
    //khoi tao gia tri ban dau cho time
    currentTime = millis();
    previousTime = currentTime;
    Serial.println("New web Client");
    //biến lưu giá trị response
    String currentLine = "";
    //nếu có client connect và không quá thời gian time out
    while(webClient.connected() &amp;&amp; currentTime - previousTime <= timeoutTime)
    {
      //đọc giá trị timer tại thời điểm hiện tại
      currentTime = millis();
      //nếu client còn kết nối
      if(webClient.available())
      {
        //đọc giá trị truyền từ client theo từng byte kiểu char
        char c = webClient.read();
        Serial.write(c);
        header += c; // lưu giá trị vào Header
        if(c == '\n') //Nếu đọc được kí tự xuống dòng (hết chuỗi truyền tới)
        {
          if (currentLine.length() == 0) 
          {
            // HTTP headers luôn luôn bắt đầu với code HTTP (ví d HTTP/1.1 200 OK)
            webClient.println("HTTP/1.1 200 OK");
            webClient.println("Content-type:text/html"); // sau đó là kiểu nội dụng mà client gửi tới, ví dụ này là html
            webClient.println("Connection: close"); // kiểu kết nối ở đây là close. Nghĩa là không giữ kết nối sau khi nhận bản tin
            webClient.println();
 
            // nếu trong file header có giá trị
            if (header.indexOf("GET /led1/on") >;= 0) 
            {
              Serial.println("Led1 on");
              led1Status = "on";
              digitalWrite(led1, HIGH);
            } 
            else if (header.indexOf("GET /led1/off") >;= 0) 
            {
              Serial.println("Led1 off");
              led1Status = "off";
              digitalWrite(led1, LOW);
            } 
            else if (header.indexOf("GET /led2/on") >;= 0) 
            {
              Serial.println("Led2 on");
              led2Status = "on";
              digitalWrite(led2, HIGH);
            } 
            else if (header.indexOf("GET /led2/off") >;= 0) 
            {
              Serial.println("Led2 off");
              led2Status = "off";
              digitalWrite(led2, LOW);
            }
            // Response trang HTML 
            webClient.println("<!DOCTYPE html>;<html>;");
            webClient.println("<head>;<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">;");
            //thêm font-awesome 
            webClient.println("<link rel=\"stylesheet\" href=\"https://cdnjs.cloudflare.com/ajax/libs/font-awesome/4.7.0/css/font-awesome.min.css\">;");
            // code CSS cho web
            //css cho toan bo trang
            webClient.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            //css cho nut nhan
            webClient.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            webClient.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            webClient.println(".button2 {background-color: #5e0101;}</style></head>");
             
            // Web Page Heading H1 with CSS
            webClient.println("<body>;<h1 style=\"color:Tomato;\">;ESP32 Access Point Web Server</h1>;");
 
            // Web Page Heading H2
            webClient.println("<h2 style=\"color:#077a39;\">;<a href=\"https://khuenguyencreator.com\">;khuenguyencreator.com</a>;</h2>;");
            webClient.println("<i class=\"fa fa-home\" aria-hidden=\"true\">;</i>;");
 
            // Display current state, and ON/OFF buttons for Led1
            webClient.println("<p>;Led1 - State " + led1Status + "</p>;");
            // If the Led1Status is off, it displays the ON button       
            if (led1Status=="off") 
            {
              //khởi tạo một nút nhấn có đường dẫn đích là /led1/on
              webClient.println("<p>;<a href=\"/led1/on\">;<button class=\"button\">;ON</button>;</a>;</p>;");
            } 
            else
            {
              //khởi tạo một nút nhấn có đường dẫn đích là /led1/off
              webClient.println("<p>;<a href=\"/led1/off\">;<button class=\"button button2\">;OFF</button>;</a>;</p>;");
            } 
                
            // Display current state, and ON/OFF buttons for Led2
            webClient.println("<p>;Led2 - State " + led2Status + "</p>;");
            // If the led2 is off, it displays the ON button       
            if (led2Status=="off") 
            {
              //khởi tạo một nút nhấn có đường dẫn đích là /led2/on
              webClient.println("<p>;<a href=\"/led2/on\">;<button class=\"button\">;ON</button>;</a>;</p>;");
            } 
            else
            {
              //khởi tạo một nút nhấn có đường dẫn đích là /led2/on
              webClient.println("<p>;<a href=\"/led2/off\">;<button class=\"button button2\">;OFF</button>;</a>;</p>;");
            }
            webClient.println("</body>;</html>;");
             
            // The HTTP response ends with another blank line
            webClient.println();
            // Break out of the while loop
            break;
          }
          else
          {
            currentLine = "";
          }
        }
        else if (c != '\r')   //nếu giá trị gửi tới khác xuống duòng
        {
          currentLine += c;     //lưu giá trị vào biến
        }
      }
    }
    // Xoá header để sử dụng cho lần tới
    header = "";
    // ngắt kết nối
    webClient.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
 
  }
}

*/

//Encoder
// ----- KHAI BÁO CHÂN -----
#define ENCODER1_A 34
#define ENCODER1_B 35
#define ENCODER2_A 18
#define ENCODER2_B 5
#define ENCODER3_A 17
#define ENCODER3_B 16
#define ENCODER4_A 2//21
#define ENCODER4_B 13//22

// ----- BIẾN LƯU ĐẾM -----
volatile long count1 = 0;
volatile long count2 = 0;
volatile long count3 = 0;
volatile long count4 = 0;

// ----- BIẾN LƯU TRẠNG THÁI TRƯỚC -----
volatile int lastState1_A, lastState1_B;
volatile int lastState2_A, lastState2_B;
volatile int lastState3_A, lastState3_B;
volatile int lastState4_A, lastState4_B;

// ---------- NGẮT CHO ENCODER 1 ----------
void IRAM_ATTR updateEncoder1() {
  int A = digitalRead(ENCODER1_A);
  int B = digitalRead(ENCODER1_B);
  if (A != lastState1_A) { // A thay đổi
    if (A == B) count1++; else count1--;
    lastState1_A = A;
  }
  else if (B != lastState1_B) { // B thay đổi
    if (A != B) count1++; else count1--;
    lastState1_B = B;
  }
}

// ---------- NGẮT CHO ENCODER 2 ----------
void IRAM_ATTR updateEncoder2() {
  int A = digitalRead(ENCODER2_A);
  int B = digitalRead(ENCODER2_B);
  if (A != lastState2_A) {
    if (A == B) count2++; else count2--;
    lastState2_A = A;
  }
  else if (B != lastState2_B) {
    if (A != B) count2++; else count2--;
    lastState2_B = B;
  }
}

// ---------- NGẮT CHO ENCODER 3 ----------
void IRAM_ATTR updateEncoder3() {
  int A = digitalRead(ENCODER3_A);
  int B = digitalRead(ENCODER3_B);
  if (A != lastState3_A) {
    if (A == B) count3++; else count3--;
    lastState3_A = A;
  }
  else if (B != lastState3_B) {
    if (A != B) count3++; else count3--;
    lastState3_B = B;
  }
}

// ---------- NGẮT CHO ENCODER 4 ----------
void IRAM_ATTR updateEncoder4() {
  int A = digitalRead(ENCODER4_A);
  int B = digitalRead(ENCODER4_B);
  if (A != lastState4_A) {
    if (A == B) count4++; else count4--;
    lastState4_A = A;
  }
  else if (B != lastState4_B) {
    if (A != B) count4++; else count4--;
    lastState4_B = B;
  }
}

void setup_Encoder() {
  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);
  pinMode(ENCODER3_A, INPUT_PULLUP);
  pinMode(ENCODER3_B, INPUT_PULLUP);
  pinMode(ENCODER4_A, INPUT_PULLUP);
  pinMode(ENCODER4_B, INPUT_PULLUP);

  // Lưu trạng thái ban đầu
  lastState1_A = digitalRead(ENCODER1_A);
  lastState1_B = digitalRead(ENCODER1_B);
  lastState2_A = digitalRead(ENCODER2_A);
  lastState2_B = digitalRead(ENCODER2_B);
  lastState3_A = digitalRead(ENCODER3_A);
  lastState3_B = digitalRead(ENCODER3_B);
  lastState4_A = digitalRead(ENCODER4_A);
  lastState4_B = digitalRead(ENCODER4_B);

  // Gắn ngắt cho cả A và B (Mode 4)
  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER1_B), updateEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_B), updateEncoder2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_A), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER3_B), updateEncoder3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_A), updateEncoder4, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER4_B), updateEncoder4, CHANGE);
}

//Speed
// --- Khai báo ---
//#define PPR 600   // số xung mỗi vòng (encoder 600 xung/vòng)
#define MODE 4    // mode 4x (đọc cả rising/falling 2 kênh)
#define CPR   2200 //(PPR * MODE)  // counts per revolution
float rpm1 = 0, rpm2 = 0, rpm3 = 0, rpm4 = 0;
float deltaT = 1000;
void processEncoder() {
  static long lastCount1 = 0, lastCount2 = 0, lastCount3 = 0, lastCount4 = 0;
  // In ra khi có thay đổi

    //snprintf(msg, sizeof(msg),
      //         "E1: %ld | E2: %ld | E3: %ld | E4: %ld",
         //      count1, count2, count3, count4);

    long delta1 = count1 - lastCount1;
    long delta2 = count2 - lastCount2;
    long delta3 = count3 - lastCount3;
    long delta4 = count4 - lastCount4;

    lastCount1 = count1;
    lastCount2 = count2;
    lastCount3 = count3;
    lastCount4 = count4;
    // Tính RPM
    float dt_sec = interval_Encoder / 1000.0;
    rpm1 = (delta1 / (float)CPR) * 60.0 / dt_sec;
    rpm2 = (delta2 / (float)CPR) * 60.0 / dt_sec;
     rpm3 = (delta3 / (float)CPR) * 60.0 / dt_sec;
     rpm4 = (delta4 / (float)CPR) * 60.0 / dt_sec;
  //client.publish("Kival/Output", msg);
  //Serial.printf("E1: %ld | E2: %ld | E3: %ld | E4: %ld\n",
  //              count1, count2, count3, count4);
  //snprintf(msg, sizeof(msg),
  //             "RPM1: %.2f | RPM2: %.2f | RPM3: %.2f | RPM4: %.2f\n",
   //            rpm1, rpm2, rpm3, rpm4);
  snprintf(msg, sizeof(msg),
           "RPM1:%.1f RPM2:%.1f RPM3:%.1f RPM4:%.1f",
            rpm1, rpm2, rpm3, rpm4);
  client.publish("Kival/Output", msg);
  Serial.printf("rpm:%.2f,min:30,max:80\n", rpm2);
  
}


// --- PID variables for each motor ---
float input1 = 0, input2 = 0, input3 = 0, input4 = 0;
float output1 = 0, output2 = 0, output3 = 0, output4 = 0;

// --- Error memory ---
float lastError1 = 0, lastError2 = 0, lastError3 = 0, lastError4 = 0;
float integral1 = 0, integral2 = 0, integral3 = 0, integral4 = 0;

// --- Tần suất PID (10ms = 100Hz) ---
const unsigned long interval_PID = 50;
unsigned long previousTime_PID = 0;
// --- PID parameters ---
float Kp_M1 = 5,  Ki_M1 = 3, Kd_M1 = 0.001;
float Kp_M2 = 5, Ki_M2 = 3, Kd_M2 = 0.001;
float Kp_M3 = 5, Ki_M3 = 3, Kd_M3 = 0.001;
float Kp_M4 = 5, Ki_M4 = 3, Kd_M4 = 0.001;
void handleCommand_PID(String cmd) {
  String cmdContent = mqttCommand.substring(2);
  if (cmdContent.startsWith("@") && cmdContent.endsWith("#")) {
    char cmd = cmdContent[1]; // P, I, D
    float value = cmdContent.substring(2, cmdContent.length()-1).toFloat();
    
    if (cmd == 'P') Kp_M1 = value;
    else if (cmd == 'I') Ki_M1 = value;
    else if (cmd == 'D') Kd_M1 = value;

    Serial.print("Updated "); Serial.print(cmd); 
    Serial.print(" = "); Serial.println(value);
  }
}
float computePID_M1(float setpoint, float input, float &lastError, float &integral) {
  float error = setpoint - fabs(input);
  integral += error * (interval_PID / 1000.0);
  float derivative = (error - lastError) / (interval_PID / 1000.0);
  float output = Kp_M1 * error + Ki_M1 * integral + Kd_M1 * derivative;
  lastError = error;
  return output;
}
float computePID_M2(float setpoint, float input, float &lastError, float &integral) {
  float error = setpoint - fabs(input);
  integral += error * (interval_PID / 1000.0);
  float derivative = (error - lastError) / (interval_PID / 1000.0);
  float output = Kp_M2 * error + Ki_M2 * integral + Kd_M2 * derivative;
  lastError = error;
  return output;
}
float computePID_M3(float setpoint, float input, float &lastError, float &integral) {
  float error = setpoint - fabs(input);
  integral += error * (interval_PID / 1000.0);
  float derivative = (error - lastError) / (interval_PID / 1000.0);
  float output = Kp_M3 * error + Ki_M3 * integral + Kd_M3 * derivative;
  lastError = error;
  return output;
}
float computePID_M4(float setpoint, float input, float &lastError, float &integral) {
  float error = setpoint - fabs(input);
  integral += error * (interval_PID / 1000.0);
  float derivative = (error - lastError) / (interval_PID / 1000.0);
  float output = Kp_M4 * error + Ki_M4 * integral + Kd_M4 * derivative;
  lastError = error;
  return output;
}

void setup() {
  setup_motor();
  setup_MQTT();
  setup_OTA();
  //setup_Wifi_AP();
  setup_PC13();
  setup_Encoder();
}

void loop() {
  currentTime = millis();

  // MQTT
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // OTA server
  server.handleClient();
  /*
  // Heartbeat
  if (currentTime - previousTime >= 2000) {
    previousTime = currentTime;
    Serial.println("Hello there :)");
  }
  */
  

  // Xử lý lệnh MQTT nếu có
  if (mqttCommand.length() > 0) {
    handleCommand(mqttCommand);
    handleCommand_PID(mqttCommand);
    mqttCommand = "";
  }

  // Cập nhật encoder theo chu kỳ interval_Encoder
  if (currentTime - previousTime_Encoder >= interval_Encoder) {
    previousTime_Encoder = currentTime;
    processEncoder();
  }

  // --- PID Control ---
  unsigned long now = millis();

  if (now - previousTime_PID >= interval_PID) {
    previousTime_PID = now;

    output1 = computePID_M1(setpoint1, rpm1, lastError1, integral1);
    output2 = computePID_M2(setpoint2, rpm2, lastError2, integral2);
    output3 = computePID_M3(setpoint3, rpm3, lastError3, integral3);
    output4 = computePID_M4(setpoint4, rpm4, lastError4, integral4);

    // Giới hạn tốc độ PWM
    M1_Speed = constrain(output1, 0, 255);
    M2_Speed = constrain(output2, 0, 255);
    M3_Speed = constrain(output3, 0, 255);
    M4_Speed = constrain(output4, 0, 255);

    moveMotor(M1_Speed, M2_Speed, M3_Speed, M4_Speed, STATION);
  }
  // Dừng motor sau 1500 ms kể từ lúc moveMotor được gọi
  if (motorRunning && (millis() - motorStartTime >= 1500)) {
    stopMotors();
  }

  // nhỏ nhẹ yield để tránh block
  delay(1);
}

