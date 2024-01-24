/*********************************************************************
 Pico W USB Host MQTT Numpad

 Copyright (c) 2023 Baiyibai
 MIT license, check LICENSE for more information

 Many additional notices and notes retained.
*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/
/*
 This example sketch was written by copying and pasting the source code of the following examples.
 Please read the original source code for details.
 The license of the original source code therefore applies.
 Please use 2.0.2 or later of "TinyUSB Library for Arduino"
(Translated into English on a translation site.)
 
 https://github.com/sekigon-gonnoc/Pico-PIO-USB/blob/main/examples/arduino/device_info/device_info.ino
 https://github.com/hathach/tinyusb/blob/master/examples/host/cdc_msc_hid/src/hid_app.c
 https://github.com/hathach/tinyusb/blob/master/examples/host/hid_controller/src/hid_app.c

// 2023-09-16 BYB MQTT + Home Assistant
I purchased a cheap 2.4 GHz 18-key numpad

Buttons usually work by Press/Release events
Using xev, it seems that all the keys are prefixed by "KP_"
Pressing the Numpad key enables a second function, hopefully we don't have to deal with detecting this. 
Implementing "shift" keys or multi-digit combinations is an exercise that others can experiment with. 
Fill a buffer and then send on pressing enter.

What really needs to be done:
Clean up the code so that there is a dictionary of keycodes and payloads at the beginning of the file.
Modify the code so that the keycodes are printed instead of being converted to ascii characters (del/enter/backspace) to allow debugging.

Add more tutorials on how to connect to home assistant

//https://community.home-assistant.io/t/media-player-volume-mute-toggle/302265/10


*/

#include <WiFi.h>
#include <PubSubClient.h> 

// pio-usb is required for rp2040 host
#include "pio_usb.h"
#define HOST_PIN_DP   2   // Pin used as D+ for host, D- = D+ + 1

#include "Adafruit_TinyUSB.h"

#define LANGUAGE_ID 0x0409  // English
#define PIN_Serial1_TX (28u) // marked A1 on the Board
#define PIN_Serial1_RX (29u) // marked A0 on the Board
#define PIN_Serial2_TX (8u)
#define PIN_Serial2_RX (9u)

// # tag::user-defined-settings[]
//Wi-Fi Settings
char ssid[] = "{ssid}"; // <1>
char password[] = "{password}"; // <2>
//MQTT Settings
char* MQTTServer = "{mqtt-IPv4-address}"; // <3>
int MQTTPort = 1883; // <4>
char* MQTTUser = "{mqtt-username}"; // <5>
char* MQTTPassword = "{mqtt-password}"; // Need to escape / <6>
char* MQTTSubTopic1 = "placeholder"; // <7>
long MQTTLastPublishTime;
long MQTTPublishInterval = 10000;
bool numlock = false; //  Need to track numlock state
byte mac[6]; //Array for the MAC address
// # end::user-defined-settings[]

WiFiClient WifiClient;
PubSubClient MQTTClient(WifiClient);

// USB Host object
Adafruit_USBH_Host USBHost;

// holding device descriptor
tusb_desc_device_t desc_device;

#define MAX_REPORT  4

//#define PRINTREPORT // Uncomment to print the report received from the HID device

// Keyboard
#define USE_ANSI_ESCAPE   0
static uint8_t const keycode2ascii[128][2] =  { HID_KEYCODE_TO_ASCII };

// Each HID idx can has multiple reports
static struct
{
  uint8_t report_count;
  tuh_hid_report_info_t report_info[MAX_REPORT];
}hid_info[CFG_TUH_HID];

typedef struct TU_ATTR_PACKED
{
  uint8_t mbuttons;
  int16_t x : 12; //Usage X & Usage Y Report Size 0x0C (12bit)
  int16_t y : 12; //
  int8_t  wheel;
  int8_t  pan;
} xy12mouse_report_t;

typedef struct TU_ATTR_PACKED
{
  uint8_t mbuttons;
  int16_t x; //Usage X & Usage Y Report Size 0x10 (16bit)
  int16_t y;
  int8_t  wheel;
  int8_t  pan;
} xy16mouse_report_t;

//------------------------------------------------------------------------

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial2.begin(115200);
  Serial2.println("TinyUSB Setting up Wi-Fi and MQTT");
  WiFiConnect();
  MQTTConnect();
  while ( !Serial2 ) delay(10);   // wait for native usb
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) { MQTTConnect(); 
  }

  if (!MQTTClient.connected()) {  MQTTConnect(); }
}
//------------------------------------------------------------------------

// core1's setup
void setup1() {
  while ( !Serial2 ) delay(10);   // wait for native usb
  Serial2.println("Core1 setup to run TinyUSB host with pio-usb");

  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial2 ) delay(10);   // wait for native usb
    Serial2.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial2.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while(1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(0);
}

// core1's loop
void loop1()
{
  USBHost.task();
}
//------------------------------------------------------------------------

//WiFi
void WiFiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial2.print(".");
  }
  Serial2.println("Wi-Fi Connected");
  Serial2.print("IP Address:");
  Serial2.println(WiFi.localIP());
}

//MQTT
void MQTTConnect() {
  MQTTClient.setServer(MQTTServer, MQTTPort);
  MQTTClient.setCallback(MQTTCallback);
  while (!MQTTClient.connected()) {
    //ClientID
    String MQTTClientid_str = "Pico_W" + WiFi.macAddress();  
    MQTTClientid_str.replace(":","_"); // No colons allowed in mqtt
    
    if (MQTTClient.connect(MQTTClientid_str.c_str(), MQTTUser, MQTTPassword)) {
      //連結成功，顯示「已連線」。
      Serial2.println("MQTT Connected");
      //訂閱SubTopic1主題
      MQTTClient.subscribe(MQTTSubTopic1);
    } else {
      //若連線不成功，則顯示錯誤訊息，並重新連線
      Serial2.print("MQTT connection failed,status code=");
      Serial2.println(MQTTClient.state());
      Serial2.println("Trying to reconnect in five seconds...");
      delay(5000);
    }
  }
}

//接收到訂閱時
void MQTTCallback(char* topic, byte* payload, unsigned int length) {
  Serial2.print(topic); Serial2.print("訂閱通知:");
  String payloadString;//將接收的payload轉成字串
  //顯示訂閱內容
  for (int i = 0; i < length; i++) {
    payloadString = payloadString + (char)payload[i];
  }
  Serial2.println(payloadString);
  //比對主題是否為訂閱主題1
  if (strcmp(topic, MQTTSubTopic1) == 0) {
    Serial2.println("改變燈號：" + payloadString);
  }
}




//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted (configured)
void tuh_mount_cb (uint8_t dev_addr)
{
  Serial2.printf("Device attached, address = %d\r\n", dev_addr);
  Serial2.printf("---------------------------\r\n");

  // Get Device Descriptor
  tuh_descriptor_get_device(dev_addr, &desc_device, 18, print_device_descriptor, 0);
}

/// Invoked when device is unmounted (bus reset/unplugged)
void tuh_umount_cb(uint8_t dev_addr)
{
  Serial2.printf("\r\n");
  Serial2.printf("Device removed, address = %d\r\n", dev_addr);
  Serial2.printf("---------------------------\r\n");
}

// Invoked when device with hid interface is mounted
// Report descriptor is also available for use. tuh_hid_parse_report_descriptor()
// can be used to parse common/simple enough descriptor.
// Note: if report descriptor length > CFG_TUH_ENUMERATION_BUFSIZE, it will be skipped
// therefore report_desc = NULL, desc_len = 0
void tuh_hid_mount_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* desc_report, uint16_t desc_len)
{
  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  Serial2.printf("HID device address = %d, idx = %d is mounted\r\n", dev_addr, idx);
  Serial2.printf("VID = %04x, PID = %04x\r\n", vid, pid);

  // By default host stack will use activate boot protocol on supported interface.
  // Therefore for this simple example, we only need to parse generic report descriptor (with built-in parser)
  hid_info[idx].report_count = tuh_hid_parse_report_descriptor(hid_info[idx].report_info, MAX_REPORT, desc_report, desc_len);
  Serial2.printf("HID has %u reports \r\n", hid_info[idx].report_count);

  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, idx);

  switch(itf_protocol){
    case HID_ITF_PROTOCOL_NONE: //HID_PROTOCOL_BOOT:NONE
      Serial2.printf("Detected NONE\r\n");


    case HID_ITF_PROTOCOL_KEYBOARD: //HID_PROTOCOL_BOOT:KEYBOARD
      Serial2.printf("Detected KEYBOARD\r\n");
      break;

    }

  if ( !tuh_hid_receive_report(dev_addr, idx) )
  {
    Serial2.printf("Error: cannot request to receive report\r\n");
  }
  Serial2.printf("------------------------------------------------\r\n");
}


// Invoked when device with hid interface is un-mounted
void tuh_hid_umount_cb(uint8_t dev_addr, uint8_t idx)
{
  Serial2.printf("HID device address = %d, idx = %d is unmounted\r\n", dev_addr, idx);
  Serial2.printf("--------------------------------------------------\r\n");

}


// Invoked when received report from device via interrupt endpoint
void tuh_hid_report_received_cb(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len)
{
#ifdef PRINTREPORT
  Serial2.println("tuh_hid_report_received_cb");
  Serial2.print(":dev_addr = ");
  Serial2.print(dev_addr);
  Serial2.print(" :idx = ");
  Serial2.println(idx);
  Serial2.print("report");
  for(uint16_t i=0;i<len;i++){
    Serial2.print(":");
    if(report[i]<0x10) Serial2.print("0");
    Serial2.print(report[i],HEX);
  }
  Serial2.println("");
#endif

  uint8_t const itf_protocol = tuh_hid_interface_protocol(dev_addr, idx);

  switch (itf_protocol)
  {
    case HID_ITF_PROTOCOL_KEYBOARD:
//    Serial2.printf("HID receive boot keyboard report\r\n");
      process_boot_kbd_report( (hid_keyboard_report_t const*) report );
    break;


    default:
      // Generic report requires matching ReportID and contents with previous parsed report info
      process_generic_report(dev_addr, idx, report, len);
    break;
  }

  // continue to request to receive report
  if ( !tuh_hid_receive_report(dev_addr, idx) )
  {
    Serial2.printf("Error: cannot request to receive report\r\n");
  }
}


//--------------------------------------------------------------------+
// Generic Report
//--------------------------------------------------------------------+
void process_generic_report(uint8_t dev_addr, uint8_t idx, uint8_t const* report, uint16_t len)
{
#ifdef PRINTREPORT
  Serial2.println("process_generic_report");
  Serial2.print(":dev_addr = ");
  Serial2.print(dev_addr);
  Serial2.print(" :idx = ");
  Serial2.println(idx);
  Serial2.print("report");
  for(uint16_t i=0;i<len;i++){
    Serial2.print(":");
    if(report[i]<0x10) Serial2.print("0");
    Serial2.print(report[i],HEX);
  }
  Serial2.println("");
#endif
  uint8_t const rpt_count = hid_info[idx].report_count;
  tuh_hid_report_info_t* rpt_info_arr = hid_info[idx].report_info;
  tuh_hid_report_info_t* rpt_info = NULL;

  if ( rpt_count == 1 && rpt_info_arr[0].report_id == 0)
  {
    // Simple report without report ID as 1st byte
    rpt_info = &rpt_info_arr[0];
  }else
  {
    // Composite report, 1st byte is report ID, data starts from 2nd byte
    uint8_t const rpt_id = report[0];

    // Find report id in the array
    for(uint8_t i=0; i<rpt_count; i++)
    {
      if (rpt_id == rpt_info_arr[i].report_id )
      {
        rpt_info = &rpt_info_arr[i];
        break;
      }
    }

    report++;
    len--;
  }

  uint16_t vid, pid;
  tuh_vid_pid_get(dev_addr, &vid, &pid);

  // For complete list of Usage Page & Usage checkout src/class/hid/hid.h. For examples:
  // - Keyboard                     : Desktop, Keyboard
  // - Mouse                        : Desktop, Mouse
  // - Gamepad                      : Desktop, Gamepad
  // - Consumer Control (Media Key) : Consumer, Consumer Control
  // - System Control (Power key)   : Desktop, System Control
  // - Generic (vendor)             : 0xFFxx, xx

  if ( rpt_info->usage_page == HID_USAGE_PAGE_DESKTOP )
  {
    switch (rpt_info->usage)
    {
      case HID_USAGE_DESKTOP_KEYBOARD:
        Serial2.println("process_generic_report");
        Serial2.println("HID receive keyboard report");
/*
        if ((vid == xxxx) && (pid == yyyy))
        {
          process_other_kbd_report(dev_addr, idx, report, len);
        } else
*/
        {
          // Assume keyboard follow boot report layout
          process_boot_kbd_report( (hid_keyboard_report_t const*) report );
        }
        break;

      default:
        Serial2.println("process_generic_report");
        Serial2.println("HID device report other than keyboard and mouce");
/*
        if ((vid == xxxx) && (pid == yyyy))
        {
          process_aaa_report(dev_addr, idx, report, len);
        }
*/
        break;
    }
  }
  else //Other than HID_USAGE_PAGE_DESKTOP
  {
//  Serial2.printf("Other than HID_USAGE_PAGE_DESKTOP\r\n");
/*
    else if ((vid == xxxx) && (pid == yyyy))
    {
      process_bbb_report(dev_addr, idx, report, len);
    }
*/

  }
}


//--------------------------------------------------------------------+
// Keyboard
//--------------------------------------------------------------------+


// look up new key in previous keys
static inline bool find_key_in_report(hid_keyboard_report_t const *report, uint8_t keycode)
{
  
  // 2023-09-16: check for keycodes here, 
  // For my particular 18-key wireless numpad, if numlock is enabled, the numlock keycode is sent first.
  // Therefore, the state of numpad must be kept track of.
  // The wireless keypad keeps track of it's own state, there is no sync between the dongle and the keypad.

  // Design Decision: There is no fixed numerical offset between the numpad keys with and without numlock enabled. 
  // Therefore, add 100 to the MQTT value.
  
  // Define the MQTT Topic. 
  // This is here because defining the MQTTPubTopic_char at the top and changing in setup() resulted in a corrupted topic, core sharing problem?
  String MQTTPubTopic_Str = "Pico_W/USB_HID/" + WiFi.macAddress() + "/";  
  MQTTPubTopic_Str.replace(":","_"); // No colons allowed in mqtt
  int str_len = MQTTPubTopic_Str.length() + 1; 
  char MQTTPubTopic_char[str_len];
  MQTTPubTopic_Str.toCharArray(MQTTPubTopic_char, str_len);

  // Uncomment out to view the MAC address to aid with setting up Home Assistant/MQTT.
  // Trying to keep print statements to a minimum because this takes CPU time and actual time
  
  // # tag::debug-mqtt-topic[]
  // Serial2.println(MQTTPubTopic_Str); 
  // # end::debug-mqtt-topic[]

  // Four situations defined.
  
  // Detect if the numpad key is pressed.
  if ( keycode == 243 ) 
  {
    numlock = !numlock; // Swap the status
    Serial2.println("Numlock key pressed (243).");
  }

  // The keycode 83 is sent before all other keycodes when numlock is on
  // Use this to set the numlock state (One additional layer)
  if ( keycode == 83 ) 
  { 
    numlock = true;
  }
  
  // Numlock on = keycode + 100
  if ( numlock == true && keycode != 83 && keycode != 243) 
    { 
      Serial2.print(keycode + 100);
      Serial2.println(" NL ON");
      // Add 100 to the keycode when using numlock
      MQTTClient.publish(MQTTPubTopic_char, String(keycode + 100).c_str());
    }
  
  // Numlock off = keycode
  if (numlock == false && keycode != 243) 
  { 
      Serial2.print(keycode);
      Serial2.println(" NL ON");
    MQTTClient.publish(MQTTPubTopic_char, String(keycode).c_str());
  }
  
  for(uint8_t i=0; i<6; i++)
  {
    if (report->keycode[i] == keycode)  return true;
  }

  return false;
}

void process_boot_kbd_report(hid_keyboard_report_t const *report)
{
  hid_keyboard_report_t prev_report = { 0, 0, {0} }; // previous report to check key released


  //------------- example code ignore control (non-printable) key affects -------------//
  for(uint8_t i=0; i<6; i++)
  {
    
    if ( report->keycode[i] )
    {

      if ( find_key_in_report(&prev_report, report->keycode[i]) )
      {
        // exist in previous report means the current key is holding
      }else
      {
        // not existed in previous report means the current key is pressed
        bool const is_shift = report->modifier & (KEYBOARD_MODIFIER_LEFTSHIFT | KEYBOARD_MODIFIER_RIGHTSHIFT);
        
        char ch = keycode2ascii[report->keycode[i]][is_shift ? 1 : 0];
//      uint8_t ch = keycode2ascii[report->keycode[i]][is_shift ? 1 : 0];

//      putchar(ch);
        //if ( ch == '\r' ) Serial2.print('\n'); // added new line for enter key
//      if ( ch == '\r' ) putchar('\n'); // added new line for enter key

//      fflush(stdout); // flush right away, else nanolib will wait for newline
      }
    }
  }

  prev_report = *report;
}

//--------------------------------------------------------------------+
// print_device_descriptor
//--------------------------------------------------------------------+

void print_device_descriptor(tuh_xfer_t* xfer)
{
  if ( XFER_RESULT_SUCCESS != xfer->result )
  {
    Serial2.printf("Failed to get device descriptor\r\n");
    return;
  }

  uint8_t const daddr = xfer->daddr;

  Serial2.printf("print_device_descriptor\r\n");
  Serial2.printf("Device %u: ID %04x:%04x\r\n", daddr, desc_device.idVendor, desc_device.idProduct);
  Serial2.printf("Device Descriptor:\r\n");
  Serial2.printf("  bLength             %u\r\n"     , desc_device.bLength);
  Serial2.printf("  bDescriptorType     %u\r\n"     , desc_device.bDescriptorType);
  Serial2.printf("  bcdUSB              %04x\r\n"   , desc_device.bcdUSB);
  Serial2.printf("  bDeviceClass        %u\r\n"     , desc_device.bDeviceClass);
  Serial2.printf("  bDeviceSubClass     %u\r\n"     , desc_device.bDeviceSubClass);
  Serial2.printf("  bDeviceProtocol     %u\r\n"     , desc_device.bDeviceProtocol);
  Serial2.printf("  bMaxPacketSize0     %u\r\n"     , desc_device.bMaxPacketSize0);
  Serial2.printf("  idVendor            0x%04x\r\n" , desc_device.idVendor);
  Serial2.printf("  idProduct           0x%04x\r\n" , desc_device.idProduct);
  Serial2.printf("  bcdDevice           %04x\r\n"   , desc_device.bcdDevice);

  // Get String descriptor using Sync API
  uint16_t temp_buf[128];

  Serial2.printf("  iManufacturer       %u     "     , desc_device.iManufacturer);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_manufacturer_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)) )
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial2.printf("\r\n");

  Serial2.printf("  iProduct            %u     "     , desc_device.iProduct);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_product_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial2.printf("\r\n");

  Serial2.printf("  iSerialNumber       %u     "     , desc_device.iSerialNumber);
  if (XFER_RESULT_SUCCESS == tuh_descriptor_get_serial_string_sync(daddr, LANGUAGE_ID, temp_buf, sizeof(temp_buf)))
  {
    print_utf16(temp_buf, TU_ARRAY_SIZE(temp_buf));
  }
  Serial2.printf("\r\n");

  Serial2.printf("  bNumConfigurations  %u\r\n"     , desc_device.bNumConfigurations);
  Serial2.printf("------------------------------------------------\r\n");
}

//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
  // TODO: Check for runover.
  (void)utf8_len;
  // Get the UTF-16 length out of the data itself.

  for (size_t i = 0; i < utf16_len; i++) {
    uint16_t chr = utf16[i];
    if (chr < 0x80) {
      *utf8++ = chr & 0xff;
    } else if (chr < 0x800) {
      *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    } else {
      // TODO: Verify surrogate.
      *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
      *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
  size_t total_bytes = 0;
  for (size_t i = 0; i < len; i++) {
    uint16_t chr = buf[i];
    if (chr < 0x80) {
      total_bytes += 1;
    } else if (chr < 0x800) {
      total_bytes += 2;
    } else {
      total_bytes += 3;
    }
    // TODO: Handle UTF-16 code points that take two entries.
  }
  return total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len) {
  size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
  size_t utf8_len = _count_utf8_bytes(temp_buf + 1, utf16_len);

  _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *) temp_buf, sizeof(uint16_t) * buf_len);
  ((uint8_t*) temp_buf)[utf8_len] = '\0';

  Serial2.printf((char*)temp_buf);
}