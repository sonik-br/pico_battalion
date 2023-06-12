/*********************************************************************
 Steel Battalion USB Adapter
 by Matheus Fraguas (sonik-br)
 https://github.com/sonik-br/pico_battalion

 For using on a RP2040 board

 Requires the arduino pico board definition from:
 https://github.com/earlephilhower/arduino-pico

 Requires the SBC TinyUSB driver from:
 https://github.com/sonik-br/tusb_drivers

 Requires Pico-PIO-USB lib
 https://github.com/sekigon-gonnoc/Pico-PIO-USB
 
*********************************************************************/

// pio-usb is required for rp2040 host
#include "pio_usb.h"
#define HOST_PIN_DP   10   // Pin used as D+ for host, D- = D+ + 1

#include "Adafruit_TinyUSB.h"

#define bitSet64(value, bit) ((value) |= ((uint64_t)1 << (bit)))

// USB Host object
Adafruit_USBH_Host USBHost;


//HID OUT
#define TUD_HID_REPORT_DESC_GAMEPAD3(...) \
  HID_USAGE_PAGE ( HID_USAGE_PAGE_DESKTOP     )                 ,\
  HID_USAGE      ( HID_USAGE_DESKTOP_GAMEPAD  )                 ,\
  HID_COLLECTION ( HID_COLLECTION_APPLICATION )                 ,\
    /* Report ID if any */\
    __VA_ARGS__ \
    /* 8 bit X, Y, Z, Rz, Rx, Ry, Slider, Wheel  */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_DESKTOP                 ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_X                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Y                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_Z                    ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RZ                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RX                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_RY                   ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_SLIDER               ) ,\
    HID_USAGE          ( HID_USAGE_DESKTOP_WHEEL                ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX_N  ( 255, 2                                 ) ,\
    HID_REPORT_COUNT   ( 8                                      ) ,\
    HID_REPORT_SIZE    ( 8                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
    /* 64 bit Button Map */ \
    HID_USAGE_PAGE     ( HID_USAGE_PAGE_BUTTON                  ) ,\
    HID_USAGE_MIN      ( 1                                      ) ,\
    HID_USAGE_MAX      ( 64                                     ) ,\
    HID_LOGICAL_MIN    ( 0                                      ) ,\
    HID_LOGICAL_MAX    ( 1                                      ) ,\
    HID_REPORT_COUNT   ( 64                                     ) ,\
    HID_REPORT_SIZE    ( 1                                      ) ,\
    HID_INPUT          ( HID_DATA | HID_VARIABLE | HID_ABSOLUTE ) ,\
  HID_COLLECTION_END \

typedef struct TU_ATTR_PACKED
{
  uint8_t  x;
  uint8_t  y;
  uint8_t  z;
  uint8_t  rz;
  uint8_t  rx;
  uint8_t  ry;
  uint8_t  slider;
  uint8_t  wheel;
  uint64_t buttons;
} hid_gamepad_report_t2;
  
// HID report descriptor using TinyUSB's template
// Single Report (no ID) descriptor
uint8_t const desc_hid_report3[] =
{
  TUD_HID_REPORT_DESC_GAMEPAD3()
};


// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
Adafruit_USBD_HID usb_hid(desc_hid_report3, sizeof(desc_hid_report3), HID_ITF_PROTOCOL_NONE, 1, false);

// Report payload
hid_gamepad_report_t2    gp;

typedef struct {
  uint8_t address;
  uint8_t instance;
} connected_device_t;

volatile bool connected {0};
volatile connected_device_t connected_device;


// the setup function runs once when you press reset or power the board
void setup()
{
  #if defined(ARDUINO_ARCH_MBED) && defined(ARDUINO_ARCH_RP2040)
    // Manual begin() is required on core without built-in support for TinyUSB such as mbed rp2040
    TinyUSB_Device_Init(0);
  #endif

  // usb_hid.setPollInterval(1);
  // usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));

  usb_hid.begin();

  // wait until device mounted
  while( !TinyUSBDevice.mounted() ) delay(1);
  
  Serial1.begin(115200);

  Serial.begin(115200);
  //while ( !Serial ) delay(10);   // wait for native usb

  Serial.println("TinyUSB Steel Battalion Controller adapter\r\n");
}

void loop() {

//to do: receive full led command and pass to device

//  if (Serial.available()) {
//    const char ch = Serial.read();
//    //send command to set leds on the controller
//    if (ch == 'z') {
//      static bool ledsEnabled = false;
//      
//      ledsEnabled = !ledsEnabled;
//
//      uint8_t ledsvalue = ledsEnabled ? 0x3 : 0x0; //each led value can be between 0x0 to 0xF
//
//      sbc_leds_t leds { };
//      
//      leds.EmergencyEject = ledsvalue;
//      leds.Start = ledsvalue;
//      leds.GearN = ledsvalue;
//      leds.Comm1 = ledsvalue;
//
//      if(!tuh_sbc_set_leds(connected_device.address, connected_device.instance, &leds)) {
//        Serial.println("LEDS ERROR\r\n");
//      }
//      return;
//    }
//  }
}

// core1's setup
void setup1() {
  // Check for CPU frequency, must be multiple of 120Mhz for bit-banging USB
  uint32_t cpu_hz = clock_get_hz(clk_sys);
  if ( cpu_hz != 120000000UL && cpu_hz != 240000000UL ) {
    while ( !Serial ) delay(10);   // wait for native usb
    Serial.printf("Error: CPU Clock = %u, PIO USB require CPU clock must be multiple of 120 Mhz\r\n", cpu_hz);
    Serial.printf("Change your CPU Clock to either 120 or 240 Mhz in Menu->CPU Speed \r\n", cpu_hz);
    while(1) delay(1);
  }

  pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
  pio_cfg.pin_dp = HOST_PIN_DP;
 
 #if defined(ARDUINO_RASPBERRY_PI_PICO_W)
  /* https://github.com/sekigon-gonnoc/Pico-PIO-USB/issues/46 */
  pio_cfg.sm_tx      = 3;
  pio_cfg.sm_rx      = 2;
  pio_cfg.sm_eop     = 3;
  pio_cfg.pio_rx_num = 0;
  pio_cfg.pio_tx_num = 1;
  pio_cfg.tx_ch      = 9;
 #endif /* ARDUINO_RASPBERRY_PI_PICO_W */
 
  USBHost.configure_pio_usb(1, &pio_cfg);

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);
}

// core1's loop
void loop1()
{
  USBHost.task();
}

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+

void tuh_sbc_mount_cb(uint8_t dev_addr, uint8_t instance, const sbch_interface_t *sbc_itf)
{
    Serial.printf("SBC MOUNTED %02x %d\n", dev_addr, instance);
    //start receiving report
    connected_device.address = dev_addr;
    connected_device.instance = instance;
    connected = true;
    tuh_sbc_receive_report(dev_addr, instance);
}

void tuh_sbc_report_received_cb(uint8_t dev_addr, uint8_t instance, uint8_t const* report, uint16_t len)
{
  //Serial.printf("SBC RECEIVED %x %x %x \n", dev_addr, instance, len);

  sbch_interface_t *xid_itf = (sbch_interface_t *)report;
  sbc_gamepad_t *p = &xid_itf->pad;


  if (xid_itf->connected && xid_itf->new_pad_data && usb_hid.ready())
  {
//    Serial.printf("[%02x, %02x], Buttons: %016llx, RotationLever: %02x, SightChangeX: %02x, SightChangeY: %02x, AimingX: %02x, AimingY: %02x, LeftPedal: %02x, MiddlePedal: %02x, RightPedal: %02x, TunerDial: %02x, GearLever: %02x\n",
//        dev_addr, instance, p->bButtons, p->bRotationLever, p->bSightChangeX, p->bSightChangeY, p->bAimingX, p->bAimingY, p->bLeftPedal, p->bMiddlePedal, p->bRightPedal, p->bTunerDial, p->bGearLever);

    gp.buttons = p->bButtons;
    
    gp.x = p->bSightChangeX + 127;
    gp.y = p->bSightChangeY + 127;
    gp.z = p->bAimingX;
    gp.rz = p->bAimingY;
    gp.rx = p->bMiddlePedal;
    gp.ry = p->bRightPedal;
    gp.slider = p->bLeftPedal;
    gp.wheel = p->bRotationLever + 127;
    bitSet64(gp.buttons, 39 + p->bTunerDial); //1 to 15

    if (p->bGearLever == 0xFE) //R
      bitSet64(gp.buttons, 55);
    else if (p->bGearLever == 0xFF) //N
      bitSet64(gp.buttons, 56);
    else
      bitSet64(gp.buttons, 56 + p->bGearLever); //1 to 5

    usb_hid.sendReport(0, &gp, sizeof(gp));
  }

  tuh_sbc_receive_report(dev_addr, instance);
}

void tuh_sbc_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    connected = false;
    connected_device.address = 0;
    connected_device.instance = 0;
    Serial.printf("SBC UNMOUNTED %02x %d\n", dev_addr, instance);
}
