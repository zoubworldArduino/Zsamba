#include <sam_ba_monitor.h>

#include "sam_ba_uart.h"
#include "sam_ba_wire.h"
#include "sam_ba_cdc.h"

/*extern volatile int8_t ledDirection;
extern void board_init(void);
*/
#define BOARD_LED_FADE_ENABLED 1
#define BOARD_LED_PIN                     (19)
#define BOARD_LED_POLARITY	LED_POLARITY_HIGH_ON

 __inline__  void LED_init(void) { 
 pinMode(BOARD_LED_PIN, OUTPUT);
  digitalWrite(BOARD_LED_PIN, !BOARD_LED_POLARITY);  
  }
 __inline__  void LED_on(void) { 
  digitalWrite(BOARD_LED_PIN, BOARD_LED_POLARITY);  }

 __inline__  void LED_off(void) { 
 
  digitalWrite(BOARD_LED_PIN, !BOARD_LED_POLARITY); 
  }
 __inline__  void LED_toggle(void) { }
 __inline__   void LED_pulse() { }
 __inline__  void LED_setSpeed(uint8_t s) { }
#if (defined DEBUG) && (DEBUG == 1)
volatile uint32_t* pulSketch_Start_Address;
#endif

static volatile bool main_b_cdc_enable = false;

/**
 * \brief Check the application startup condition
 *
 */
static void check_start_application(void)
{

#if (!defined DEBUG) || ((defined DEBUG) && (DEBUG == 0))
uint32_t* pulSketch_Start_Address;
#endif

  /*
   * Test sketch stack pointer @ &__sketch_vectors_ptr
   * Stay in SAM-BA if value @ (&__sketch_vectors_ptr) == 0xFFFFFFFF (Erased flash cell value)
   */
  if (__sketch_vectors_ptr == 0xFFFFFFFF)
  {
    /* Stay in bootloader */
    return;
  }

  /*
   * Load the sketch Reset Handler address
   * __sketch_vectors_ptr is exported from linker script and point on first 32b word of sketch vector table
   * Remember, variables exported from a linker script are different than normal variables (only address, no value)
   * First 32b word is sketch stack
   * Second 32b word is sketch entry point: Reset_Handler()
   */
  pulSketch_Start_Address =(uint32_t* ) &__sketch_vectors_ptr ;
  pulSketch_Start_Address++ ;

  /*
   * Test vector table address of sketch @ &__sketch_vectors_ptr
   * Stay in SAM-BA if this function is not aligned enough, ie not valid
   */
  if ( ((uint32_t)(&__sketch_vectors_ptr) & ~SCB_VTOR_TBLOFF_Msk) != 0x00)
  {
    /* Stay in bootloader */
    return;
  }
#if defined(BOOT_LOAD_PIN_ENABLED)
  pinConfig(BOOT_LOAD_PIN_PORT, BOOT_LOAD_PIN, BOOT_LOAD_PIN_CONFIG);

  /* Allow time for debouncing capacitor (if using a button) to charge (10ms) */
  delayUs(10000UL);

  // Read the BOOT_LOAD_PIN status
  if (isPinActive(BOOT_LOAD_PIN_PORT, BOOT_LOAD_PIN, BOOT_LOAD_PIN_POLARITY))
  {
    // Stay in bootloader
    return;
  }
#endif

#if defined(BOOT_DOUBLE_TAP_ENABLED)
  #define DOUBLE_TAP_MAGIC 0x07738135
#if (SAMD21 ||SAMD20 || SAMD11)
  if (PM->RCAUSE.bit.POR)
	  /*
#elif (SAMC21 || SAMC20)
	 if (1==0)// C21 is buggy  RST==POR
 */
#elif (SAML21 || SAMC21|| SAMC20)
  if (RSTC->RCAUSE.bit.POR)
#else
  #error "main.c: Missing dependency or unsupported chip. Please install CMSIS-Atmel from MattairTech (see Prerequisites for Building in README.md)."
#endif
  {
    /* On power-on initialize double-tap */
    BOOT_DOUBLE_TAP_DATA = 0;
  }
  else
  {
    if (BOOT_DOUBLE_TAP_DATA == DOUBLE_TAP_MAGIC)
    {
      /* Second tap, stay in bootloader */
      BOOT_DOUBLE_TAP_DATA = 0;
      return;
    }

    /* First tap */
    BOOT_DOUBLE_TAP_DATA = DOUBLE_TAP_MAGIC;

    /* Wait 0.5sec to see if the user tap reset again */
     LED_off();
    delayUs(500000UL);
     LED_on();

    /* Timeout happened, continue boot... */
    BOOT_DOUBLE_TAP_DATA = 0;
  }
#endif


  /* Rebase the Stack Pointer */
  __set_MSP( (uint32_t)(__sketch_vectors_ptr) );

  /* Rebase the vector table base address */
  SCB->VTOR = ((uint32_t)(&__sketch_vectors_ptr) & SCB_VTOR_TBLOFF_Msk);
  LED_off();
  /* Jump to application Reset Handler in the application */
  asm("bx %0"::"r"(*pulSketch_Start_Address));
}

#if DEBUG_ENABLE
#	define DEBUG_PIN_HIGH 	port_pin_set_output_level(BOOT_LED, 1)
#	define DEBUG_PIN_LOW 	port_pin_set_output_level(BOOT_LED, 0)
#else
#	define DEBUG_PIN_HIGH 	do{}while(0)
#	define DEBUG_PIN_LOW 	do{}while(0)
#endif



void setup() {

#if defined(USBCON)
 #if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  P_USB_CDC pCdc;
  #endif
#endif
  DEBUG_PIN_HIGH;
 // delay_init(  VARIANT_MCK);
  LED_init();
  LED_on();
 
  /* Jump in application if condition is satisfied */
  check_start_application();
  /* We have determined we should stay in the monitor. */
  /* System initialization */
  //board_init();
  __enable_irq();
  /* Initialize LEDs */
  LED_init();
  #if !defined(BOARD_LED_FADE_ENABLED)
  LED_on();
  #endif

  /* Start the sys tick (20 us) */
  SysTick_Config(VARIANT_MCK / 50000);
wire_setup(Wire, 0x41,BOSSAC_ADDRESS);
uart_setup(SerialFTDI);


/* If SDCARD_ENABLED defined, read optional external pins and run SD Card bootloader (if enabled).
 */
  #if defined(SDCARD_ENABLED)
    uint8_t bootloaderMode = SD_BOOTLOADER_MODE_NO_UPDATE;
    #if defined(SAM_BA_INTERFACE_USE_PIN)
      uint8_t sdcardStatus = SD_BOOTLOADER_NOT_CALLED;
    #endif

    #if defined(SDCARD_USE_PIN1)
      pinConfig(SDCARD_PIN1_PORT, SDCARD_PIN1_PIN, SDCARD_PIN1_CONFIG);
    #endif
    #if defined(SDCARD_USE_PIN2)
      #if !defined(SDCARD_USE_PIN1)
        #error "main.c: When SDCARD_USE_PIN2 is defined, SDCARD_USE_PIN1 must also be defined"
      #endif
      pinConfig(SDCARD_PIN2_PORT, SDCARD_PIN2_PIN, SDCARD_PIN2_CONFIG);
    #endif

    #if (defined(SDCARD_USE_PIN1) && defined(SDCARD_USE_PIN2))
      bool pin1 = isPinActive(SDCARD_PIN1_PORT, SDCARD_PIN1_PIN, SDCARD_PIN1_POLARITY);
      bool pin2 = isPinActive(SDCARD_PIN2_PORT, SDCARD_PIN2_PIN, SDCARD_PIN2_POLARITY);
      if (pin1) {
        bootloaderMode = SD_BOOTLOADER_MODE_UPDATE;
      } else if (pin2) {
        bootloaderMode = SD_BOOTLOADER_MODE_UPDATE2;
      }
    #elif defined(SDCARD_USE_PIN1)
      if (isPinActive(SDCARD_PIN1_PORT, SDCARD_PIN1_PIN, SDCARD_PIN1_POLARITY)) {
        bootloaderMode = SD_BOOTLOADER_MODE_UPDATE;
      }
    #else
      bootloaderMode = SD_BOOTLOADER_MODE_UPDATE;
    #endif

    if (bootloaderMode != SD_BOOTLOADER_MODE_NO_UPDATE) {
    #if defined(SAM_BA_INTERFACE_USE_PIN)
      sdcardStatus = sdBootloader(bootloaderMode);
    #else
      sdBootloader(bootloaderMode);
    #endif
    }
  #endif

/* When using SAM_BA_INTERFACE_USE_PIN, check the pin to determine which SAM-BA interface to use.
 */
#if defined(SAM_BA_INTERFACE_USE_PIN)
  uint8_t sambaInterface = SAM_BA_NONE;
  pinConfig(SAM_BA_INTERFACE_PIN_PORT, SAM_BA_INTERFACE_PIN_PIN, SAM_BA_INTERFACE_PIN_CONFIG);

  if (isPinActive(SAM_BA_INTERFACE_PIN_PORT, SAM_BA_INTERFACE_PIN_PIN, PIN_POLARITY_ACTIVE_LOW)) {      // pin is low
  #if ((SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES) && (SAM_BA_INTERFACE_PIN_POLARITY == PIN_POLARITY_USBCDC_LOW))
    sambaInterface = SAM_BA_USBCDC_ONLY;
  #elif ((SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY) && (SAM_BA_INTERFACE_PIN_POLARITY == PIN_POLARITY_ACTIVE_LOW))
    sambaInterface = SAM_BA_USBCDC_ONLY;
  #elif ((SAM_BA_INTERFACE == SAM_BA_UART_ONLY) && (SAM_BA_INTERFACE_PIN_POLARITY == PIN_POLARITY_ACTIVE_LOW))
    sambaInterface = SAM_BA_UART_ONLY;
  #endif
  } else {      // pin is high
  #if ((SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES) && (SAM_BA_INTERFACE_PIN_POLARITY == PIN_POLARITY_USBCDC_HIGH))
    sambaInterface = SAM_BA_USBCDC_ONLY;
  #elif ((SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY) && (SAM_BA_INTERFACE_PIN_POLARITY == PIN_POLARITY_ACTIVE_HIGH))
    sambaInterface = SAM_BA_USBCDC_ONLY;
  #elif ((SAM_BA_INTERFACE == SAM_BA_UART_ONLY) && (SAM_BA_INTERFACE_PIN_POLARITY == PIN_POLARITY_ACTIVE_HIGH))
    sambaInterface = SAM_BA_UART_ONLY;
  #endif
  }
#endif

/* Check if we should continue with a SAM-BA interface (if compiled and enabled).
 * Only show LED_STATUS_FILE_NOT_FOUND if a SAM-BA interface is not available.
 * LED_STATUS_FILE_NOT_FOUND occurs if there is no SD Card, no FAT16/FAT32
 * filesystem, or no file (UPDATE.BIN or UPDATE2.BIN) in the root directory.
 */
#if defined(SAM_BA_INTERFACE_USE_PIN)
  #if defined(SDCARD_ENABLED)
  if (sambaInterface == SAM_BA_NONE) {
    if (sdcardStatus == SD_BOOTLOADER_FILE_NOT_FOUND) {
      LED_status(LED_STATUS_FILE_NOT_FOUND);
    } else if (sdcardStatus == SD_BOOTLOADER_NOT_CALLED) {
      LED_status(LED_STATUS_NO_SAM_BA_INTERFACE);
    }
  }
  #else
    #if (SAM_BA_INTERFACE == SAM_BA_NONE)
      LED_status(LED_STATUS_NO_SAM_BA_INTERFACE);
    #else
    if (sambaInterface == SAM_BA_NONE) {
      LED_status(LED_STATUS_NO_SAM_BA_INTERFACE);
    }
    #endif
  #endif
#else
  #if (SAM_BA_INTERFACE == SAM_BA_NONE)
    LED_status(LED_STATUS_NO_SAM_BA_INTERFACE);
  #endif
#endif

/* Enable the appropriate SAM-BA interfaces. When using SAM_BA_INTERFACE_USE_PIN,
 * the peripheral and pins are only setup for the selected/enabled interface.
 */
#if defined(SAM_BA_INTERFACE_USE_PIN)
  if (sambaInterface == SAM_BA_UART_ONLY) {
#endif

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  /* UART is enabled in all cases */
  LED_setSpeed(240);
  uart_open(115200);  
  LED_setSpeed(120);  
#endif

#if SAM_BA_INTERFACE == SAM_BA_WIRE_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  /* UART is enabled in all cases */
  LED_setSpeed(240);
  wire_open(115200);  
  LED_setSpeed(120);  
#endif

#if defined(SAM_BA_INTERFACE_USE_PIN)
  } else if (sambaInterface == SAM_BA_USBCDC_ONLY) {
#endif
#if defined(USBCON)
#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
  pCdc = usb_init();
#endif
#endif
#if defined(SAM_BA_INTERFACE_USE_PIN)
  }
#endif


  //ledDirection = 4;

  DEBUG_PIN_LOW;


  /* Wait for a complete enum on usb or a '#' char on serial line */
}
#define pCdc (&sam_ba_cdc)
void loop() {
  #if defined(SAM_BA_INTERFACE_USE_PIN)
    if (sambaInterface == SAM_BA_USBCDC_ONLY) {
#endif
#if defined(USBCON)

#if SAM_BA_INTERFACE == SAM_BA_USBCDC_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    if (pCdc->IsConfigured(pCdc) != 0)
    {
      main_b_cdc_enable = true;
    }

    /* Check if a USB enumeration has succeeded and if comm port has been opened */
    if (main_b_cdc_enable)
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_USBCDC);
      /* SAM-BA on USB loop */
      while( 1 )
      {
        sam_ba_monitor_run();
      }
    }
#endif

#endif
#if defined(SAM_BA_INTERFACE_USE_PIN)
    } else if (sambaInterface == SAM_BA_UART_ONLY) {
#endif

#if SAM_BA_INTERFACE == SAM_BA_UART_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    /* Check if a '#' has been received */
    if (!main_b_cdc_enable && uart_sharp_received())
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_USART);
      /* SAM-BA on Serial loop */
      while(1)
      {
        sam_ba_monitor_run();
      }
    }    
#endif

#if defined(SAM_BA_INTERFACE_USE_PIN)
    }
#endif
#if defined(SAM_BA_INTERFACE_USE_PIN)
    else if (sambaInterface == SAM_BA_WIRE_ONLY) {
#endif

#if SAM_BA_INTERFACE == SAM_BA_WIRE_ONLY  ||  SAM_BA_INTERFACE == SAM_BA_BOTH_INTERFACES
    /* Check if a '#' has been received */
    if (wire_sharp_received())
    {
      sam_ba_monitor_init(SAM_BA_INTERFACE_WIRE);
      /* SAM-BA on Serial loop */
      while(1)
      {
        sam_ba_monitor_run();
      }
    }    
#endif

#if defined(SAM_BA_INTERFACE_USE_PIN)
    }
#endif

}

#ifdef __cplusplus
extern "C"
{
#endif

int sysTickHook(void)
{


#if (defined(BOARD_LED_FADE_ENABLED) && defined(BOARD_LED_PORT))
  LED_pulse();
#endif

return false;// keep systick implementation of delay
}
#ifdef __cplusplus
}
#endif

