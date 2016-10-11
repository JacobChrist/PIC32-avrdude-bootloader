/* PONTECH Boards */


#define     prodQuick240                0x0001
#define     prodPontechUAV100           0x0002
#define     prodPontechNoFire           0x0003

#if defined(_BOARD_UAV100_)  // 32MX440F512H
#define _CONFIG_VALID_

#if defined(PUT_CONFIG_BITS_HERE)

    //* Oscillator Settings
    #pragma config FNOSC    = PRIPLL                                // Oscillator selection
    #pragma config POSCMOD  = HS                                    // Primary oscillator mode
    #pragma config FPLLIDIV = DIV_2                                 // PLL input divider
    #pragma config FPLLMUL  = MUL_20                                // PLL multiplier
    #pragma config FPLLODIV = DIV_1                                 // PLL output divider
    #pragma config FPBDIV   = DIV_1                                 // Peripheral bus clock divider
    #pragma config FSOSCEN  = OFF                                   // Secondary oscillator enable

    //* Clock control settings
    #pragma config IESO     = OFF                                   // Internal/external clock switchover
    #pragma config FCKSM    = CSECME                                // Clock switching (CSx)/Clock monitor (CMx)
    #pragma config OSCIOFNC = OFF                                   // Clock output on OSCO pin enable

    //* USB Settings
    #pragma config UPLLEN   = ON                                    // USB PLL enable
    #pragma config UPLLIDIV = DIV_2                                 // USB PLL input divider

    //* Other Peripheral Device settings
    #pragma config FWDTEN   = OFF                                   // Watchdog timer enable
    #pragma config WDTPS    = PS1024                                // Watchdog timer postscaler

    //* Code Protection settings
    #pragma config CP       = OFF                                   // Code protection
    #pragma config BWP      = OFF                                   // Boot flash write protect
    #pragma config PWP      = OFF                                   // Program flash write protect

    //* Debug settings
    #pragma config ICESEL   = ICS_PGx2                              // ICE pin selection
#endif

    #define CAPABILITIES    (blCapBootLED | blCapUSBInterface | blCapProgramButton | blCapVirtualProgramButton | CAPCOMMON)

    // BTN / LED sense
    #define LedOn       High
    #define BntOn       Low

    // Boot LED
    #define BLedLat     F
    #define BLedBit     0

    // Virtual program button
    #define VPBntLat    D
    #define VPBntBit    4

    // Program button
    #define PBntPort    D
    #define PBntBit     4

    #define _CPU_NAME_                  "32MX440F512H"
    #define VEND                        vendPontech
    #define PROD                        prodPontechUAV100
    #define F_CPU                       80000000UL
    #define F_PBUS                      F_CPU

    #define FLASH_BYTES                 0x80000                     // 512K
    #define FLASH_PAGE_SIZE             4096
    #define LoadFlashWaitStates()       (CHECON = 2)            // 0 for 0-30Mhz, 1 for 31-60Mhz, 2 for 61-80Mhz
#endif

//************************************************************************

#if defined(_BOARD_QUICK240_)
#define _CONFIG_VALID_

#if defined(PUT_CONFIG_BITS_HERE)

    //*    Oscillator Settings
    #pragma config FNOSC        = PRIPLL                            // Oscillator selection
    #pragma config POSCMOD  = HS                                    // Primary oscillator mode
    #pragma config FPLLIDIV     = DIV_2                             // PLL input divider
    #pragma config FPLLMUL      = MUL_20                            // PLL multiplier
    #pragma config FPLLODIV     = DIV_1                             // PLL output divider
    #pragma config FPBDIV       = DIV_1                             // Peripheral bus clock divider
    #pragma config FSOSCEN      = OFF                               // Secondary oscillator enable

    //*    Clock control settings
    #pragma config IESO         = OFF                               // Internal/external clock switchover
    #pragma config FCKSM    = CSECME                                // Clock switching (CSx)/Clock monitor (CMx)
    #pragma config OSCIOFNC     = OFF                               // Clock output on OSCO pin enable

    //* USB Settings
    #pragma config UPLLEN   = ON                                    // USB PLL enable
    #pragma config UPLLIDIV = DIV_2                                 // USB PLL input divider
    #pragma config FUSBIDIO = OFF                                   // USB USID pin controlled by port function
    #pragma config FVBUSONIO = OFF                                  // USB VBUSON pin controlled by port function

    //*    Other Peripheral Device settings
    #pragma config FWDTEN       = OFF                               // Watchdog timer enable
    #pragma config WDTPS        = PS1024                            // Watchdog timer postscaler

    //*    Code Protection settings
    #pragma config CP           = OFF                               // Code protection
    #pragma config BWP          = OFF                               // Boot flash write protect
    #pragma config PWP          = OFF                               // Program flash write protect

    //*    Debug settings
    #pragma config ICESEL       = ICS_PGx2                          // ICE pin selection

    //*    Other Peripheral Device settings
    #pragma config FSRSSEL      = PRIORITY_7                        // SRS interrupt priority
    #pragma config FCANIO       = ON                               // Standard/alternate CAN pin select (OFF=Alt)
    #pragma config FETHIO       = ON                                // Standard/alternate ETH pin select (OFF=Alt)
    #pragma config FMIIEN       = OFF                               // MII/RMII select (OFF=RMII)

#endif

    #define CAPABILITIES    (blCapBootLED | blCapUSBInterface | blCapProgramButton | blCapVirtualProgramButton | CAPCOMMON)

    // BTN / LED sense
    #define LedOn       High
    #define BntOn       Low

    // Boot LED
    #define BLedLat     E
    #define BLedBit     0

    // Download LED
    #define DLedLat     A
    #define DLedBit     7

    // Virtual program button
    #define VPBntLat    G
    #define VPBntBit    15

    // Program button
    #define PBntPort    G
    #define PBntBit     15

    // Other capabilities
    //#define LISTEN_BEFORE_LOAD          2000                // no less than 2 seconds
    //#define BOOTLOADER_UART             1                   // avrdude program UART
    //#define BAUDRATE                    115200              // avrdude baudrate

    #define _CPU_NAME_                  "32MX795F512L"
    #define VEND                        vendPontech
    #define PROD                        prodQuick240
    #define F_CPU                       80000000UL
    #define F_PBUS                      F_CPU

    #define FLASH_BYTES                 0x80000                     // 512K
    #define FLASH_PAGE_SIZE             4096
    #define LoadFlashWaitStates()       (CHECON = 2)                // 0 for 0-30Mhz, 1 for 31-60Mhz, 2 for 61-80Mhz
#elif defined(_BOARD_PONTECH_NOFIRE_) // PIC32MZ2048EFG100
#define _CONFIG_VALID_

#if defined(PUT_CONFIG_BITS_HERE)

     //*    Oscillator Settings
    // works with proper timing
    #pragma config POSCMOD      = EC                                // External Clock
//    #pragma config POSCMOD      = HS                                // Crystal

    #pragma config FPLLIDIV     = DIV_3                             // 8 MHz
    #pragma config FPLLICLK     = PLL_POSC                          // 8MHz Posc

    #pragma config FNOSC        = SPLL                              // Oscillator selection
    #pragma config FPLLMULT     = MUL_50                            // 400 MHz
    #pragma config FPLLODIV     = DIV_2                             // 200 MHz

    #pragma config FPLLRNG      = RANGE_5_10_MHZ                    // 5-10Mhz
    #pragma config FSOSCEN      = OFF                               // Secondary oscillator enable
    #pragma config UPLLFSEL     = FREQ_24MHZ                        // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

    //*    Clock control settings
    #pragma config IESO         = ON                                // Internal/external clock switchover
    #pragma config FCKSM        = CSDCMD                            // Clock switching (CSx)/Clock monitor (CMx)
    #pragma config OSCIOFNC     = OFF                               // Clock output on OSCO pin enable

    //*    Other Peripheral Device settings
    #pragma config FWDTEN       = OFF                               // Watchdog timer enable
    #pragma config WDTPS        = PS1048576                         // Watchdog timer postscaler
    #pragma config WDTSPGM      = STOP                              // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
    #pragma config WINDIS       = NORMAL                            // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
    #pragma config FWDTWINSZ    = WINSZ_25                          // Watchdog Timer Window Size (Window size is 25%)
    #pragma config FDMTEN       = OFF                               // Deadman Timer Enable (Deadman Timer is disabled)

    //*    Code Protection settings
    #pragma config CP           = OFF                               // Code protection

    //*    Debug settings
//    #pragma config DEBUG       = ON                               // turn debugging on
    #pragma config ICESEL       = ICS_PGx2                          // ICE pin selection

    #pragma config FETHIO       = ON                                // Standard/alternate ETH pin select (OFF=Alt)
    #pragma config FMIIEN       = OFF                               // MII/RMII select (OFF=RMII)

    #pragma config BOOTISA  = MIPS32

    //*    USB Settings
//    #pragma config UPLLEN       = ON                                // USB PLL enable
    #pragma config FUSBIDIO     = OFF                               // USBID pin control

    #pragma config DMTCNT       = 0

//    #pragma config FDBGWP       = 0

#pragma config PGL1WAY  = OFF             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = OFF             // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY  = OFF             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config DMTINTV  = WIN_127_128     // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config EJTAGBEN = NORMAL
#pragma config DBGPER   = PG_ALL
#pragma config FSLEEP   = OFF
#pragma config FECCCON  = OFF_UNLOCKED
#pragma config TRCEN    = OFF

#endif

    #define CAPABILITIES    (blCapBootLED | blCapDownloadLED | blCapUARTInterface | blCapAutoResetListening | blCapVirtualProgramButton | CAPCOMMON)

    // BTN / LED sense
    #define LedOn       High
    #define BntOn       High

    // Boot LED
    #define BLedLat     G
    #define BLedBit     6

    // Download LED
    #define DLedLat     D
    #define DLedBit     4

    // Virtual program button
    #define VPBntLat    C
    #define VPBntBit    12

    // Other capabilities
    #define LISTEN_BEFORE_LOAD          2000                // no less than 2 seconds
    #define BOOTLOADER_UART             4                   // avrdude program UART
    #define BAUDRATE                    115200              // avrdude baudrate
    #define UARTMapRX()                 (U4RXR = 0b1011)    // RPF2 -> U4RX
    #define UARTMapTX()                 (RPF8R = 0b0010)    // RF8 -> U4TX
    #define JTAG                        1                   // have dedicated JTAG pins, turn on JTAG

    #define _CPU_NAME_                  "PIC32MZ2048EFG"
    #define VEND                        vendPontech
    #define PROD                        prodPontechNoFire
    #define F_CPU                       200000000UL
//    #define F_CPU                       80000000UL
    #define F_PBUS                      (F_CPU / (PB2DIVbits.PBDIV + 1))

    #define FLASH_BYTES                 0x200000                    // 2MB
    #define FLASH_PAGE_SIZE             0x4000                      // 16K
    //#define LoadFlashWaitStates()       (PRECON = 0x32)             // prefetching ON, 2 wait states
#endif

