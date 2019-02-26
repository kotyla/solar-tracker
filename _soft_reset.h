#define soft_reset() \
do { \
  wdt_enable(WDTO_15MS); \
  for(;;) \
  { \
  } \
} while(0);

// Soft Reset Function Pototype
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));

// Soft Reset Function Implementation
void wdt_init(void) {
  MCUSR = 0;
  wdt_disable();
  return;
}
