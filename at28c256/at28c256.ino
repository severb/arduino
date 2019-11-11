/*
  AT28C256 Programmer for MEGA 2560

  This programmer is not portable because it uses ports to meet the timing
  requirements for Page Mode Write.

  PORT MAPPING

  | ARDUINO | AT28C256 |
  +---------+----------+
  | PA0  22 | 10  A0   |
  | PA1  23 | 9   A1   |
  | PA2  24 | 8   A2   |
  | PA3  25 | 7   A3   |
  | PA4  26 | 6   A4   |
  | PA5  27 | 5   A5   |
  | PA6  28 | 4   A6   |
  | PA7  29 | 3   A7   |
  +---------+----------+
  | PC0  37 | 25  A8   |
  | PC1  36 | 24  A9   |
  | PC2  35 | 21  A10  |
  | PC3  34 | 23  A11  |
  | PC4  33 | 2   A12  |
  | PC5  32 | 26  A13  |
  | PC6  31 | 1   A14  |
  | PC7  30 |          |
  +---------+----------+
  | PL0  49 | 11  IO0  |
  | PL1  48 | 12  IO1  |
  | PL2  47 | 13  IO2  |
  | PL3  46 | 15  IO3  |
  | PL4  45 | 16  IO4  |
  | PL5  44 | 17  IO5  |
  | PL6  43 | 18  IO6  |
  | PL7  42 | 19  IO7  |
  +---------+----------+
  | PG0  41 | 20  CEB  |
  | PG1  40 | 27  WEB  |
  | PG2  39 | 22  OEB  |

*/

#define BAUD_RATE 57600

#define PIN_CE 41
#define PIN_WE 40
#define PIN_OE 39

#define PORT_ADDR_LO PORTA
#define DDR_ADDR_LO DDRA

#define PORT_ADDR_HI PORTC
#define DDR_ADDR_HI DDRC

#define PORT_DATA PORTL
#define PIN_DATA PINL
#define DDR_DATA DDRL

#define BLOCK_SIZE 64
#define BLOCK_COUNT 512
#define MAX_ADDR 32767U

// Chip state invariant: CE, WE and OE are HIGH in between calls

static inline void setAddr(unsigned short addr) {
  if (addr > MAX_ADDR) {
    addr = MAX_ADDR;
  }
  PORT_ADDR_LO = (byte)addr;
  PORT_ADDR_HI &= 0x80;
  PORT_ADDR_HI |= (byte)(addr >> 8);
}

static inline void dataInputMode() {
  DDR_DATA = 0x00;
}

static inline void dataOutputMode() {
  DDR_DATA = 0xff;
}

static inline void setData(byte data) {
  PORT_DATA = data;
}

static inline byte readData() {
  return PIN_DATA;
}

static inline void chipEnable() {
  digitalWrite(PIN_CE, LOW);
}

static inline void chipDisable() {
  digitalWrite(PIN_CE, HIGH);
}

static inline void readEnable() {
  digitalWrite(PIN_OE, LOW);
}

static inline void readDisable() {
  digitalWrite(PIN_OE, HIGH);
}

static inline void writeEnable() {
  //digitalWrite(PIN_WE, LOW);
  // digitalWrite is not fast enough, bit-fiddle the port
  PORTG &= 0xfd; // unset the 2nd bit
}

static inline void writeDisable() {
  //digitalWrite(PIN_WE, HIGH);
  // digitalWrite is not fast enough, bit-fiddle the port
  PORTG |= 0x02; // set the 2nd bit

}

static byte readByte(unsigned short addr) {
  setAddr(addr);
  dataInputMode();
  chipEnable();
  readEnable();
  byte data = readData();
  readDisable();
  chipDisable();
  return data;
}

static bool poll(byte data) {
  setData(0);
  dataInputMode();
  // sginal err if the poll doesn't succeed before the counter goes to 0.
  unsigned int counter = 1 << 12;
  bool inCycle = true;
  chipEnable();
  while (inCycle && counter > 0) {
    readEnable(); // TODO: no need to toggle OE
    inCycle = data != readData();
    readDisable();
    counter--;
  }
  chipDisable();
  return counter > 0;
}

static void writeByte(byte data, unsigned short addr) {
  setAddr(addr);
  dataOutputMode();
  setData(data);
  chipEnable();
  writeEnable();
  writeDisable();
  chipDisable();
  poll(data);
}

static void writeBlock(byte *data, unsigned short addr, byte len) {
  if (len == 0) {
    return;
  }
  if (len > 64) {
    return;
  }
  if (addr > MAX_ADDR - len + 1) {
    return;
  }
  digitalWrite(LED_BUILTIN, HIGH);
writeBlock:
  dataOutputMode();
  chipEnable();
  for (byte i = 0; i < len; i++) {
    setAddr(addr + i);
    setData(data[i]);
    writeEnable();
    writeDisable();
  }
  chipDisable();
  if (!poll(data[len - 1])) {
    // block write failed, retry
    goto writeBlock;
  }
  digitalWrite(LED_BUILTIN, LOW);
}

static void zero() {
  byte block[BLOCK_SIZE] = {0};
  for (unsigned short i = 0; i < BLOCK_COUNT; i++) {
    writeBlock(block, i * sizeof(block), sizeof(block));
  }
}

static void unlock() {
  dataOutputMode();
  chipEnable();

  setAddr(0x5555); setData(0xaa);
  writeEnable(); writeDisable();

  setAddr(0x2aaa); setData(0x55);
  writeEnable(); writeDisable();

  setAddr(0x5555); setData(0x80);
  writeEnable(); writeDisable();

  setAddr(0x5555); setData(0xaa);
  writeEnable(); writeDisable();

  setAddr(0x2aaa); setData(0x55);
  writeEnable(); writeDisable();

  setAddr(0x5555); setData(0x20);
  writeEnable(); writeDisable();

  chipDisable();
  delay(10); // don't know how to poll here, wait instead
}

static void lock() {
  dataOutputMode();
  chipEnable();

  setAddr(0x5555); setData(0xaa);
  writeEnable(); writeDisable();

  setAddr(0x2aaa); setData(0x55);
  writeEnable(); writeDisable();

  setAddr(0x5555); setData(0xa0);
  writeEnable(); writeDisable();

  chipDisable();
  delay(10); // don't know how to poll here, wait instead
}

static void user() {
  byte data[] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x0b, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x0d, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x0e, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
    0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
  };
  for (unsigned short i = 0; i < sizeof(data); i += BLOCK_SIZE) {
    writeBlock(&data[i], i, min(BLOCK_SIZE, sizeof(data) - i));
  }
}

static char const *hexTable = "0123456789abcdef";

static void printHexByte(byte b) {
  Serial.print("0x");
  Serial.print(hexTable[b >> 4]);
  Serial.print(hexTable[b & 0x0f]);
}

static void printHexShort(unsigned short s) {
  Serial.print("0x");
  Serial.print(hexTable[(byte)(s & 0xf000) >> 12]);
  Serial.print(hexTable[(byte)(s & 0x0f00) >> 8]);
  Serial.print(hexTable[(byte)(s & 0x00f0) >> 4]);
  Serial.print(hexTable[(byte)(s & 0x000f) >> 0]);
}

static void readall() {
  dataInputMode();
  chipEnable();
  readEnable();
  for (unsigned short i = 0; i < MAX_ADDR; i++) {
    if (i % 16 == 0) {
      printHexShort(i);
      Serial.print(": ");
    }
    setAddr(i);
    byte data = readData();
    printHexByte(data);
    Serial.print(" ");
    if (i && (i + 1) % 16 == 0) {
      Serial.println();
    }
  }
  readDisable();
  chipDisable();
}

static char cmd[255];

static void prompt() {
  byte n = 0;
top:
  n = 0;
  Serial.print("> ");
  while (n < 255) {
    while (Serial.available()) {
      int c = Serial.read();
      if (c == '\n') {
        cmd[n] = '\0';
        return;
      }
      cmd[n++] = (char)c;
    }
    delay(1);
  }
  // we failed to find a \n before running out of space
  while (1) {
    while (Serial.available()) {
      if (Serial.read() == '\n') {
        Serial.println("error: command too long");
        goto top;
      }
    }
    delay(1);
  }
}

static void execute() {
  String s = String(cmd);
  s.trim();
  if (s.startsWith("addr ")) {
    int addr = s.substring(5).toInt();
    if (addr < 0) {
      Serial.println("error: address must be positive");
      return;
    }
    setAddr(addr);
    return;
  }
  if (s.startsWith("set ")) {
    int data = s.substring(4).toInt();
    if (data < 0) {
      Serial.println("error: data must be positive");
      return;
    }
    if (data > 255) {
      Serial.println("error: data must be smaller than 255");
      return;
    }
    setData(data);
    return;
  }
  if (s.equals("+chip")) {
    chipEnable();
    return;
  }
  if (s.equals("+write")) {
    writeEnable();
    return;
  }
  if (s.equals("+read")) {
    readEnable();
    return;
  }
  if (s.equals("-chip")) {
    chipDisable();
    return;
  }
  if (s.equals("-write")) {
    writeDisable();
    return;
  }
  if (s.equals("-read")) {
    readDisable();
    return;
  }
  if (s.equals("in")) {
    dataInputMode();
    return;
  }
  if (s.equals("out")) {
    dataOutputMode();
    return;
  }
  if (s.equals("read")) {
    Serial.println(readData());
    return;
  }
  if (s.equals("zero!")) {
    zero();
    return;
  }
  if (s.equals("unlock")) {
    unlock();
    return;
  }
  if (s.equals("lock!")) {
    lock();
    return;
  }
  if (s.equals("user!")) {
    user();
    return;
  }
  if (s.equals("readall")) {
    readall();
    return;
  }
  if (s.equals("help")) {
    Serial.println("addr <addr> sets the address pins");
    Serial.println("set <data>  sets the I/O pins");
    Serial.println("[+|-]chip   enable/disable the chip");
    Serial.println("[+|-]write  enable/disable writing");
    Serial.println("[+|-]read   enable/disable reading");
    Serial.println("in          set MEGA's I/O pins to input (i.e., before reading)");
    Serial.println("out         set MEGA's I/O pins to output (i.e., before setting data)");
    Serial.println("read        read the byte at the current address");
    Serial.println("readall     read the entire EEPROM");
    Serial.println("zero!       zero-out the EEPROM");
    Serial.println("unlock      remove software data protection");
    Serial.println("lock!       set software data protection");
    Serial.println("user!       write the user data (i.e., your custom code)");
    Serial.println("help        print this help");
    return;
  }
  Serial.println("error: unknown command -- type 'help' for help");
}

static void initPins() {
  pinMode(PIN_CE, OUTPUT);
  pinMode(PIN_WE, OUTPUT);
  pinMode(PIN_OE, OUTPUT);
  chipDisable();
  writeDisable();
  readDisable();

  DDR_ADDR_HI |= 0x7f; // set all (bust most significant) address bits bit to output
  DDR_ADDR_LO = 0xff;  // all low address bits to output

  // init registers to known state
  dataOutputMode();
  setAddr(0);
  setData(0);

  // disable the bulit-in LED
  pinMode(LED_BUILTIN, OUTPUT);
}

void setup() {
  initPins();
  Serial.begin(BAUD_RATE);
  delay(1); // 15ns data write protection at start-up
}

void loop() {
  prompt();
  execute();
}
