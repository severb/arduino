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

#include <string.h>
#include <limits.h>

#define BAUD_SLOW 9600
#define BAUD_FAST 230400

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

static bool fastBaud = true;

static inline void setFastBaud() {
  Serial.begin(BAUD_FAST);
  fastBaud = true;
}

static inline void setSlowBaud() {
  Serial.begin(BAUD_SLOW);
  fastBaud = false;
}

static inline bool isFastBaud() {
  return fastBaud;
}

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
    readEnable();
    inCycle = data != readData();
    counter--;
    readDisable();
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
  dataInputMode();
  chipEnable();
  readEnable();
  for (byte i = 0; i < len; i++) {
    setAddr(addr + i);
    if (data[i] != readData()) {
      readDisable();
      chipDisable();
      goto writeBlock;
    }
  }
  readDisable();
  chipDisable();
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
  // send custom data manually entered here
  byte data[] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
    0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
    0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f,
    0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
    0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
    0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47,
    0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f,
    0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
    0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
    0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x66, 0x67,
    0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f,
    0x70, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77,
    0x78, 0x79, 0x7a, 0x7b, 0x7c, 0x7d, 0x7e, 0x7f,
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
  Serial.print(hexTable[(s & 0xf000) >> 12]);
  Serial.print(hexTable[(s & 0x0f00) >> 8]);
  Serial.print(hexTable[(s & 0x00f0) >> 4]);
  Serial.print(hexTable[(s & 0x000f) >> 0]);
}

static void printBlock(unsigned short a, unsigned short b) {
  if (a > MAX_ADDR || b > MAX_ADDR) return;
  unsigned short from = min(a, b);
  unsigned short to = max(a, b);
  dataInputMode();
  chipEnable();
  readEnable();
  for (unsigned short i = from; i <= to; i++) {
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
  Serial.println();
  readDisable();
  chipDisable();
}

static void file(unsigned short s) {
  if (s > MAX_ADDR) return;
  byte buf[BLOCK_SIZE];
  unsigned short i = 0;
  while (i < s) {
    if (Serial.available()) {
      buf[i % BLOCK_SIZE] = Serial.read();
      i++;
      if (i > 0 && i % BLOCK_SIZE == 0) {
        writeBlock(buf, i - BLOCK_SIZE, BLOCK_SIZE);
      }
    }
  }
  if (i % BLOCK_SIZE != 0) {
    writeBlock(buf, i - (i % BLOCK_SIZE), i % BLOCK_SIZE);
  }
}

static char cmd[255];
static char *cur;

static void prompt() {
  cur = &cmd[0];
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

static inline void skipWhite() {
  while ('\0' != *cur && ' ' == *cur)
    cur++;
}

static int readInt() {
  skipWhite();
  if (cur[0] == '0' && cur[1] == 'x') {
    cur += 2;
    int hex = readHex();
    return hex;
  }
  return readDec();
}

static int readHex() {
  static const char *accepted = "0123456789abcdef";
  int r = 0;
  bool overflow = false;
  while (1) {
    bool found = false;
    for (int i = 0; i < 16; i++) {
      if (accepted[i] == *cur) {
        if (r > (INT_MAX - i) / 16) {
          overflow = true; // consume the entire number
        }
        r = r * 16 + i;
        cur++;
        found = true;
        break;
      }
    }
    if (!found) {
      return overflow ? -1 : r;
    }
  }
}

static int readDec() {
  int r = 0;
  bool overflow = false;
  while ('\0' != *cur && '0' <= *cur && *cur <= '9') {
    byte n = *cur - '0';
    if (r > (INT_MAX - n) / 10) {
      overflow = true;
    }
    r = r * 10 + n;
    cur++;
  }
  return overflow ? -1 : r;
}

static bool addr() {
  int addr = readInt();
  if (addr < 0) {
    Serial.println("error: address must be positive");
    return false;
  }
  setAddr(addr);
  return true;
}

static bool data() {
  int data = readInt();
  if (data < 0) {
    Serial.println("error: data must be positive");
    return false;
  }
  if (data > 255) {
    Serial.println("error: data must be smaller than 255");
    return false;
  }
  setData(data);
  return true;
}

static bool match(char const *s) {
  int n = strlen(s);
  bool result = strncmp(s, cur, n) == 0;
  if (result) {
    cur += n;
  }
  return result;
}

#define ADDR ((PORT_ADDR_HI << 8) | PORT_ADDR_LO)

static void execute() {
  skipWhite();
  if (match("read ")) {
    if (!addr()) return;
    Serial.println(readByte(ADDR));
    return;
  }
  if (match("write ")) {
    if (!addr()) return;
    if (!data()) return;
    writeByte(PORT_DATA, ADDR);
    return;
  }
  if (match("block ")) {
    int a = readInt();
    int b = readInt();
    if (a < 0 || b < 0) {
      Serial.println("error: block range cannot be negative");
      return;
    }
    printBlock(a, b);
    return;
  }

  // low level
  if (match("addr")) {
    addr();
    return;
  }
  if (match("data ")) {
    data();
    return;
  }
  if (match("+chip")) {
    chipEnable();
    return;
  }
  if (match("-chip")) {
    chipDisable();
    return;
  }
  if (match("+write")) {
    writeEnable();
    return;
  }
  if (match("-write")) {
    writeDisable();
    return;
  }
  if (match("+read")) {
    readEnable();
    return;
  }
  if (match("-read")) {
    readDisable();
    return;
  }
  if (match("in")) {
    dataInputMode();
    return;
  }
  if (match("out")) {
    dataOutputMode();
    return;
  }
  if (match("get")) {
    Serial.println(readData());
    return;
  }

  // data protection
  if (match("lock!")) {
    lock();
    return;
  }
  if (match("unlock")) {
    unlock();
    return;
  }

  // large block
  if (match("file!")) {
    if (isFastBaud()) {
      Serial.println("error: switch to slow Baud rate");
      return;
    }
    int s = readInt();
    if (s < 0) {
      Serial.println("error: size cannot be negative");
      return;
    }
    skipWhite();
    file(s);
    return;
  }
  if (match("zero!")) {
    zero();
    return;
  }
  if (match("user!")) {
    user();
    return;
  }
  if (match("all")) {
    printBlock(0, MAX_ADDR);
    return;
  }

  // misc
  if (match("slow")) {
    setSlowBaud();
    return;
  }
  if (match("fast")) {
    setFastBaud();
    return;
  }
  if (match("help")) {
    Serial.println("read  <addr>         read a byte");
    Serial.println("write <addr> <data>  write a byte");
    Serial.println("block <start> <stop> print a range of bytes");
    Serial.println();
    Serial.println("-- low level control --");
    Serial.println("addr <addr>  set the address pins");
    Serial.println("data <data>  set the I/O pins");
    Serial.println("[+|-]chip    enable/disable the chip");
    Serial.println("[+|-]write   enable/disable writing");
    Serial.println("[+|-]read    enable/disable reading");
    Serial.println("in           set MEGA's I/O pins to input (i.e., before reading)");
    Serial.println("out          set MEGA's I/O pins to output (i.e., before setting data)");
    Serial.println("get          read the byte at the current address");
    Serial.println();
    Serial.println("-- data protection --");
    Serial.println("unlock  remove software data protection");
    Serial.println("lock!   set software data protection");
    Serial.println();
    Serial.println("-- large block operations --");
    Serial.println("file! <size> load an file in the EEPROM");
    Serial.println("zero!        zero-out the EEPROM");
    Serial.println("user!        write the user data (i.e., your custom code)");
    Serial.println("all          print the entire EEPROM");
    Serial.println();
    Serial.println("-- misc --");
    Serial.println("slow  set Baud rate to 9600 (for data upload)");
    Serial.println("fast  set Baud rate to 230400 (for data download)");
    Serial.println("help  print this help");
    return;
  }
  Serial.println("error: unknown command -- type 'help' for help");
}

static void initPins() {
  chipDisable();
  pinMode(PIN_CE, OUTPUT);
  writeDisable();
  pinMode(PIN_WE, OUTPUT);
  readDisable();
  pinMode(PIN_OE, OUTPUT);

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
  setFastBaud();
  delay(1); // 15ns data write protection at start-up
}

void loop() {
  prompt();
  execute();
}
