/***** BlackPill F411 + SIM7600/A7682 + RS485
 * Mục đích:
 *   - STM32 là Modbus RTU slave ID = 1.
 *   - PLC gửi dữ liệu cân qua RS485 bằng Modbus.
 *   - STM32 lưu dữ liệu vào Holding Register HR[].
 *   - Nếu dữ liệu thay đổi thì STM32 publish JSON lên MQTT bằng SIM7600/A7682.
 *
 * Có 2 cân:
 *   - NLDEMO dùng register base 1, 3, 5.
 *   - NLD2   dùng register base 7, 9, 11.
 *
 * PLC ghi bằng FC16, qty = 2 register mỗi giá trị.
 * Ví dụ ghi KL của NLDEMO:
 *   addr = 1, qty = 2 -> ghi HR[1] và HR[2].
 *
 * Map thanh ghi:
 *   NLDEMO:
 *     KL   @ HR[1..2]
 *     SL   @ HR[3..4]
 *     TONG @ HR[5..6]
 *
 *   NLD2:
 *     KL   @ HR[7..8]
 *     SL   @ HR[9..10]
 *     TONG @ HR[11..12]
 *****************************************************************************/

#include <Arduino.h>           // Hàm Arduino cơ bản: pinMode, digitalWrite, millis, Serial...
#include <TinyGsmClient.h>     // Điều khiển modem SIM7600/A7682.
#include <ArduinoMqttClient.h> // Gửi MQTT.
#include <math.h>              // Dùng isfinite() để kiểm tra số float hợp lệ.
#include <string.h>            // Dùng memset() để xóa mảng.
#include <limits.h>            // Dùng INT32_MIN.
#include "secrets.h"           // Thông tin APN/MQTT cục bộ, không commit lên GitHub.

// Nếu chưa define modem SIM7600 ở platformio.ini thì define tại đây.
#ifndef TINY_GSM_MODEM_SIM7600
#define TINY_GSM_MODEM_SIM7600
#endif

/* ==== Pins ==== */
#define RS485_TX   PA2    // TX UART2 -> DI module RS485.
#define RS485_RX   PA3    // RX UART2 <- RO module RS485.
#define RS485_DE   PB10   // Điều khiển RS485: LOW = nhận, HIGH = gửi.
#define SIM_TX     PB6    // TX UART1 -> RX module SIM.
#define SIM_RX     PB7    // RX UART1 <- TX module SIM.

HardwareSerial MODEM(USART1);   // UART1 giao tiếp SIM7600/A7682.
HardwareSerial BUS(USART2);     // UART2 giao tiếp RS485/Modbus.

/* ==== RS485 ==== */
static const uint32_t MB_BAUD = 9600;       // Baudrate Modbus RTU.
static const uint32_t MB_CFG  = SERIAL_8N1; // 8 data bit, không parity, 1 stop bit.

static inline void RS485_RXMode(){ digitalWrite(RS485_DE, LOW); }  // Chuyển RS485 sang chế độ nhận.
static inline void RS485_TXMode(){ digitalWrite(RS485_DE, HIGH); } // Chuyển RS485 sang chế độ gửi.
static inline void RS485_TXFlush(){
  BUS.flush(); // Chờ UART gửi xong byte cuối.
  delayMicroseconds((11UL * 1000000UL) / MB_BAUD * 2); // Đợi thêm 2 ký tự để tránh cắt đuôi frame.
}

/* ==== APN / MQTT ==== */
static const char* APN               = CELLULAR_APN;
static const char* MQTT_HOST         = MQTT_BROKER_HOST;
static const uint16_t MQTT_PORT      = MQTT_BROKER_PORT;
static const char* MQTT_USER         = MQTT_BROKER_USER;
static const char* MQTT_PASS         = MQTT_BROKER_PASSWORD;
static const char* MQTT_TOPIC_PREFIX = MQTT_BASE_TOPIC;

TinyGsm       modem(MODEM); // Object điều khiển modem qua UART MODEM.
TinyGsmClient net(modem);   // TCP client đi qua mạng SIM.
MqttClient    mqtt(net);    // MQTT client chạy trên TCP client.

/* ==== Modbus map ==== */
static const uint8_t  SLAVE_ID = 1;   // ID Modbus slave của STM32.
static const uint16_t REGS_MAX = 256; // Số Holding Register giả lập.

/* ---- NLDEMO ---- */
static const uint16_t REG1_KL_BASE   = 1;   // NLDEMO KL   nằm ở HR[1],  HR[2].
static const uint16_t REG1_SL_BASE   = 3;   // NLDEMO SL   nằm ở HR[3],  HR[4].
static const uint16_t REG1_TONG_BASE = 5;   // NLDEMO TONG nằm ở HR[5],  HR[6].

/* ---- NLD2 ---- */
static const uint16_t REG2_KL_BASE   = 7;   // NLD2 KL   nằm ở HR[7],  HR[8].
static const uint16_t REG2_SL_BASE   = 9;   // NLD2 SL   nằm ở HR[9],  HR[10].
static const uint16_t REG2_TONG_BASE = 11;  // NLD2 TONG nằm ở HR[11], HR[12].

volatile uint16_t HR[REGS_MAX]; // Mảng Holding Register. PLC ghi dữ liệu vào mảng này.

/* ==== State ==== */
static char ts_buf[32];          // Lưu timestamp. Ví dụ: 2026-05-11T10:30:00Z.
static char json_buf[384];       // Lưu payload JSON trước khi publish MQTT.
static unsigned long msg_id = 0; // Số thứ tự message, mỗi lần publish tăng 1.
static String g_cid;             // MQTT client ID. Ví dụ: F411-12345678.

// Gom 3 giá trị của 1 cân.
struct Snap3 {
  int32_t kl;   // Khối lượng. Ví dụ 12345 sẽ gửi MQTT thành 123.45.
  int32_t sl;   // Số lượng.
  int32_t tong; // Tổng khối lượng. Ví dụ 50000 sẽ gửi MQTT thành 500.00.
};

// Mô tả 1 cân: mã cân, địa chỉ register, giá trị cũ, và có cần publish không.
struct ScaleSlot {
  const char* code;       // Mã cân. Ví dụ: NLDEMO / NLD2.
  uint16_t reg_kl_base;   // Địa chỉ register bắt đầu của KL.
  uint16_t reg_sl_base;   // Địa chỉ register bắt đầu của SL.
  uint16_t reg_tong_base; // Địa chỉ register bắt đầu của TONG.
  Snap3 last;             // Giá trị đã publish lần trước.
  bool dirty;             // true = dữ liệu mới, cần publish.
};

// Danh sách 2 cân cần xử lý.
static ScaleSlot scales[] = {
  { "NLDEMO", REG1_KL_BASE, REG1_SL_BASE, REG1_TONG_BASE,
    { INT32_MIN, INT32_MIN, INT32_MIN }, true }, // Ép publish lần đầu vì last là giá trị đặc biệt.

  { "NLD2",   REG2_KL_BASE, REG2_SL_BASE, REG2_TONG_BASE,
    { INT32_MIN, INT32_MIN, INT32_MIN }, true }  // dirty=true để khởi động xong là gửi ngay.
};

static const size_t SCALE_COUNT = sizeof(scales) / sizeof(scales[0]); // Số lượng cân trong mảng scales.

/* ==== CRC16 Modbus ==== */
// Tính CRC16 Modbus RTU cho frame.
// Ví dụ frame gửi đi phải có 2 byte CRC ở cuối: CRC low byte trước, high byte sau.
static uint16_t mb_crc16(const uint8_t* p, size_t n){
  uint16_t crc = 0xFFFF; // Giá trị khởi tạo chuẩn Modbus.
  for(size_t i = 0; i < n; i++){
    crc ^= p[i]; // XOR từng byte vào CRC.
    for(uint8_t b = 0; b < 8; b++){
      crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : (crc >> 1); // Tính từng bit.
    }
  }
  return crc;
}

/* ==== RX by gap ==== */
static const uint16_t FRAME_GAP_MS = 5; // Nếu im lặng > 5ms thì xem như hết 1 frame Modbus.
static uint8_t  rxBuf[260];            // Buffer chứa frame Modbus nhận được.
static size_t   rxLen = 0;             // Số byte đang có trong rxBuf.
static uint32_t lastRx = 0;            // Thời điểm nhận byte cuối.

/* ==== Helpers RS485 ==== */
// Gửi 1 mảng byte ra RS485.
static void sendRS485(const uint8_t* p, size_t n){
  RS485_TXMode();   // Bật chế độ gửi.
  BUS.write(p, n);  // Gửi n byte.
  RS485_TXFlush();  // Đợi gửi xong.
  RS485_RXMode();   // Quay về chế độ nhận.
}

// Trả lời lỗi Modbus.
// Ví dụ code 0x02 = illegal data address, code 0x03 = illegal data value.
static void mb_exception(uint8_t slave, uint8_t fc, uint8_t code){
  uint8_t rsp[5] = { slave, (uint8_t)(0x80 | fc), code, 0, 0 }; // Function lỗi = fc | 0x80.
  uint16_t c = mb_crc16(rsp, 3); // Tính CRC cho 3 byte đầu.
  rsp[3] = c & 0xFF;             // CRC low byte.
  rsp[4] = c >> 8;               // CRC high byte.
  sendRS485(rsp, 5);             // Gửi exception response.
}

/* ==== Giải mã 32-bit từ 2 thanh ghi ==== */
// PLC ghi 2 register 16-bit, code ghép lại thành int32_t.
// Thứ tự đang dùng: reg1 = word thấp, reg2 = word cao.
// Ví dụ muốn ra 12345 decimal = 0x00003039:
//   reg1 = 0x3039, reg2 = 0x0000.
static inline int32_t s32_from_regs(uint16_t reg1, uint16_t reg2){
  uint8_t C = (uint8_t)(reg1 >> 8);   // Byte cao của reg1.
  uint8_t D = (uint8_t)(reg1 & 0xFF); // Byte thấp của reg1.
  uint8_t A = (uint8_t)(reg2 >> 8);   // Byte cao của reg2.
  uint8_t B = (uint8_t)(reg2 & 0xFF); // Byte thấp của reg2.

  uint32_t u = ((uint32_t)A << 24) | // Ghép thành 32-bit: A B C D.
               ((uint32_t)B << 16) |
               ((uint32_t)C <<  8) |
               (uint32_t)D;
  return (int32_t)u; // Ép về signed 32-bit.
}

/* ==== Modem / MQTT / Time ==== */
// Tắt các chế độ ngủ của modem để kết nối ổn định hơn.
static inline void tuneModem(){
  modem.sendAT("+CSCLK=0");  modem.waitResponse(); // Tắt sleep clock.
  modem.sendAT("+CPSMS=0");  modem.waitResponse(); // Tắt power saving mode.
  modem.sendAT("+CEDRXS=0"); modem.waitResponse(); // Tắt eDRX.
}

// Khởi động modem, chờ vào mạng, kết nối GPRS.
static bool connectCell(){
  MODEM.setTx(SIM_TX);              // Gán chân TX cho UART modem.
  MODEM.setRx(SIM_RX);              // Gán chân RX cho UART modem.
  MODEM.begin(115200, SERIAL_8N1);  // Baudrate giao tiếp với SIM.
  delay(300);                       // Đợi UART/modem ổn định.

  if (!modem.restart()) modem.init(); // Restart modem, nếu fail thì init.
  tuneModem();                        // Tắt sleep modem.

  if (!modem.waitForNetwork(180000L)) { // Chờ đăng ký mạng tối đa 180 giây.
    return false;
  }
  if (!modem.gprsConnect(APN, "", "")) { // Kết nối data bằng APN.
    return false;
  }

  String imei = modem.getIMEI(); // Lấy IMEI để tạo client ID riêng.
  if (imei.length() < 6) imei = String((uint32_t)millis(), HEX); // Nếu không lấy được IMEI thì dùng millis.

  g_cid = "F411-" + imei.substring(imei.length() - 8); // Ví dụ: F411-12345678.
  return true;
}

// Kết nối MQTT server.
static bool connectMQTT(){
  mqtt.setId(g_cid.c_str());                    // Set MQTT client ID.
  mqtt.setUsernamePassword(MQTT_USER, MQTT_PASS);// Set user/password.
  mqtt.setKeepAliveInterval(30);                // Gửi keepalive mỗi 30 giây.
  mqtt.setCleanSession(true);                   // Mỗi lần connect là session mới.

  if (!mqtt.connect(MQTT_HOST, MQTT_PORT)) {    // Kết nối server MQTT.
    return false;
  }
  return true;
}

// Đảm bảo SIM và MQTT còn kết nối; mất thì tự nối lại.
static bool ensureLinks(){
  if (!modem.isNetworkConnected() || !modem.isGprsConnected()){
    if (!modem.gprsConnect(APN, "", "")) return false; // Nối lại GPRS nếu mất.
  }
  if (!mqtt.connected()){
    if (!connectMQTT()) return false; // Nối lại MQTT nếu mất.
  }
  return true;
}

// Đồng bộ giờ modem qua NTP.
static void tryNtpSync(){
  modem.sendAT("+CNTP=\"pool.ntp.org\",0"); // Cài NTP server.
  modem.waitResponse(2000);
  modem.sendAT("+CNTP");                    // Bắt đầu sync NTP.
  modem.waitResponse(10000);
}

// Đọc giờ từ modem và đổi về UTC dạng ISO.
// Output ví dụ: 2026-05-11T10:30:00Z.
static bool getClockUTC_Z(char* out, size_t outSz){
  modem.sendAT("+CCLK?"); // Hỏi giờ hiện tại của modem.
  if (modem.waitResponse(1000L, "+CCLK:") != 1){
    modem.waitResponse();
    return false;
  }

  String s = modem.stream.readStringUntil('\n'); // Đọc dòng trả về của +CCLK.
  modem.waitResponse();
  s.trim();

  int q1 = s.indexOf('"');       // Tìm dấu quote đầu.
  int q2 = s.lastIndexOf('"');   // Tìm dấu quote cuối.
  if (q1 >= 0 && q2 > q1) s = s.substring(q1 + 1, q2); // Lấy phần thời gian bên trong quote.
  if (s.length() < 17) return false; // Chuỗi quá ngắn thì xem là lỗi.

  int yy = 2000 + s.substring(0, 2).toInt();  // Năm: "26" -> 2026.
  int MM = s.substring(3, 5).toInt();         // Tháng.
  int dd = s.substring(6, 8).toInt();         // Ngày.
  int hh = s.substring(9, 11).toInt();        // Giờ.
  int mm = s.substring(12, 14).toInt();       // Phút.
  int ss = s.substring(15, 17).toInt();       // Giây.
  if (yy < 2020) return false;                // Năm quá cũ thì coi như modem chưa có giờ đúng.

  int sign = (s.length() >= 20 && s.charAt(17) == '-') ? -1 : +1; // Dấu timezone.
  int zz   = (s.length() >= 20) ? s.substring(18).toInt() : 0;    // Timezone theo đơn vị 15 phút.
  long offMin = sign * (long)zz * 15L;                            // Đổi timezone ra phút.

  long totMin = (long)hh * 60L + mm - offMin; // Đổi giờ local modem về UTC.
  long carry = 0;                             // Nếu đổi UTC bị qua ngày trước/sau.

  while (totMin < 0)      { totMin += 24 * 60; carry--; } // Lùi ngày.
  while (totMin >= 24*60) { totMin -= 24 * 60; carry++; } // Sang ngày.

  hh = totMin / 60; // Giờ UTC.
  mm = totMin % 60; // Phút UTC.

  // Hàm trả về số ngày trong tháng, có tính năm nhuận.
  auto dim = [](int Y, int M)->int{
    const int d[12] = {31,28,31,30,31,30,31,31,30,31,30,31};
    bool leap = ((Y % 4 == 0 && Y % 100 != 0) || (Y % 400 == 0));
    return (M == 2 && leap) ? 29 : d[M - 1];
  };

  // Nếu đổi timezone làm lệch ngày thì sửa ngày/tháng/năm.
  if (carry){
    dd += carry;
    while (dd < 1){
      MM--;
      if (MM < 1){ MM = 12; yy--; }
      dd += dim(yy, MM);
    }
    while (dd > dim(yy, MM)){
      dd -= dim(yy, MM);
      MM++;
      if (MM > 12){ MM = 1; yy++; }
    }
  }

  snprintf(out, outSz, "%04d-%02d-%02dT%02d:%02d:%02dZ", yy, MM, dd, hh, mm, ss); // Format ISO UTC.
  return true;
}

// Lấy timestamp. Nếu modem chưa có giờ thì sync NTP, vẫn fail thì dùng mốc 1970.
static void ensureTimeOrFallback(){
  if (getClockUTC_Z(ts_buf, sizeof(ts_buf))) return;
  tryNtpSync();
  if (!getClockUTC_Z(ts_buf, sizeof(ts_buf))){
    snprintf(ts_buf, sizeof(ts_buf), "1970-01-01T00:00:00Z");
  }
}

/* ==== Utils ==== */
// Đổi float thành chuỗi.
// Ví dụ: f2s(12.345, out, 2) -> "12.35".
static inline void f2s(float v, char* out, int prec = 2){
  dtostrf(v, 0, prec, out);
}

// Đọc 3 giá trị KL/SL/TONG của 1 cân từ HR[].
static Snap3 readScaleRegs(const ScaleSlot& s){
  Snap3 x;
  x.kl   = s32_from_regs(HR[s.reg_kl_base],   HR[s.reg_kl_base + 1]);   // Đọc KL từ 2 register.
  x.sl   = s32_from_regs(HR[s.reg_sl_base],   HR[s.reg_sl_base + 1]);   // Đọc SL từ 2 register.
  x.tong = s32_from_regs(HR[s.reg_tong_base], HR[s.reg_tong_base + 1]); // Đọc TONG từ 2 register.
  return x;
}

// So sánh dữ liệu hiện tại với lần publish trước.
// Nếu khác thì đánh dấu dirty để loop() publish lên MQTT.
static void updateDirtyFlags(){
  for (size_t i = 0; i < SCALE_COUNT; i++){
    Snap3 cur = readScaleRegs(scales[i]); // Đọc giá trị hiện tại của cân i.
    if (cur.kl   != scales[i].last.kl ||
        cur.sl   != scales[i].last.sl ||
        cur.tong != scales[i].last.tong){
      scales[i].dirty = true; // Có thay đổi, cần gửi MQTT.
    }
  }
}

/* ==== Chỉ cho phép ghi tại 1,3,5,7,9,11 ==== */
// Bảo vệ register: PLC chỉ được ghi đúng các base hợp lệ và mỗi lần ghi đúng 2 register.
static inline bool isWriteRangeAllowed(uint16_t addr, uint16_t qty){
  if (qty != 2) return false; // Mỗi giá trị 32-bit bắt buộc gồm 2 register.

  return (addr == REG1_KL_BASE)   || // Cho ghi KL NLDEMO.
         (addr == REG1_SL_BASE)   || // Cho ghi SL NLDEMO.
         (addr == REG1_TONG_BASE) || // Cho ghi TONG NLDEMO.
         (addr == REG2_KL_BASE)   || // Cho ghi KL NLD2.
         (addr == REG2_SL_BASE)   || // Cho ghi SL NLD2.
         (addr == REG2_TONG_BASE);   // Cho ghi TONG NLD2.
}

/* ==== Publish 1 cân ==== */
// Đọc dữ liệu của 1 cân, tạo JSON, gửi lên MQTT.
static void publishScale(ScaleSlot& s){
  ensureTimeOrFallback(); // Lấy timestamp để gắn vào JSON.

  Snap3 cur = readScaleRegs(s); // Đọc KL/SL/TONG hiện tại từ HR[].

  float kl_f   = cur.kl   / 100.0f; // Đổi KL từ số nguyên sang số thập phân. 12345 -> 123.45.
  int   sl_i   = (int)cur.sl;       // SL giữ dạng số nguyên.
  float tong_f = cur.tong / 100.0f; // Đổi TONG. 50000 -> 500.00.

  s.last = cur;      // Lưu lại giá trị đã gửi.
  s.dirty = false;   // Gửi xong thì hết dirty.

  char s_kl[20], s_tong[20]; // Chuỗi KL và TONG để đưa vào JSON.
  f2s(isfinite(kl_f)   ? kl_f   : 0.0f, s_kl,   2); // Nếu KL lỗi thì gửi 0.00.
  f2s(isfinite(tong_f) ? tong_f : 0.0f, s_tong, 2); // Nếu TONG lỗi thì gửi 0.00.

  ++msg_id; // Tăng số thứ tự bản tin.

  char topic[64];
  snprintf(topic, sizeof(topic), "%s%s/", MQTT_TOPIC_PREFIX, s.code); // Ví dụ: weight/push/NLDEMO/

  int n = snprintf(
    json_buf, sizeof(json_buf),
    "{"
      "\"weight_code\":\"%s\","    // Mã cân.
      "\"timestamp\":\"%s\","      // Thời gian UTC.
      "\"msg_id\":%lu,"            // Số thứ tự message.
      "\"PLC1_KL_CAN_1\":%s,"      // Khối lượng hiện tại.
      "\"PLC1_SOLUONG_1\":%d,"     // Số lượng.
      "\"PLC1_TONG_KL_1\":%s,"     // Tổng khối lượng.
      "\"value\":%s"               // Giá trị chính, đang lấy bằng KL.
    "}",
    s.code,
    ts_buf,
    msg_id,
    s_kl,
    sl_i,
    s_tong,
    s_kl
  );

  mqtt.beginMessage(topic, n, true, 1);        // Bắt đầu gửi MQTT: retained=true, QoS=1.
  mqtt.write((const uint8_t*)json_buf, n);     // Ghi nội dung JSON.
  mqtt.endMessage();                           // Kết thúc và publish.
}

/* ==== Modbus core ==== */
// Xử lý 1 frame Modbus vừa nhận được.
static void processFrame(const uint8_t* f, size_t n){
  if (n < 5) return;          // Frame ngắn quá thì bỏ.
  if (f[0] != SLAVE_ID) return; // Không phải ID của mình thì bỏ.

  uint16_t crc_rx   = (uint16_t)f[n - 2] | ((uint16_t)f[n - 1] << 8); // CRC nhận được từ frame.
  uint16_t crc_calc = mb_crc16(f, n - 2);                             // CRC tự tính lại.
  if (crc_rx != crc_calc){
    return;
  }

  uint8_t fc = f[1]; // Function code Modbus.

  /* FC16: Write Multiple Registers */
  // PLC ghi nhiều register. Code này chỉ chấp nhận qty = 2.
  if (fc == 0x10 && n >= 9){
    uint16_t addr = (f[2] << 8) | f[3]; // Địa chỉ register bắt đầu.
    uint16_t qty  = (f[4] << 8) | f[5]; // Số register cần ghi.
    uint8_t  bc   = f[6];               // Byte count = qty * 2.

    if (qty == 0 || bc != qty * 2){
      mb_exception(SLAVE_ID, fc, 0x03); // Sai giá trị request.
      return;
    }

    if (!isWriteRangeAllowed(addr, qty)){
      mb_exception(SLAVE_ID, fc, 0x02); // Địa chỉ không được phép ghi.
      return;
    }

    if (addr + qty > REGS_MAX){
      mb_exception(SLAVE_ID, fc, 0x02); // Vượt quá mảng HR[].
      return;
    }

    for (uint16_t i = 0; i < qty; i++){
      uint8_t hi = f[7 + 2 * i];       // Byte cao của register.
      uint8_t lo = f[7 + 2 * i + 1];   // Byte thấp của register.
      HR[addr + i] = ((uint16_t)hi << 8) | lo; // Ghi vào HR[].
    }

    updateDirtyFlags(); // Kiểm tra cân nào có dữ liệu thay đổi.

    uint8_t rsp[8] = { f[0], 0x10, f[2], f[3], f[4], f[5], 0, 0 }; // Response FC16 echo addr + qty.
    uint16_t c = mb_crc16(rsp, 6);
    rsp[6] = c & 0xFF; // CRC low byte.
    rsp[7] = c >> 8;   // CRC high byte.
    sendRS485(rsp, 8); // Gửi ACK về PLC.
    return;
  }

  /* FC03: debug đọc 1..12 */
  // Cho phép master đọc HR[1..12] để test/debug.
  if (fc == 0x03 && n == 8){
    uint16_t addr = (f[2] << 8) | f[3]; // Địa chỉ bắt đầu đọc.
    uint16_t qty  = (f[4] << 8) | f[5]; // Số register muốn đọc.

    if (qty == 0 || qty > 12 || addr < 1 || (addr + qty) > 13 || (addr + qty) > REGS_MAX){
      mb_exception(SLAVE_ID, fc, 0x02); // Chỉ cho đọc trong vùng HR[1..12].
      return;
    }

    uint8_t rsp[5 + 2 * qty]; // Response gồm: slave, fc, bytecount, data, crc.
    rsp[0] = f[0];
    rsp[1] = 0x03;
    rsp[2] = (uint8_t)(2 * qty); // Số byte data trả về.

    for (uint16_t i = 0; i < qty; i++){
      uint16_t v = HR[addr + i];             // Lấy giá trị register.
      rsp[3 + 2 * i]     = (uint8_t)(v >> 8);   // Byte cao.
      rsp[3 + 2 * i + 1] = (uint8_t)(v & 0xFF); // Byte thấp.
    }

    uint16_t c = mb_crc16(rsp, 3 + 2 * qty); // Tính CRC cho response.

    RS485_TXMode();
    BUS.write(rsp, 3 + 2 * qty); // Gửi phần header + data.
    BUS.write(c & 0xFF);         // Gửi CRC low byte.
    BUS.write(c >> 8);           // Gửi CRC high byte.
    RS485_TXFlush();
    RS485_RXMode();
    return;
  }

  /* FC06: không dùng */
  if (fc == 0x06){
    mb_exception(SLAVE_ID, fc, 0x01); // 0x01 = illegal function.
    return;
  }
}

/* ==== setup / loop ==== */
void setup(){
  pinMode(RS485_DE, OUTPUT); // Chân DE điều khiển hướng RS485.
  RS485_RXMode();            // Mặc định ở chế độ nhận.

  BUS.setTx(RS485_TX);       // Gán TX cho UART RS485.
  BUS.setRx(RS485_RX);       // Gán RX cho UART RS485.
  BUS.begin(MB_BAUD, MB_CFG);// Mở UART RS485 9600 8N1.

  memset((void*)HR, 0, sizeof(HR)); // Xóa tất cả Holding Register về 0.

  // Khởi tạo register của NLDEMO về 0.
  HR[REG1_KL_BASE]       = 0;
  HR[REG1_KL_BASE + 1]   = 0;
  HR[REG1_SL_BASE]       = 0;
  HR[REG1_SL_BASE + 1]   = 0;
  HR[REG1_TONG_BASE]     = 0;
  HR[REG1_TONG_BASE + 1] = 0;

  // Khởi tạo register của NLD2 về 0.
  HR[REG2_KL_BASE]       = 0;
  HR[REG2_KL_BASE + 1]   = 0;
  HR[REG2_SL_BASE]       = 0;
  HR[REG2_SL_BASE + 1]   = 0;
  HR[REG2_TONG_BASE]     = 0;
  HR[REG2_TONG_BASE + 1] = 0;

  while (!connectCell()){
    delay(2000); // Nếu chưa vào mạng thì thử lại sau 2 giây.
  }

  while (!connectMQTT()){
    delay(2000); // Nếu chưa vào MQTT thì thử lại sau 2 giây.
  }

  // Publish lần đầu cả 2 cân, dù dữ liệu đang là 0.
  for (size_t i = 0; i < SCALE_COUNT; i++){
    publishScale(scales[i]);
  }
}

void loop(){
  /* Nhận Modbus theo gap */
  // Đọc tất cả byte đang có trong UART RS485.
  while (BUS.available()){
    int c = BUS.read(); // Đọc 1 byte.
    if (c < 0) break;

    if (rxLen < sizeof(rxBuf)){
      rxBuf[rxLen++] = (uint8_t)c; // Lưu byte vào buffer frame.
    }
    lastRx = millis(); // Cập nhật thời điểm nhận byte cuối.
  }

  // Nếu đã có byte và im lặng quá FRAME_GAP_MS thì xem như hết frame.
  if (rxLen && (millis() - lastRx > FRAME_GAP_MS)){
    processFrame(rxBuf, rxLen); // Xử lý frame Modbus.
    rxLen = 0;                  // Xóa buffer để nhận frame mới.
  }

  if (!ensureLinks()){
    delay(500); // Mất mạng/MQTT thì đợi 500ms rồi loop lại.
    return;
  }

  mqtt.poll(); // Duy trì kết nối MQTT, xử lý keepalive.

  // Chỉ publish cân nào có dữ liệu thay đổi.
  for (size_t i = 0; i < SCALE_COUNT; i++){
    if (scales[i].dirty){
      publishScale(scales[i]); // Gửi MQTT cho cân bị dirty.
    }
  }
}
