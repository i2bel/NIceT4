/*
  Nice BusT4
  Обмен данными по UART на скорости 19200 8n1
  Перед пакетом с данными отправляется break длительностью 519us (10 бит)
  Содержимое пакета, которое удалось понять, описано в структуре packet_cmd_body_t

 

  Для Oview к адресу всегда прибавляется 80.
  Адрес контроллера ворот без изменений.


Подключение

BusT4                       ESP8266

Стенка устройства        Rx Tx GND
9  7  5  3  1  
10 8  6  4  2
место для кабеля
            1 ---------- Rx
            2 ---------- GND
            4 ---------- Tx
            5 ---------- +24V




Из мануала nice_dmbm_integration_protocol.pdf

• ADR: это адрес сети NICE, где находятся устройства, которыми вы хотите управлять. Это может быть значение от 1 до 63 (от 1 до 3F).
Это значение должно быть в HEX. Если адресатом является модуль интеграции на DIN-BAR, это значение равно 0 (adr = 0), если адресат
является интеллектуальным двигателем, это значение равно 1 (adr = 1).
• EPT: это адрес двигателя Nice, входящего в сетевой ADR. Это может быть значение от 1 до 127. Это значение должно быть в HEX.
• CMD: это команда, которую вы хотите отправить по назначению (ADR, EPT).
• PRF: команда установки профиля.
• FNC: это функция, которую вы хотите отправить по назначению (ADR, EPT).
• EVT: это событие, которое отправляется в пункт назначения (ADR, EPT).



*/

#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"
#include <HardwareSerial.h>
#include "esphome/core/helpers.h"
#include <queue>

namespace esphome {
namespace bus_t4 {

using namespace esphome::cover;

// Константы UART
static const int _UART_NO = UART0;
static const int TX_P = 1;
static const uint32_t BAUD_BREAK = 9200;
static const uint32_t BAUD_WORK = 19200;
static const uint8_t START_CODE = 0x55;

// Тайминги (в микросекундах)
static const uint32_t BREAK_TIMEOUT_US = 5000;    // 5 мс таймаут для break
static const uint32_t DATA_TIMEOUT_US = 10000;    // 10 мс таймаут для данных
static const uint32_t RESPONSE_TIMEOUT_MS = 100;  // 100 мс ожидание ответа
static const uint32_t RETRY_DELAY_MS = 50;        // Задержка между ретриями

// Размеры буферов
static const size_t MAX_RX_BUFFER_SIZE = 128;      // Максимальный размер RX буфера
static const size_t MAX_PACKET_SIZE = 64;          // Максимальный размер пакета
static const size_t MIN_PACKET_SIZE = 10;          // Минимальный размер пакета

static const float CLOSED_POSITION_THRESHOLD = 0.007;
static const uint32_t POSITION_UPDATE_INTERVAL = 500;

enum mes_type : uint8_t {
  CMD = 0x01,
  INF = 0x08,
};

enum cmd_mnu : uint8_t {
  CONTROL = 0x01,
};

enum sub_run_cmd2 : uint8_t {
  STA_OPENING = 0x02,
  STA_CLOSING = 0x03,
  OPENED = 0x04,
  CLOSED = 0x05,
  ENDTIME = 0x06,
  STOPPED = 0x08,
  PART_OPENED = 0x10,
};

enum errors_byte : uint8_t {
  NOERR = 0x00,
  FD = 0xFD,
};

enum motor_type : uint8_t {
  SLIDING = 0x01,
  SECTIONAL = 0x02,
  SWING = 0x03,
  BARRIER = 0x04,
  UPANDOVER = 0x05,
};

enum whose_pkt : uint8_t {
  FOR_ALL = 0x00,
  FOR_CU = 0x04,
  FOR_OXI = 0x0A,
};

enum command_pkt : uint8_t {
  TYPE_M = 0x00,
  INF_STATUS = 0x01,
  WHO = 0x04,
  MAC = 0x07,
  MAN = 0x08,
  PRD = 0x09,
  INF_SUPPORT = 0x10,
  HWR = 0x0a,
  FRM = 0x0b,
  DSC = 0x0c,
  CUR_POS = 0x11,
  MAX_OPN = 0x12,
  POS_MAX = 0x18,
  POS_MIN = 0x19,
  INF_P_OPN1 = 0x21,
  INF_P_OPN2 = 0x22,
  INF_P_OPN3 = 0x23,
  INF_SLOW_OPN = 0x24,
  INF_SLOW_CLS = 0x25,
  OPN_OFFSET = 0x28,
  CLS_OFFSET = 0x29,
  OPN_DIS = 0x2a,
  CLS_DIS = 0x2b,
  REV_TIME = 0x31,
  OPN_PWR = 0x4A,
  CLS_PWR = 0x4B,
  SPEED_OPN = 0x42,
  SPEED_CLS = 0x43,
  SPEED_SLW_OPN = 0x45,
  SPEED_SLW_CLS = 0x46,
  OUT1 = 0x51,
  OUT2 = 0x52,
  LOCK_TIME = 0x5A,
  S_CUP_TIME = 0x5C,
  LAMP_TIME = 0x5B,
  COMM_SBS = 0x61,
  COMM_POPN = 0x62,
  COMM_OPN = 0x63,
  COMM_CLS = 0x64,
  COMM_STP = 0x65,
  COMM_PHOTO = 0x68,
  COMM_PHOTO2 = 0x69,
  COMM_PHOTO3 = 0x6A,
  COMM_OPN_STP = 0x6B,
  COMM_CLS_STP = 0x6C,
  IN1 = 0x71,
  IN2 = 0x72,
  IN3 = 0x73,
  IN4 = 0x74,
  COMM_LET_OPN = 0x78,
  COMM_LET_CLS = 0x79,
  AUTOCLS = 0x80,
  P_TIME = 0x81,
  PH_CLS_ON = 0x84,
  PH_CLS_VAR = 0x86,
  PH_CLS_TIME = 0x85,
  ALW_CLS_ON = 0x88,
  ALW_CLS_VAR = 0x8A,
  ALW_CLS_TIME = 0x89,
  STAND_BY_ACT = 0x8c,
  WAIT_TIME = 0x8d,
  STAND_BY_MODE = 0x8e,
  START_ON = 0x90,
  START_TIME = 0x91,
  SLOW_ON = 0xA2,
  DIS_VAL = 0xA4,
  BLINK_ON = 0x94,
  BLINK_OPN_TIME = 0x95,
  BLINK_CLS_TIME = 0x99,
  OP_BLOCK = 0x9a,
  KEY_LOCK = 0x9c,
  T_VAL = 0xB1,
  P_COUNT = 0xB2,
  C_MAIN = 0xB4,
  DIAG_BB = 0xD0,
  INF_IO = 0xD1,
  DIAG_PAR = 0xD2,
  CUR_MAN = 0x02,
  SUBMNU = 0x04,
  STA = 0xC0,
  MAIN_SET = 0x80,
  RUN = 0x82,
};

enum run_cmd : uint8_t {
  SET = 0xA9,
  GET = 0x99,
  GET_SUPP_CMD = 0x89,
};

enum control_cmd : uint8_t {
  SBS = 0x01,
  STOP = 0x02,
  OPEN = 0x03,
  CLOSE = 0x04,
  P_OPN1 = 0x05,
  P_OPN2 = 0x06,
  P_OPN3 = 0x07,
  RSP = 0x19,
  EVT = 0x29,
  P_OPN4 = 0x0b,
  P_OPN5 = 0x0c,
  P_OPN6 = 0x0d,
  UNLK_OPN = 0x19,
  CLS_LOCK = 0x0E,
  UNLCK_CLS = 0x1A,
  LOCK = 0x0F,
  UNLOCK = 0x10,
  LIGHT_TIMER = 0x11,
  LIGHT_SW = 0x12,
  HOST_SBS = 0x13,
  HOST_OPN = 0x14,
  HOST_CLS = 0x15,
  SLAVE_SBS = 0x16,
  SLAVE_OPN = 0x17,
  SLAVE_CLS = 0x18,
  AUTO_ON = 0x1B,
  AUTO_OFF = 0x1C,
};

enum position_hook_type : uint8_t {
  IGNORE = 0x00,
  STOP_UP = 0x01,
  STOP_DOWN = 0x02
};

class NiceBusT4 : public Component, public Cover {
 public:
  // Настройки привода
  bool autocls_flag;
  bool photocls_flag;
  bool alwayscls_flag;
  bool init_ok = false;
  bool is_walky = false;
  bool is_robus = false;
  bool is_ro = false;
  
  std::vector<uint8_t> unknown = {0x55, 0x55};

  void setup() override;
  void loop() override;
  void dump_config() override;

  void send_raw_cmd(std::string data);
  void send_cmd(uint8_t data) { this->tx_buffer_.push(gen_control_cmd(data)); }
  void send_with_retry(std::vector<uint8_t> cmd, int max_retries = 3);
  void send_inf_cmd(std::string to_addr, std::string whose, std::string command, 
                    std::string type_command, std::string next_data, bool data_on, 
                    std::string data_command);
  void set_mcu(std::string command, std::string data_command);

  void set_class_gate(uint8_t class_gate) { class_gate_ = class_gate; }

  cover::CoverTraits get_traits() override;

 protected:
  void control(const cover::CoverCall &call) override;
  void request_position(void);
  void update_position(uint16_t newpos);

  uint32_t last_position_time{0};
  uint32_t update_interval_{500};
  uint32_t last_update_{0};
  uint32_t last_uart_byte_{0};

  CoverOperation last_published_op;
  float last_published_pos{-1};

  void publish_state_if_changed(void);

  uint8_t position_hook_type{IGNORE};
  uint16_t position_hook_value;

  uint8_t class_gate_ = 0x55;
  
  bool init_cu_flag = false;
  bool init_oxi_flag = false;

  // UART переменные
  uint8_t _uart_nr;
  uart_t* _uart = nullptr;
  uint16_t _max_opn = 0;
  uint16_t _pos_opn = 2048;
  uint16_t _pos_cls = 0;
  uint16_t _pos_usl = 0;
  
  uint8_t addr_from[2] = {0x00, 0x66};
  uint8_t addr_to[2];
  uint8_t addr_oxi[2];

  std::vector<uint8_t> raw_cmd_prepare(std::string data);

  // Генерация INF команд
  std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, 
                                   const uint8_t whose, const uint8_t inf_cmd, 
                                   const uint8_t run_cmd, const uint8_t next_data, 
                                   const std::vector<uint8_t> &data, size_t len);
  std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, 
                                   const uint8_t run_cmd) {
    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, 
                       run_cmd, 0x00, {0x00}, 0);
  }
  std::vector<uint8_t> gen_inf_cmd(const uint8_t whose, const uint8_t inf_cmd, 
                                   const uint8_t run_cmd, const uint8_t next_data, 
                                   std::vector<uint8_t> data) {
    return gen_inf_cmd(this->addr_to[0], this->addr_to[1], whose, inf_cmd, 
                       run_cmd, next_data, data, data.size());
  }
  std::vector<uint8_t> gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, 
                                   const uint8_t whose, const uint8_t inf_cmd, 
                                   const uint8_t run_cmd, const uint8_t next_data) {
    return gen_inf_cmd(to_addr1, to_addr2, whose, inf_cmd, run_cmd, 
                       next_data, {0x00}, 0);
  }
  
  // Генерация CMD команд
  std::vector<uint8_t> gen_control_cmd(const uint8_t control_cmd);

  void init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device);
  void send_array_cmd(std::vector<uint8_t> data);
  void send_array_cmd(const uint8_t *data, size_t len);

  void parse_status_packet(const std::vector<uint8_t> &data);
  void handle_char_(uint8_t c);
  bool validate_message_();

  std::vector<uint8_t> rx_message_;
  std::queue<std::vector<uint8_t>> tx_buffer_;
  bool ready_to_tx_{true};

  std::vector<uint8_t> manufacturer_ = {0x55, 0x55};
  std::vector<uint8_t> product_;
  std::vector<uint8_t> hardware_;
  std::vector<uint8_t> firmware_;
  std::vector<uint8_t> description_;
  std::vector<uint8_t> oxi_product;
  std::vector<uint8_t> oxi_hardware;
  std::vector<uint8_t> oxi_firmware;
  std::vector<uint8_t> oxi_description;
};

} // namespace bus_t4
} // namespace esphome
