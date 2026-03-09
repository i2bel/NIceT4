#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // для использования вспомогательных функций

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";
#include "nice-bust4.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // для использования вспомогательных функций

namespace esphome {
namespace bus_t4 {

static const char *TAG = "bus_t4.cover";

using namespace esphome::cover;

// Определения констант (добавьте это после using namespace)
const int _UART_NO = UART0;
const int TX_P = 1;
const uint32_t BAUD_BREAK = 9200;
const uint32_t BAUD_WORK = 19200;
const uint8_t START_CODE = 0x55;

const uint32_t BREAK_TIMEOUT_MS = 5;
const uint32_t DATA_TIMEOUT_MS = 10;
const uint32_t RESPONSE_TIMEOUT_MS = 100;
const uint32_t RETRY_DELAY_MS = 50;

const size_t MAX_RX_BUFFER_SIZE = 128;
const size_t MAX_PACKET_SIZE = 64;
const size_t MIN_PACKET_SIZE = 10;

const float CLOSED_POSITION_THRESHOLD = 0.007;
const uint32_t POSITION_UPDATE_INTERVAL = 500;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

// ... остальной код без изменений ...


using namespace esphome::cover;

CoverTraits NiceBusT4::get_traits() {
  auto traits = CoverTraits();
  traits.set_supports_position(true);
  traits.set_supports_stop(true);
  return traits;
}

void NiceBusT4::control(const CoverCall &call) {
  position_hook_type = IGNORE;
  if (call.get_stop()) {
    send_cmd(STOP);
  } else if (call.get_position().has_value()) {
    float newpos = *call.get_position();
    if (newpos != position) {
      if (newpos == COVER_OPEN) {
        if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
      } else if (newpos == COVER_CLOSED) {
        if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
      } else {
        position_hook_value = (_pos_opn - _pos_cls) * newpos + _pos_cls;
        ESP_LOGI(TAG, "Требуемое положение привода: %d", position_hook_value);
        if (position_hook_value > _pos_usl) {
          position_hook_type = STOP_UP;
          if (current_operation != COVER_OPERATION_OPENING) send_cmd(OPEN);
        } else {
          position_hook_type = STOP_DOWN;
          if (current_operation != COVER_OPERATION_CLOSING) send_cmd(CLOSE);
        }
      }
    }
  }
}

void NiceBusT4::setup() {
  // Увеличенный буфер UART для надежности
  _uart = uart_init(_UART_NO, BAUD_WORK, SERIAL_8N1, SERIAL_FULL, TX_P, 512, false);
}

void NiceBusT4::loop() {
  if ((millis() - this->last_update_) > 10000) {
    if (this->init_ok == false) {
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, WHO, GET, 0x00));
      this->tx_buffer_.push(gen_inf_cmd(0x00, 0xff, FOR_ALL, PRD, GET, 0x00));
    } else if (this->class_gate_ == 0x55) {
      init_device(this->addr_to[0], this->addr_to[1], 0x04);  
    } else if (this->manufacturer_ == unknown) {
      init_device(this->addr_to[0], this->addr_to[1], 0x04);  
    }
    this->last_update_ = millis();
  }

  uint32_t now = millis();
  if (now - this->last_uart_byte_ > 100) {
    this->ready_to_tx_ = true;
    this->last_uart_byte_ = now;
  }

  // Чтение из UART с защитой от переполнения
  while (uart_rx_available(_uart) > 0) {
    uint8_t c = (uint8_t)uart_read_char(_uart);
    this->handle_char_(c);
    this->last_uart_byte_ = now;
  }

  if (this->ready_to_tx_ && !this->tx_buffer_.empty()) {
    this->send_array_cmd(this->tx_buffer_.front());
    this->tx_buffer_.pop();
    this->ready_to_tx_ = false;
  }

  if (!is_robus) {
    now = millis();
    if (init_ok && (current_operation != COVER_OPERATION_IDLE) && 
        (now - last_position_time > POSITION_UPDATE_INTERVAL)) {
      last_position_time = now;
      request_position();
    }
  }
}

void NiceBusT4::handle_char_(uint8_t c) {
  // Защита от переполнения буфера
  if (this->rx_message_.size() > MAX_RX_BUFFER_SIZE) {
    ESP_LOGW(TAG, "RX buffer overflow (%d), clearing", this->rx_message_.size());
    this->rx_message_.clear();
  }
  
  this->rx_message_.push_back(c);
  
  if (!this->validate_message_()) {
    // При ошибке валидации - сбрасываем буфер, но сохраняем стартовый байт если это начало нового сообщения
    if (c == START_CODE) {
      this->rx_message_.clear();
      this->rx_message_.push_back(c);
    } else {
      this->rx_message_.clear();
    }
  }
}

bool NiceBusT4::validate_message_() {
  size_t len = this->rx_message_.size();
  if (len < MIN_PACKET_SIZE) return true;
  
  uint8_t *data = &this->rx_message_[0];
  
  // Byte 0: HEADER1 (всегда 0x00)
  if (data[0] != 0x00) {
    ESP_LOGV(TAG, "Invalid header1: 0x%02X", data[0]);
    return false;
  }
  
  // Byte 1: HEADER2 (всегда START_CODE)
  if (data[1] != START_CODE) {
    ESP_LOGV(TAG, "Invalid header2: 0x%02X", data[1]);
    return false;
  }
  
  uint8_t packet_size = data[2];
  uint8_t expected_len = packet_size + 3;
  
  // Проверка разумности длины пакета
  if (expected_len < MIN_PACKET_SIZE || expected_len > MAX_PACKET_SIZE) {
    ESP_LOGW(TAG, "Invalid packet length: %d (size=%d)", expected_len, packet_size);
    return false;
  }
  
  // Ждем полный пакет
  if (len < expected_len) return true;

  // Byte 3: Серия (ряд) кому пакет
  // Проверка не проводится
  //  uint8_t command = data[3];
  if (len == 4)
    return true;

  // Byte 4: Адрес кому пакет
  // Byte 5: Серия (ряд) от кого пакет
  // Byte 6: Адрес от кого пакет
  // Byte 7: Тип сообшения CMD или INF
  // Byte 8: Количество байт дальше за вычетом двух байт CRC в конце.

  if (len <= 9)
    // Проверка не проводится
    return true;

  uint8_t crc1 = (data[3] ^ data[4] ^ data[5] ^ data[6] ^ data[7] ^ data[8]);

  // Byte 9: crc1 = XOR (Byte 3 : Byte 8) XOR шести предыдущих байт
  if (len == 10)
    if (data[9] != crc1) {
      ESP_LOGW(TAG, "Received invalid message checksum 1 %02X!=%02X", data[9], crc1);
      return false;
    }
  // Byte 10:
  // ...

  // ждем пока поступят все данные пакета
  if (len < expected_len)
    return true;

  // считаем crc2
  uint8_t crc2 = data[10];
  for (uint8_t i = 11; i < expected_len - 1; i++) {
    crc2 = (crc2 ^ data[i]);
  }

  if (data[expected_len - 1] != crc2 ) {
    ESP_LOGW(TAG, "Received invalid message checksum 2 %02X!=%02X", data[expected_len - 1], crc2);
    return false;
  }

  // Byte Last: packet_size
  if (data[expected_len] != packet_size ) {
    ESP_LOGW(TAG, "Received invalid message size %02X!=%02X", data[expected_len], packet_size);
    return false;
  }

  // Если сюда дошли - правильное сообщение получено и лежит в буфере rx_message_

  // Удаляем 0x00 в начале сообщения
  rx_message_.erase(rx_message_.begin());

  // для вывода пакета в лог
  std::string pretty_cmd = format_hex_pretty(rx_message_);
  ESP_LOGI(TAG,  "Получен пакет: %s ", pretty_cmd.c_str() );

  // здесь что-то делаем с сообщением
  parse_status_packet(rx_message_);

  // возвращаем false чтобы обнулить rx buffer
  return false;
}

void NiceBusT4::parse_status_packet(const std::vector<uint8_t> &data) {
  if ((data[1] == 0x0d) && (data[13] == 0xFD)) {
    ESP_LOGE(TAG, "Команда недоступна для этого устройства");
  }

  if (((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && (data[13] == NOERR)) {
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    std::string str(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
    ESP_LOGI(TAG, "Строка с данными: %s", str.c_str());
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG, "Данные HEX %s", pretty_data.c_str());

    if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == GET - 0x80) && (data[13] == NOERR)) {
      ESP_LOGI(TAG, "Получен ответ на запрос %X", data[10]);
      switch (data[10]) {
        case TYPE_M:
          switch (data[14]) {
            case SLIDING:
              this->class_gate_ = SLIDING;
              break;
            case SECTIONAL:
              this->class_gate_ = SECTIONAL;
              break;
            case SWING:
              this->class_gate_ = SWING;
              break;
            case BARRIER:
              this->class_gate_ = BARRIER;
              break;
            case UPANDOVER:
              this->class_gate_ = UPANDOVER;
              break;
          }
          break;
        case INF_IO:
          switch (data[16]) {
            case 0x00:
              ESP_LOGI(TAG, "  Концевик не сработал ");
              break;
            case 0x01:
              ESP_LOGI(TAG, "  Концевик на закрытие ");
              this->position = COVER_CLOSED;
              break;
            case 0x02:
              ESP_LOGI(TAG, "  Концевик на открытие ");
              this->position = COVER_OPEN;
              break;
          }
          this->publish_state_if_changed();
          break;
        case MAX_OPN:
          if (is_walky) {
            this->_max_opn = data[15];
            this->_pos_opn = data[15];
          } else {
            this->_max_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Максимальное положение энкодера: %d", this->_max_opn);
          break;
        case POS_MIN:
          this->_pos_cls = (data[14] << 8) + data[15];
          ESP_LOGI(TAG, "Положение закрытых ворот: %d", this->_pos_cls);
          break;
        case POS_MAX:
          if (((data[14] << 8) + data[15]) > 0x00) {
            this->_pos_opn = (data[14] << 8) + data[15];
          }
          ESP_LOGI(TAG, "Положение открытых ворот: %d", this->_pos_opn);
          break;
        case CUR_POS:
          if (is_walky)
            update_position(data[15]);
          else
            update_position((data[14] << 8) + data[15]);
          break;
        case INF_STATUS:
          switch (data[14]) {
            case OPENED:
              ESP_LOGI(TAG, "  Ворота открыты");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_OPEN;
              break;
            case CLOSED:
              ESP_LOGI(TAG, "  Ворота закрыты");
              this->current_operation = COVER_OPERATION_IDLE;
              this->position = COVER_CLOSED;
              break;
            case 0x01:
              ESP_LOGI(TAG, "  Ворота остановлены");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x00:
              ESP_LOGI(TAG, "  Статус ворот неизвестен");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case 0x0b:
              ESP_LOGI(TAG, "  Поиск положений сделан");
              this->current_operation = COVER_OPERATION_IDLE;
              request_position();
              break;
            case STA_OPENING:
              ESP_LOGI(TAG, "  Идёт открывание");
              this->current_operation = COVER_OPERATION_OPENING;
              break;
            case STA_CLOSING:
              ESP_LOGI(TAG, "  Идёт закрывание");
              this->current_operation = COVER_OPERATION_CLOSING;
              break;
          }
          this->publish_state_if_changed();
          break;
        case AUTOCLS:
          this->autocls_flag = data[14];
          ESP_LOGCONFIG(TAG, "  Автозакрытие - L1: %s", autocls_flag ? "Да" : "Нет");
          break;
        case PH_CLS_ON:
          this->photocls_flag = data[14];
          break;
        case ALW_CLS_ON:
          this->alwayscls_flag = data[14];
          break;
      }
    }

    if ((data[6] == INF) && (data[11] == GET - 0x81) && (data[13] == NOERR)) {
      ESP_LOGI(TAG, "Получен незавершенный ответ на запрос %X, продолжение со смещением %X", data[10], data[12]);
      tx_buffer_.push(gen_inf_cmd(data[4], data[5], data[9], data[10], GET, data[12]));
    }

    if ((data[6] == INF) && (data[9] == FOR_CU) && (data[11] == SET - 0x80) && (data[13] == NOERR)) {
      switch (data[10]) {
        case AUTOCLS:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, AUTOCLS, GET));
          break;
        case PH_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, PH_CLS_ON, GET));
          break;
        case ALW_CLS_ON:
          tx_buffer_.push(gen_inf_cmd(FOR_CU, ALW_CLS_ON, GET));
          break;
      }
    }

    if ((data[6] == INF) && (data[9] == FOR_ALL) && 
        ((data[11] == GET - 0x80) || (data[11] == GET - 0x81)) && 
        (data[13] == NOERR)) {
      switch (data[10]) {
        case MAN:
          this->manufacturer_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          break;
        case PRD:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_product.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->product_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
            std::vector<uint8_t> wla1 = {0x57, 0x4C, 0x41, 0x31, 0x00, 0x06, 0x57};
            std::vector<uint8_t> ROBUSHSR10 = {0x52, 0x4F, 0x42, 0x55, 0x53, 0x48, 0x53, 0x52, 0x31, 0x30, 0x00};
            if (this->product_ == wla1) {
              this->is_walky = true;
            }
            if (this->product_ == ROBUSHSR10) {
              this->is_robus = true;
            }
          }
          break;
        case HWR:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_hardware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->hardware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case FRM:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_firmware.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->firmware_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case DSC:
          if ((this->addr_oxi[0] == data[4]) && (this->addr_oxi[1] == data[5])) {
            this->oxi_description.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          } else if ((this->addr_to[0] == data[4]) && (this->addr_to[1] == data[5])) {
            this->description_.assign(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
          }
          break;
        case WHO:
          if (data[12] == 0x01) {
            if (data[14] == 0x04) {
              this->addr_to[0] = data[4];
              this->addr_to[1] = data[5];
              this->init_ok = true;
            } else if (data[14] == 0x0A) {
              this->addr_oxi[0] = data[4];
              this->addr_oxi[1] = data[5];
              init_device(data[4], data[5], data[14]);
            }
          }
          break;
      }
    }

    if ((data[9] == 0x0A) && (data[10] == 0x25) && (data[11] == 0x01) && 
        (data[12] == 0x0A) && (data[13] == NOERR)) {
      std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
      ESP_LOGCONFIG(TAG, "Номер пульта: %X%X%X%X, команда: %X, кнопка: %X, режим: %X, счётчик нажатий: %d", 
                   vec_data[5], vec_data[4], vec_data[3], vec_data[2], 
                   vec_data[8] / 0x10, vec_data[5] / 0x10, vec_data[7] + 0x01, vec_data[6]);
    }

    if ((data[9] == 0x0A) && (data[10] == 0x26) && (data[11] == 0x41) && 
        (data[12] == 0x08) && (data[13] == NOERR)) {
      std::vector<uint8_t> vec_data(this->rx_message_.begin() + 14, this->rx_message_.end() - 2);
      ESP_LOGCONFIG(TAG, "Кнопка %X, номер пульта: %X%X%X%X", 
                   vec_data[0] / 0x10, vec_data[0] % 0x10, vec_data[1], vec_data[2], vec_data[3]);
    }
  } else if (data[1] > 0x0d) {
    ESP_LOGD(TAG, "Получен пакет RSP");
    std::vector<uint8_t> vec_data(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    std::string str(this->rx_message_.begin() + 12, this->rx_message_.end() - 3);
    ESP_LOGI(TAG, "Строка с данными: %s", str.c_str());
    std::string pretty_data = format_hex_pretty(vec_data);
    ESP_LOGI(TAG, "Данные HEX %s", pretty_data.c_str());
    
    switch (data[9]) {
      case FOR_CU:
        ESP_LOGI(TAG, "Пакет контроллера привода");
        switch (data[10] + 0x80) {
          case RUN:
            ESP_LOGI(TAG, "Подменю RUN");
            if (data[11] >= 0x80) {
              switch (data[11] - 0x80) {
                case SBS:
                  ESP_LOGI(TAG, "Команда: Пошагово");
                  break;
                case STOP:
                  ESP_LOGI(TAG, "Команда: STOP");
                  break;
                case OPEN:
                  ESP_LOGI(TAG, "Команда: OPEN");
                  this->current_operation = COVER_OPERATION_OPENING;
                  break;
                case CLOSE:
                  ESP_LOGI(TAG, "Команда: CLOSE");
                  this->current_operation = COVER_OPERATION_CLOSING;
                  break;
                case P_OPN1:
                  ESP_LOGI(TAG, "Команда: Частичное открывание 1");
                  break;
                case STOPPED:
                  ESP_LOGI(TAG, "Команда: Остановлено");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                case ENDTIME:
                  ESP_LOGI(TAG, "Операция завершена по таймауту");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                default:
                  ESP_LOGI(TAG, "Неизвестная команда: %X", data[11]);
              }
            } else {
              switch (data[11]) {
                case STA_OPENING:
                  ESP_LOGI(TAG, "Операция: Открывается");
                  this->current_operation = COVER_OPERATION_OPENING;
                  break;
                case STA_CLOSING:
                  ESP_LOGI(TAG, "Операция: Закрывается");
                  this->current_operation = COVER_OPERATION_CLOSING;
                  break;
                case CLOSED:
                  ESP_LOGI(TAG, "Операция: Закрыто");
                  this->current_operation = COVER_OPERATION_IDLE;
                  this->position = COVER_CLOSED;
                  break;
                case OPENED:
                  ESP_LOGI(TAG, "Операция: Открыто");
                  this->current_operation = COVER_OPERATION_IDLE;
                  this->position = COVER_OPEN;
                  if (this->_max_opn == 0) {
                    this->_max_opn = this->_pos_opn = this->_pos_usl;
                    ESP_LOGI(TAG, "Opened position calibrated");
                  }
                  break;
                case STOPPED:
                  ESP_LOGI(TAG, "Операция: Остановлено");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                case PART_OPENED:
                  ESP_LOGI(TAG, "Операция: Частично открыто");
                  this->current_operation = COVER_OPERATION_IDLE;
                  request_position();
                  break;
                default:
                  ESP_LOGI(TAG, "Неизвестная операция: %X", data[11]);
              }
            }
            this->publish_state_if_changed();
            break;
          case STA:
            ESP_LOGI(TAG, "Подменю Статус в движении");
            switch (data[11]) {
              case STA_OPENING:
              case 0x83:
                ESP_LOGI(TAG, "Движение: Открывается");
                this->current_operation = COVER_OPERATION_OPENING;
                break;
              case STA_CLOSING:
              case 0x84:
                ESP_LOGI(TAG, "Движение: Закрывается");
                this->current_operation = COVER_OPERATION_CLOSING;
                break;
              case CLOSED:
                ESP_LOGI(TAG, "Движение: Закрыто");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_CLOSED;
                break;
              case OPENED:
                ESP_LOGI(TAG, "Движение: Открыто");
                this->current_operation = COVER_OPERATION_IDLE;
                this->position = COVER_OPEN;
                break;
              case STOPPED:
                ESP_LOGI(TAG, "Движение: Остановлено");
                this->current_operation = COVER_OPERATION_IDLE;
                request_position();
                break;
              default:
                ESP_LOGI(TAG, "Движение: %X", data[11]);
            }
            update_position((data[12] << 8) + data[13]);
            break;
          default:
            ESP_LOGI(TAG, "Подменю %X", data[10]);
        }
        break;
      case CONTROL:
        ESP_LOGI(TAG, "Пакет CONTROL");
        break;
      case FOR_ALL:
        ESP_LOGI(TAG, "Пакет для всех");
        break;
      case 0x0A:
        ESP_LOGI(TAG, "Пакет приёмника");
        break;
      default:
        ESP_LOGI(TAG, "Меню %X", data[9]);
    }
  }

  if ((data[6] == CMD) && (data[9] == FOR_CU) && (data[10] == CUR_MAN) && (data[13] == NOERR)) {
    switch (data[11]) {
      case STA_OPENING:
        this->current_operation = COVER_OPERATION_OPENING;
        ESP_LOGD(TAG, "Статус: Открывается");
        break;
      case STA_CLOSING:
        this->current_operation = COVER_OPERATION_CLOSING;
        ESP_LOGD(TAG, "Статус: Закрывается");
        break;
      case OPENED:
        this->position = COVER_OPEN;
        ESP_LOGD(TAG, "Статус: Открыто");
        this->current_operation = COVER_OPERATION_IDLE;
        break;
      case CLOSED:
        this->position = COVER_CLOSED;
        ESP_LOGD(TAG, "Статус: Закрыто");
        this->current_operation = COVER_OPERATION_IDLE;
        break;
      case STOPPED:
        this->current_operation = COVER_OPERATION_IDLE;
        ESP_LOGD(TAG, "Статус: Остановлено");
        break;
    }
    this->publish_state();
  }
}

void NiceBusT4::dump_config() {
  ESP_LOGCONFIG(TAG, "  Bus T4 Cover");
  switch (this->class_gate_) {
    case SLIDING:
      ESP_LOGCONFIG(TAG, "  Тип: Откатные ворота");
      break;
    case SECTIONAL:
      ESP_LOGCONFIG(TAG, "  Тип: Секционные ворота");
      break;
    case SWING:
      ESP_LOGCONFIG(TAG, "  Тип: Распашные ворота");
      break;
    case BARRIER:
      ESP_LOGCONFIG(TAG, "  Тип: Шлагбаум");
      break;
    case UPANDOVER:
      ESP_LOGCONFIG(TAG, "  Тип: Подъёмно-поворотные ворота");
      break;
    default:
      ESP_LOGCONFIG(TAG, "  Тип: Неизвестные ворота, 0x%02X", this->class_gate_);
  }

  ESP_LOGCONFIG(TAG, "  Максимальное положение энкодера или таймера: %d", this->_max_opn);
  ESP_LOGCONFIG(TAG, "  Положение отрытых ворот: %d", this->_pos_opn);
  ESP_LOGCONFIG(TAG, "  Положение закрытых ворот: %d", this->_pos_cls);

  std::string manuf_str(this->manufacturer_.begin(), this->manufacturer_.end());
  ESP_LOGCONFIG(TAG, "  Производитель: %s", manuf_str.c_str());

  std::string prod_str(this->product_.begin(), this->product_.end());
  ESP_LOGCONFIG(TAG, "  Привод: %s", prod_str.c_str());

  std::string hard_str(this->hardware_.begin(), this->hardware_.end());
  ESP_LOGCONFIG(TAG, "  Железо привода: %s", hard_str.c_str());

  std::string firm_str(this->firmware_.begin(), this->firmware_.end());
  ESP_LOGCONFIG(TAG, "  Прошивка привода: %s", firm_str.c_str());
  
  std::string dsc_str(this->description_.begin(), this->description_.end());
  ESP_LOGCONFIG(TAG, "  Описание привода: %s", dsc_str.c_str());

  ESP_LOGCONFIG(TAG, "  Адрес шлюза: 0x%02X%02X", addr_from[0], addr_from[1]);
  ESP_LOGCONFIG(TAG, "  Адрес привода: 0x%02X%02X", addr_to[0], addr_to[1]);
  ESP_LOGCONFIG(TAG, "  Адрес приёмника: 0x%02X%02X", addr_oxi[0], addr_oxi[1]);
  
  std::string oxi_prod_str(this->oxi_product.begin(), this->oxi_product.end());
  ESP_LOGCONFIG(TAG, "  Приёмник: %s", oxi_prod_str.c_str());
  
  std::string oxi_hard_str(this->oxi_hardware.begin(), this->oxi_hardware.end());
  ESP_LOGCONFIG(TAG, "  Железо приёмника: %s", oxi_hard_str.c_str());

  std::string oxi_firm_str(this->oxi_firmware.begin(), this->oxi_firmware.end());
  ESP_LOGCONFIG(TAG, "  Прошивка приёмника: %s", oxi_firm_str.c_str());
  
  std::string oxi_dsc_str(this->oxi_description.begin(), this->oxi_description.end());
  ESP_LOGCONFIG(TAG, "  Описание приёмника: %s", oxi_dsc_str.c_str());
 
  ESP_LOGCONFIG(TAG, "  Автозакрытие - L1: %s", autocls_flag ? "Да" : "Нет");
  ESP_LOGCONFIG(TAG, "  Закрыть после фото - L2: %s", photocls_flag ? "Да" : "Нет");
  ESP_LOGCONFIG(TAG, "  Всегда закрывать - L3: %s", alwayscls_flag ? "Да" : "Нет");
}

std::vector<uint8_t> NiceBusT4::gen_control_cmd(const uint8_t control_cmd) {
  std::vector<uint8_t> frame = {this->addr_to[0], this->addr_to[1], 
                                 this->addr_from[0], this->addr_from[1]};
  frame.push_back(CMD);
  frame.push_back(0x05);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(CONTROL);
  frame.push_back(RUN);
  frame.push_back(control_cmd);
  frame.push_back(0x64);
  uint8_t crc2 = (frame[7] ^ frame[8] ^ frame[9] ^ frame[10]);
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);
  return frame;
}

std::vector<uint8_t> NiceBusT4::gen_inf_cmd(const uint8_t to_addr1, const uint8_t to_addr2, 
                                           const uint8_t whose, const uint8_t inf_cmd, 
                                           const uint8_t run_cmd, const uint8_t next_data, 
                                           const std::vector<uint8_t> &data, size_t len) {
  std::vector<uint8_t> frame = {to_addr1, to_addr2, this->addr_from[0], this->addr_from[1]};
  frame.push_back(INF);
  frame.push_back(0x06 + len);
  uint8_t crc1 = (frame[0] ^ frame[1] ^ frame[2] ^ frame[3] ^ frame[4] ^ frame[5]);
  frame.push_back(crc1);
  frame.push_back(whose);
  frame.push_back(inf_cmd);
  frame.push_back(run_cmd);
  frame.push_back(next_data);
  frame.push_back(len);
  if (len > 0) {
    frame.insert(frame.end(), data.begin(), data.end());
  }
  uint8_t crc2 = frame[7];
  for (size_t i = 8; i < 12 + len; i++) {
    crc2 = crc2 ^ frame[i];
  }
  frame.push_back(crc2);
  uint8_t f_size = frame.size();
  frame.push_back(f_size);
  frame.insert(frame.begin(), f_size);
  frame.insert(frame.begin(), START_CODE);
  return frame;
}

void NiceBusT4::send_raw_cmd(std::string data) {
  std::vector<uint8_t> v_cmd = raw_cmd_prepare(data);
  send_array_cmd(v_cmd);
}

std::vector<uint8_t> NiceBusT4::raw_cmd_prepare(std::string data) {
  data.erase(remove_if(data.begin(), data.end(), [](const unsigned char ch) {
    return (!(isxdigit(ch)));
  }), data.end());

  std::vector<uint8_t> frame;
  frame.resize(0);

  for (uint8_t i = 0; i < data.size(); i += 2) {
    std::string sub_str(data, i, 2);
    char hexstoi = (char)std::strtol(&sub_str[0], 0, 16);
    frame.push_back(hexstoi);
  }
  return frame;
}

void NiceBusT4::send_array_cmd(std::vector<uint8_t> data) {
  send_array_cmd((const uint8_t *)data.data(), data.size());
}

void NiceBusT4::send_array_cmd(const uint8_t *data, size_t len) {
  // Запрещаем прерывания на время отправки break
  noInterrupts();
  
  uart_flush(_uart);
  
  // Устанавливаем низкую скорость для break
  uart_set_baudrate(_uart, BAUD_BREAK);
  delayMicroseconds(50);
  
  // Отправляем 2 байта 0x00 для гарантированного break
  uint8_t break_bytes[2] = {0x00, 0x00};
  uart_write(_uart, (char *)break_bytes, 2);
  
  // Ждем окончания отправки break
  uart_wait_tx_empty(_uart);
  delayMicroseconds(200); // Гарантированная пауза после break
  
  // Возвращаем рабочую скорость
  uart_set_baudrate(_uart, BAUD_WORK);
  delayMicroseconds(100);
  
  // Отправляем основные данные
  uart_write(_uart, (char *)data, len);
  
  // Ждем окончания отправки данных
  uart_wait_tx_empty(_uart);
  
  interrupts();
  
  std::string pretty_cmd = format_hex_pretty(data, len);
  ESP_LOGI(TAG, "Отправлено: %s", pretty_cmd.c_str());
}

void NiceBusT4::send_with_retry(std::vector<uint8_t> cmd, int max_retries) {
  for (int i = 0; i < max_retries; i++) {
    size_t queue_size = tx_buffer_.size();
    send_array_cmd(cmd);
    
    uint32_t timeout = millis() + RESPONSE_TIMEOUT_MS;
    bool got_response = false;
    
    while (millis() < timeout) {
      if (tx_buffer_.size() > queue_size) {
        got_response = true;
        break;
      }
      delay(5);
    }
    
    if (got_response) {
      return;
    }
    
    ESP_LOGW(TAG, "No response, retry %d/%d", i + 1, max_retries);
    delay(RETRY_DELAY_MS);
  }
  
  ESP_LOGE(TAG, "Command failed after %d retries", max_retries);
}

void NiceBusT4::send_inf_cmd(std::string to_addr, std::string whose, std::string command, 
                            std::string type_command, std::string next_data, 
                            bool data_on, std::string data_command) {
  std::vector<uint8_t> v_to_addr = raw_cmd_prepare(to_addr);
  std::vector<uint8_t> v_whose = raw_cmd_prepare(whose);
  std::vector<uint8_t> v_command = raw_cmd_prepare(command);
  std::vector<uint8_t> v_type_command = raw_cmd_prepare(type_command);
  std::vector<uint8_t> v_next_data = raw_cmd_prepare(next_data);
  std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);

  if (data_on) {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], 
                                v_type_command[0], v_next_data[0], v_data_command, 
                                v_data_command.size()));
  } else {
    tx_buffer_.push(gen_inf_cmd(v_to_addr[0], v_to_addr[1], v_whose[0], v_command[0], 
                                v_type_command[0], v_next_data[0]));
  }
}

void NiceBusT4::set_mcu(std::string command, std::string data_command) {
  std::vector<uint8_t> v_command = raw_cmd_prepare(command);
  std::vector<uint8_t> v_data_command = raw_cmd_prepare(data_command);
  tx_buffer_.push(gen_inf_cmd(0x04, v_command[0], 0xa9, 0x00, v_data_command));
}

void NiceBusT4::init_device(const uint8_t addr1, const uint8_t addr2, const uint8_t device) {
  if (device == FOR_CU) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, TYPE_M, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, MAN, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MAX, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, POS_MIN, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
    if (is_walky)
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00, {0x01}, 1));
    else
      tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, MAX_OPN, GET, 0x00));
    request_position();
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, INF_STATUS, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, AUTOCLS, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, PH_CLS_ON, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, device, ALW_CLS_ON, GET, 0x00));
  }
  if (device == FOR_OXI) {
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, PRD, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, HWR, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, FRM, GET, 0x00));
    tx_buffer_.push(gen_inf_cmd(addr1, addr2, FOR_ALL, DSC, GET, 0x00));
  }
}

void NiceBusT4::request_position() {
  if (is_walky)
    tx_buffer_.push(gen_inf_cmd(this->addr_to[0], this->addr_to[1], FOR_CU, CUR_POS, GET, 0x00, {0x01}, 1));
  else
    tx_buffer_.push(gen_inf_cmd(FOR_CU, CUR_POS, GET));
}

void NiceBusT4::update_position(uint16_t newpos) {
  last_position_time = millis();
  _pos_usl = newpos;
  position = (_pos_usl - _pos_cls) * 1.0f / (_pos_opn - _pos_cls);
  ESP_LOGI(TAG, "Условное положение ворот: %d, положение в %%: %.3f", newpos, position);
  if (position < CLOSED_POSITION_THRESHOLD) position = COVER_CLOSED;
  publish_state_if_changed();
  
  if ((position_hook_type == STOP_UP && _pos_usl >= position_hook_value) || 
      (position_hook_type == STOP_DOWN && _pos_usl <= position_hook_value)) {
    ESP_LOGI(TAG, "Достигнуто требуемое положение. Останавливаем ворота");
    send_cmd(STOP);
    position_hook_type = IGNORE;
  }
}

void NiceBusT4::publish_state_if_changed() {
  if (current_operation == COVER_OPERATION_IDLE) position_hook_type = IGNORE;
  if (last_published_op != current_operation || last_published_pos != position) {
    publish_state();
    last_published_op = current_operation;
    last_published_pos = position;
  }
}

}  // namespace bus_t4
}  // namespace esphome
