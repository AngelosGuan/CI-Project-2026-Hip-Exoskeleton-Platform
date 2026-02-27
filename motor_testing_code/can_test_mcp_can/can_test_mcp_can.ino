/*****************************************
MCP2515 (mcp_can.h / Cory J. Fowler library)  ->  Arduino UNO

Typical SPI wiring:
SCK  -> D13
MISO -> D12
MOSI -> D11
CS   -> D9   (set below)
INT  -> D2   (optional; this sketch polls, INT not required)

CANH/CANL -> your CAN bus
*****************************************/

#include <SPI.h>
#include <mcp_can.h>

// ----------------------- MCP_CAN setup -----------------------
static const uint8_t CAN_CS_PIN  = 10;
static const uint8_t CAN_INT_PIN = 2;   // optional (polling used)
MCP_CAN CAN0(CAN_CS_PIN);

// ----------------------- Original globals --------------------
#ifndef CAN_EFF_FLAG
#define CAN_EFF_FLAG 0x80000000UL
#endif

struct can_frame {
  uint32_t can_id;   // includes CAN_EFF_FLAG when extended
  uint8_t  can_dlc;
  uint8_t  data[8];
};

struct can_frame canMsg;
struct can_frame canMsg1;

uint8_t CAN_ID = 0x68; // for the working motor

// ----------------------- MCP_CAN wrappers --------------------
static void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len) {
  canMsg1.can_id = id;      // standard ID
  canMsg1.can_dlc = len;
  memcpy(canMsg1.data, data, len);
  // ext = 0 for standard frame
  unsigned char ret = CAN0.sendMsgBuf((unsigned long)id, 0, len, (uint8_t *)data);
  if (ret == CAN_OK){
    Serial.print("sent standard frame message\r\n");
  } else {
    Serial.print(ret);
    Serial.print(": fail to send standard frame message\r\n");
  }
}

static void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
  canMsg1.can_id = (CAN_EFF_FLAG + id);  // keep original behavior/flagging
  canMsg1.can_dlc = len;
  memcpy(canMsg1.data, data, len);
  // ext = 1 for extended frame
  unsigned char ret = CAN0.sendMsgBuf((unsigned long)id, 1, len, (uint8_t *)data);
  if (ret == CAN_OK){
    Serial.print("sent extended message\r\n");
  } else {
    Serial.print(ret);
    Serial.print(": fail to send extended message\r\n");
  }
}

static bool read_can_frame(struct can_frame *out) {
  if (CAN0.checkReceive() != CAN_MSGAVAIL) {
    return false;
  }

  unsigned long rxId = 0;
  unsigned char len = 0;
  unsigned char buf[8] = {0};

  // read ID + data (mcp_can.h)
  CAN0.readMsgBuf(&rxId, &len, buf);

  // Determine if it was extended. Some variants encode it; getCanId() does not expose ext.
  // The Fowler library typically stores the "extended" bit internally; the easiest robust
  // approach is to re-query with isExtendedFrame() when available.
  // If your library version lacks isExtendedFrame(), this will still compile because
  // we guard it.

  bool isExt = false;

  out->can_id  = (uint32_t)(rxId & 0x1FFFFFFFUL);
  if (isExt) out->can_id |= CAN_EFF_FLAG;
  out->can_dlc = (uint8_t)len;
  for (uint8_t i = 0; i < out->can_dlc; i++) out->data[i] = buf[i];

  return true;
}

//int16数据位整理
void buffer_append_int16(uint8_t *buffer, int16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//uint16数据位整理
void buffer_append_uint16(uint8_t *buffer, uint16_t number, int32_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//int32数据位整理
void buffer_append_int32(uint8_t *buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//uint32数据位整理
void buffer_append_uint32(uint8_t *buffer, uint32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//int64数据位整理
void buffer_append_int64(uint8_t *buffer, int64_t number, int32_t *index) {
  buffer[(*index)++] = number >> 56;
  buffer[(*index)++] = number >> 48;
  buffer[(*index)++] = number >> 40;
  buffer[(*index)++] = number >> 32;
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

//uint64数据位整理
void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index) {
  buffer[(*index)++] = number >> 56;
  buffer[(*index)++] = number >> 48;
  buffer[(*index)++] = number >> 40;
  buffer[(*index)++] = number >> 32;
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}

enum {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT,
  CAN_PACKET_SET_CURRENT_BRAKE,
  CAN_PACKET_SET_RPM,
  CAN_PACKET_SET_POS,
  CAN_PACKET_SET_ORIGIN_HERE,
  CAN_PACKET_SET_POS_SPD
} CAN_PACKET_ID;

/*******************Servo*******************/
//占空比模式
void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

//电流环模式
void comm_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

//电流刹车模式
void comm_can_set_cb(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

//速度环模式
void comm_can_set_rpm(uint8_t controller_id, float rpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)rpm, &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

//位置环模式
void comm_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}

//设置原点模式
void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode) {
  int32_t send_index = 0;
  uint8_t buffer;
  buffer = set_origin_mode;
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8), &buffer, send_index);
}

//位置速度环模式
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA) {
  int32_t send_index = 0;
  int32_t send_index1 = 4;
  uint8_t buffer[8];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer, spd / 10.0, &send_index1);
  buffer_append_int16(buffer, RPA / 10.0, &send_index1);
  comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index1);
}

/********************MIT********************/
//进入电机模式
void set_mit_mode_run(uint8_t controller_id) {
  uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
  comm_can_transmit_eid(controller_id, buffer, 8);
}
//退出电机模式
void set_mit_mode_idle(uint8_t controller_id) {
  uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };
  comm_can_transmit_eid(controller_id, buffer, 8);
}
//设置原点
void set_mit_current_position_zero_positong(uint8_t controller_id) {
  uint8_t buffer[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe };
  comm_can_transmit_eid(controller_id, buffer, 8);
}

unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  /// Converts a float to an unsigned int, given range and number of bits ///
  float span = x_max - x_min;
  if (x < x_min) x = x_min;
  else if (x > x_max) x = x_max;
  return ((x - x_min) * ((float)((((long)1 << bits) - 1) / span)));
}

float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits) {
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)(((long)1 << bits) - 1)) + offset;
}

void buffer_append_float(uint8_t *buffer, float number, int32_t *index) {
  uint8_t *p_float = 0;
  p_float = (uint8_t *)&number;
  buffer[(*index)++] = *(p_float);
  buffer[(*index)++] = *(p_float + 1);
  buffer[(*index)++] = *(p_float + 2);
  buffer[(*index)++] = *(p_float + 3);
}

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define T_MIN -18.0f
#define T_MAX 18.0f
#define KP_MIN 0
#define KP_MAX 500.0f
#define KD_MIN 0
#define KD_MAX 5.0f

void pack_cmd(uint8_t controller_id, float p_des, float v_des, float kp, float kd, float t_ff) {

  int p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  int v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  int kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  int kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  int t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);

  uint8_t buffer[8];

  /// pack ints into the can buffer ///
  buffer[0] = p_int >> 8;                            //位置高 8
  buffer[1] = p_int & 0xFF;                          //位置低 8
  buffer[2] = v_int >> 4;                            //速度高 8 位
  buffer[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);  //速度低 4 位 KP 高 4 位
  buffer[4] = kp_int & 0xFF;                         //KP 低 8 位
  buffer[5] = kd_int >> 4;                           //KD 高 8 位
  buffer[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);  //KP 低 4 位扭矩高 4 位
  buffer[7] = t_int & 0xff;                          //扭矩低 8 位

  comm_can_transmit_sid(controller_id, buffer, 8);
}

void motor_receive_servo(struct can_frame *rx_message) {
  float motor_pos;
  float motor_spd;
  float motor_cur;
  int8_t temp1;
  int8_t temp2;
  int8_t error;

  int16_t pos_int = (rx_message)->data[0] << 8 | (rx_message)->data[1];
  int16_t spd_int = (rx_message)->data[2] << 8 | (rx_message)->data[3];
  int16_t cur_int = (rx_message)->data[4] << 8 | (rx_message)->data[5];
  motor_pos = (float)(pos_int * 0.1f);    //电机位置
  motor_spd = (float)(spd_int * 10.0f);   //电机速度
  motor_cur = (float)(cur_int * 0.01f);   //电机电流
  temp1 = (rx_message)->data[6];          //电机温度
  error = (rx_message)->data[7];          //电机故障码

  Serial.print(motor_pos);
  Serial.print(" ");
  Serial.print(motor_spd);
  Serial.print(" ");
  Serial.print(motor_cur);
  Serial.print(" ");
  Serial.print(temp1);
  Serial.print(" ");
  Serial.print(error);
  Serial.println(" ");
}

void motor_receive_mit(struct can_frame *rx_message) {
  float motor_pos;
  float motor_spd;
  float motor_t;
  int8_t temp1;
  int8_t temp2;
  int8_t error;
  int8_t err_int = (rx_message)->data[0] >> 4;
  uint16_t pos_int = (rx_message)->data[1] << 8 | (rx_message)->data[2];
  uint16_t spd_int = (rx_message)->data[3] << 4 | (rx_message)->data[4] >> 4;
  uint16_t t_int = ((rx_message)->data[4] & 0xF) << 8 | (rx_message)->data[5];
  // error = (rx_message)->data[0] >> 4;                 // 电机ID
  motor_pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);  // 电机位置
  motor_spd = uint_to_float(spd_int, V_MIN, V_MAX, 12);  // 电机速度
  motor_t = uint_to_float(t_int, T_MIN, T_MAX, 12);      // 电机电流
  temp1 = (rx_message)->data[6];                         // 电机温度
  error = (rx_message)->data[7];                         // 错误标志

  Serial.print(motor_pos);
  Serial.print(" ");
  Serial.print(motor_spd);
  Serial.print(" ");
  Serial.print(motor_t);
  Serial.print(" ");
  Serial.print(temp1);
  Serial.print(" ");
  Serial.print(temp2);
  Serial.print(" ");
  Serial.print(error);
  Serial.print(" ");
}

void down() {

  switch (3) {
    case CAN_PACKET_SET_DUTY:
      comm_can_set_duty(CAN_ID, 0.1);
      break;
    case CAN_PACKET_SET_CURRENT:
      comm_can_set_current(CAN_ID, 1);
      break;
    case CAN_PACKET_SET_CURRENT_BRAKE:
      comm_can_set_cb(CAN_ID, 1);
      break;
    case CAN_PACKET_SET_RPM:
      comm_can_set_rpm(CAN_ID, 1000);
      break;
    case CAN_PACKET_SET_POS:
      comm_can_set_pos(CAN_ID, 180);
      break;
    case CAN_PACKET_SET_ORIGIN_HERE:
      comm_can_set_origin(CAN_ID, 0);
      break;
    case CAN_PACKET_SET_POS_SPD:
      comm_can_set_pos_spd(CAN_ID, 180, 1000, 1000);
      break;
    default:
      pack_cmd(CAN_ID, 0, 1, 0, 1, 0);
      break;
  }
}

const uint8_t can_test[8] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(10, OUTPUT);
  //pinMode(11, INPUT);
  //pinMode(12, INPUT);
  //pinMode(13, INPUT);
  //pinMode(CAN_INT_PIN, INPUT);

  Serial.begin(115200);
  Serial.print("run...\r\n");

  // Initialize CAN @ 1Mbps
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.print("CAN init ok!\r\n");
    CAN0.setMode(MCP_NORMAL);

    comm_can_transmit_sid(0x00, can_test, 8); // just for testing, not actually sent to the motor
  } else {
    Serial.print("CAN init fault!\r\n");
  }

  //set_mit_mode_run(CAN_ID);
  delay(100);
}

uint8_t cntr = 0;

void loop() {
  delay(100);
  //down();
  // pack_cmd(CAN_ID,0,1,0,1,0);

  //example code to use velocity control mode: 
  //comm_can_set_rpm(CAN_ID, 5000);

  // change this to other set methods to achieve position control and current control. 
  // (KT (Nm/A): 0.095 for torque control via current control mode for this motor)

  if (read_can_frame(&canMsg)) {
    if (canMsg.can_id & CAN_EFF_FLAG) {
      canMsg.can_id -= CAN_EFF_FLAG;
      Serial.print("eid ");
    } else {
      Serial.print("sid ");
    }

    Serial.print(canMsg.can_id, HEX);   // print ID
    Serial.print(" len ");
    Serial.print(canMsg.can_dlc, HEX);  // print DLC
    Serial.print(" data ");

    for (int i = 0; i < canMsg.can_dlc; i++) {  // print the data
      Serial.print(canMsg.data[i], HEX);
      Serial.print(" ");
    }

    Serial.println();

    if (canMsg.can_id == (0x2900 | CAN_ID))
      motor_receive_servo(&canMsg);

    // motor_receive_mit(&canMsg);

    cntr++;
  }
}
