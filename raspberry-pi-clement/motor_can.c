#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>

// ----------------------- Motor ID -----------------------
#define CAN_ID 0x68

// ----------------------- MIT mode limits ----------------
#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -45.0f
#define V_MAX   45.0f
#define T_MIN  -18.0f
#define T_MAX   18.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f

// ----------------------- Packet IDs --------------------
typedef enum {
    CAN_PACKET_SET_DUTY         = 0,
    CAN_PACKET_SET_CURRENT      = 1,
    CAN_PACKET_SET_CURRENT_BRAKE= 2,
    CAN_PACKET_SET_RPM          = 3,
    CAN_PACKET_SET_POS          = 4,
    CAN_PACKET_SET_ORIGIN_HERE  = 5,
    CAN_PACKET_SET_POS_SPD      = 6
} CAN_PACKET_ID;

// ----------------------- Global socket ------------------
static int can_sock = -1;

// ========================================================
//  CAN socket init
// ========================================================
int init_can(const char *interface)
{
    can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_sock < 0) { perror("socket"); return -1; }

    struct ifreq ifr;
    strncpy(ifr.ifr_name, interface, IFNAMSIZ);
    if (ioctl(can_sock, SIOCGIFINDEX, &ifr) < 0) { perror("ioctl"); return -1; }

    struct sockaddr_can addr = {
        .can_family  = AF_CAN,
        .can_ifindex = ifr.ifr_ifindex
    };
    if (bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); return -1;
    }
    return 0;
}

// ========================================================
//  Buffer helpers  (direct ports of the Arduino functions)
// ========================================================
void buffer_append_int16(uint8_t *buf, int16_t n, int32_t *idx) {
    buf[(*idx)++] = n >> 8;
    buf[(*idx)++] = n;
}
void buffer_append_uint16(uint8_t *buf, uint16_t n, int32_t *idx) {
    buf[(*idx)++] = n >> 8;
    buf[(*idx)++] = n;
}
void buffer_append_int32(uint8_t *buf, int32_t n, int32_t *idx) {
    buf[(*idx)++] = n >> 24;
    buf[(*idx)++] = n >> 16;
    buf[(*idx)++] = n >> 8;
    buf[(*idx)++] = n;
}
void buffer_append_uint32(uint8_t *buf, uint32_t n, int32_t *idx) {
    buf[(*idx)++] = n >> 24;
    buf[(*idx)++] = n >> 16;
    buf[(*idx)++] = n >> 8;
    buf[(*idx)++] = n;
}

// ========================================================
//  Transmit helpers
//  SID = standard 11-bit frame
//  EID = extended 29-bit frame  (CAN_EFF_FLAG tells the kernel)
// ========================================================
static void comm_can_transmit_sid(uint32_t id, const uint8_t *data, uint8_t len)
{
    struct can_frame frame = {0};
    frame.can_id  = id & CAN_SFF_MASK;   // standard, no EFF flag
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    if (write(can_sock, &frame, sizeof(frame)) != sizeof(frame))
        perror("transmit_sid write");
    else
        printf("sent standard frame  ID=0x%03X\n", id);
}

static void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len)
{
    struct can_frame frame = {0};
    frame.can_id  = (id & CAN_EFF_MASK) | CAN_EFF_FLAG;  // extended frame
    frame.can_dlc = len;
    memcpy(frame.data, data, len);

    if (write(can_sock, &frame, sizeof(frame)) != sizeof(frame))
        perror("transmit_eid write");
    else
        printf("sent extended frame  ID=0x%08X\n", id);
}

// ========================================================
//  Receive — returns 1 on success, 0 on timeout, -1 on error
//  timeout_ms: pass 0 to poll without blocking
// ========================================================
int read_can_frame(struct can_frame *out, int timeout_ms)
{
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(can_sock, &rfds);

    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };

    int ret = select(can_sock + 1, &rfds, NULL, NULL, &tv);
    if (ret < 0)  { perror("select"); return -1; }
    if (ret == 0) return 0;   // timeout, nothing available

    ssize_t n = read(can_sock, out, sizeof(*out));
    return (n == sizeof(*out)) ? 1 : -1;
}

// ========================================================
//  MIT mode helpers
// ========================================================
static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    if (x > x_max) x = x_max;
    return (unsigned int)((x - x_min) * (float)(((1L << bits) - 1)) / span);
}

static float uint_to_float(uint16_t x_int, float x_min, float x_max, int bits)
{
    float span   = x_max - x_min;
    return (float)x_int * span / (float)((1L << bits) - 1) + x_min;
}

// ========================================================
//  Servo mode — motor commands
// ========================================================
void comm_can_set_duty(uint8_t controller_id, float duty)
{
    int32_t idx = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, (int32_t)(duty * 100000.0f), &idx);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buf, idx);
}

void comm_can_set_current(uint8_t controller_id, float current)
{
    int32_t idx = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, (int32_t)(current * 1000.0f), &idx);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buf, idx);
}

void comm_can_set_cb(uint8_t controller_id, float current)
{
    int32_t idx = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, (int32_t)(current * 1000.0f), &idx);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buf, idx);
}

void comm_can_set_rpm(uint8_t controller_id, float rpm)
{
    int32_t idx = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, (int32_t)rpm, &idx);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buf, idx);
}

void comm_can_set_pos(uint8_t controller_id, float pos)
{
    int32_t idx = 0;
    uint8_t buf[4];
    buffer_append_int32(buf, (int32_t)(pos * 10000.0f), &idx);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buf, idx);
}

void comm_can_set_origin(uint8_t controller_id, uint8_t set_origin_mode)
{
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_ORIGIN_HERE << 8),
                          &set_origin_mode, 0);
}

void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA)
{
    int32_t idx0 = 0, idx1 = 4;
    uint8_t buf[8];
    buffer_append_int32(buf, (int32_t)(pos * 10000.0f), &idx0);
    buffer_append_int16(buf, (int16_t)(spd / 10.0f),    &idx1);
    buffer_append_int16(buf, (int16_t)(RPA / 10.0f),    &idx1);
    comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buf, idx1);
}

// ========================================================
//  MIT mode — motor commands
// ========================================================
void set_mit_mode_run(uint8_t controller_id) {
    uint8_t buf[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc };
    comm_can_transmit_eid(controller_id, buf, 8);
}

void set_mit_mode_idle(uint8_t controller_id) {
    uint8_t buf[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd };
    comm_can_transmit_eid(controller_id, buf, 8);
}

void set_mit_current_position_zero(uint8_t controller_id) {
    uint8_t buf[8] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe };
    comm_can_transmit_eid(controller_id, buf, 8);
}

void pack_cmd(uint8_t controller_id,
              float p_des, float v_des,
              float kp,    float kd,
              float t_ff)
{
    unsigned int p_int  = float_to_uint(p_des, P_MIN,  P_MAX,  16);
    unsigned int v_int  = float_to_uint(v_des, V_MIN,  V_MAX,  12);
    unsigned int kp_int = float_to_uint(kp,   KP_MIN, KP_MAX,  12);
    unsigned int kd_int = float_to_uint(kd,   KD_MIN, KD_MAX,  12);
    unsigned int t_int  = float_to_uint(t_ff, T_MIN,  T_MAX,   12);

    uint8_t buf[8];
    buf[0] = p_int >> 8;
    buf[1] = p_int & 0xFF;
    buf[2] = v_int >> 4;
    buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    buf[4] = kp_int & 0xFF;
    buf[5] = kd_int >> 4;
    buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    buf[7] = t_int & 0xFF;

    comm_can_transmit_sid(controller_id, buf, 8);
}

// ========================================================
//  Receive parsers  (Serial.print → printf)
// ========================================================
void motor_receive_servo(const struct can_frame *rx)
{
    int16_t pos_int = (int16_t)((rx->data[0] << 8) | rx->data[1]);
    int16_t spd_int = (int16_t)((rx->data[2] << 8) | rx->data[3]);
    int16_t cur_int = (int16_t)((rx->data[4] << 8) | rx->data[5]);

    float motor_pos = pos_int * 0.1f;
    float motor_spd = spd_int * 10.0f;
    float motor_cur = cur_int * 0.01f;
    int8_t temp  = (int8_t)rx->data[6];
    int8_t error = (int8_t)rx->data[7];

    printf("SERVO  pos=%.2f  spd=%.1f  cur=%.3f  temp=%d  err=%d\n",
           motor_pos, motor_spd, motor_cur, temp, error);
}

void motor_receive_mit(const struct can_frame *rx)
{
    uint16_t pos_int = (uint16_t)((rx->data[1] << 8) | rx->data[2]);
    uint16_t spd_int = (uint16_t)((rx->data[3] << 4) | (rx->data[4] >> 4));
    uint16_t t_int   = (uint16_t)(((rx->data[4] & 0xF) << 8) | rx->data[5]);

    float motor_pos = uint_to_float(pos_int, P_MIN, P_MAX, 16);
    float motor_spd = uint_to_float(spd_int, V_MIN, V_MAX, 12);
    float motor_t   = uint_to_float(t_int,   T_MIN, T_MAX, 12);
    int8_t temp  = (int8_t)rx->data[6];
    int8_t error = (int8_t)rx->data[7];

    printf("MIT  pos=%.4f  spd=%.4f  torque=%.4f  temp=%d  err=%d\n",
           motor_pos, motor_spd, motor_t, temp, error);
}

// ========================================================
//  main  (replaces setup() + loop())
// ========================================================
int main(void)
{
    printf("run...\n");

    if (init_can("can0") < 0) {
        fprintf(stderr, "CAN init fault!\n");
        return 1;
    }
    printf("CAN init ok!\n");

    // --- one-time test frame (mirrors the setup() transmit) ---
    const uint8_t can_test[8] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
    comm_can_transmit_sid(0x00, can_test, 8);

    // --- optionally enter MIT mode ---
    // set_mit_mode_run(CAN_ID);
    usleep(100000);   // 100 ms delay, same as Arduino delay(100)

    // ---- loop -----------------------------------------------
    struct can_frame canMsg;

    while (1) {
        usleep(100000);  // 100 ms — same cadence as Arduino loop()

        // --- send a motor command here ---
        // Uncomment whichever mode you need:

        // comm_can_set_rpm(CAN_ID, 5000);
        // comm_can_set_duty(CAN_ID, 0.1f);
        // comm_can_set_current(CAN_ID, 1.0f);
        // comm_can_set_pos(CAN_ID, 180.0f);
        // comm_can_set_pos_spd(CAN_ID, 180.0f, 1000, 1000);
        // pack_cmd(CAN_ID, 0.0f, 1.0f, 0.0f, 1.0f, 0.0f);

        // --- receive any incoming frame (100 ms timeout) ---
        int ret = read_can_frame(&canMsg, 100);
        if (ret <= 0) continue;   // timeout or error, loop again

        int is_ext = (canMsg.can_id & CAN_EFF_FLAG) != 0;
        uint32_t raw_id = canMsg.can_id & (is_ext ? CAN_EFF_MASK : CAN_SFF_MASK);

        printf("%s ID=0x%X len=%d data=",
               is_ext ? "EID" : "SID", raw_id, canMsg.can_dlc);
        for (int i = 0; i < canMsg.can_dlc; i++)
            printf("%02X ", canMsg.data[i]);
        printf("\n");

        // Servo mode status reply
        if (raw_id == (0x2900 | CAN_ID))
            motor_receive_servo(&canMsg);

        // MIT mode status reply — uncomment if using MIT mode
        // motor_receive_mit(&canMsg);
    }

    close(can_sock);
    return 0;
}
