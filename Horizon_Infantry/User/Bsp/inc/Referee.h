#ifndef __REFEREE_H
#define __REFEREE_H

#include "usart.h"
#include "string.h"
#include "stdint.h"
#include "CRC_DJI.h"

/* 帧长度 */
#define REFEREE_RXFRAME_LENGTH 256
#define FrameHeader_Length 5U
#define CMDID_Length 2U
#define CRC16_Length 2U

/* ==================== CMD ID ==================== */
enum Read_Cmd_ID_Typdef
{
    game_state                 = 0x0001,
    Match_results              = 0x0002,
    Robot_HP                   = 0x0003,
    Venue_Events               = 0x0101,
    Referee_warning            = 0x0104,
    Dart_fire                  = 0x0105,
    Robot_performan            = 0x0201,
    time_power                 = 0x0202,
    Robot_location             = 0x0203,
    Robot_buff                 = 0x0204,
    Damage_status              = 0x0206,
    time_shooting              = 0x0207,
    Allowable_ammo             = 0x0208,
    RFID_status                = 0x0209,
    Dart_directives            = 0x020A,
    Ground_location            = 0x020B,
    Radar_Marking              = 0x020C,
    Route_Informat             = 0x020D,
    Radar_Informat             = 0x020E,

    Robot_Interaction          = 0x0301,
    Custom_controller_to_robot = 0x0302,
    Minimap                    = 0x0303,
    Map_receive_radar          = 0x0305,
    Custom_controller_to_map   = 0x0306,
    Map_receive_path           = 0x0307,
    Map_receive_robot_info     = 0x0308,
    Robot_to_custom_controller = 0x0309,
    Robot_to_custom_client     = 0x0310,
    Custom_client_to_robot     = 0x0311
};

typedef struct __packed
{
    uint8_t  SOF;
    uint16_t DataLenth;
    uint8_t  Seq;
    uint8_t  CRC8;
} frame_header_R_Typdef;

typedef struct __packed
{
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
} game_status_t;

typedef struct __packed
{
    uint8_t winner;
} game_result_t;

typedef struct __packed
{
    uint16_t ally_1_robot_HP;
    uint16_t ally_2_robot_HP;
    uint16_t ally_3_robot_HP;
    uint16_t ally_4_robot_HP;
    uint16_t reserved;
    uint16_t ally_7_robot_HP;
    uint16_t ally_outpost_HP;
    uint16_t ally_base_HP;
} game_robot_HP_t;

/* 0x0101 场地事件数据 */
typedef struct __packed
{
    uint32_t supply_zone_status : 1;                 // bit 0: 己方补给区的占领状态
    uint32_t reserved_1 : 1;                         // bit 1: 保留位
    uint32_t supply_zone_status_rmul : 1;            // bit 2: 己方补给区的占领状态 RMUL
    uint32_t small_energy_mechanism_status : 2;      // bit 3-4: 己方小能量机关的激活状态
    uint32_t big_energy_mechanism_status : 2;        // bit 5-6: 己方大能量机关的激活状态
    uint32_t central_highland_status : 2;            // bit 7-8: 己方中央高地的占领状态
    uint32_t trapezoidal_highland_status : 2;        // bit 9-10: 己方梯形高地的占领状态
    uint32_t dart_hit_time : 9;                      // bit 11-19: 对方飞镖最后击中己方前哨站或基地的时间
    uint32_t dart_hit_target : 3;                    // bit 20-22: 对方飞镖最后击中的具体目标
    uint32_t center_buff_status : 2;                 // bit 23-24: 中心增益点的占领状态 RMUL
    uint32_t fortress_buff_status : 2;               // bit 25-26: 己方堡垒增益点的占领状态
    uint32_t outpost_buff_status : 2;                // bit 27-28: 己方前哨站增益点的占领状态
    uint32_t base_buff_status : 1;                   // bit 29: 己方基地增益点的占领状态
    uint32_t reserved_2 : 2;                         // bit 30-31: 保留位
} event_data_t;

typedef struct __packed
{
    uint8_t level;
    uint8_t offending_robot_id;
    uint8_t count;
} referee_warning_t;

/* 0x0105 飞镖发射相关数据 */
typedef struct __packed
{
    uint8_t dart_remaining_time;
    uint16_t dart_hit_target : 3;       // bit 0-2: 最近一次己方飞镖击中的目标
    uint16_t dart_hit_count : 3;        // bit 3-5: 对方最近被击中的目标累计被击中计次数
    uint16_t dart_selected_target : 3;  // bit 6-8: 飞镖此时选定的击打目标
    uint16_t reserved : 7;              // bit 9-15: 保留位
} dart_info_t;

/* 0x0201 机器人性能体系数据 */
typedef struct __packed
{
    uint8_t robot_id;
    uint8_t robot_level;
    uint16_t current_HP;
    uint16_t maximum_HP;
    uint16_t shooter_barrel_cooling_value;
    uint16_t shooter_barrel_heat_limit;
    uint16_t chassis_power_limit;
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
    uint8_t reserved_power_management : 5;       // 补齐一个字节，防止错位
} robot_status_t;

typedef struct __packed
{
    uint16_t reserved_1;
    uint16_t reserved_2;
    float reserved_3;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

typedef struct __packed
{
    float x;
    float y;
    float angle;
} robot_pos_t;

/* 0x0204 机器人增益和底盘能量数据 */
typedef struct __packed
{
    uint8_t recovery_buff;
    uint16_t cooling_buff;
    uint8_t defence_buff;
    uint8_t vulnerability_buff;
    uint16_t attack_buff;
    uint8_t energy_125 : 1;              // bit 0: >=125%
    uint8_t energy_100 : 1;              // bit 1: >=100%
    uint8_t energy_50 : 1;               // bit 2: >=50%
    uint8_t energy_30 : 1;               // bit 3: >=30%
    uint8_t energy_15 : 1;               // bit 4: >=15%
    uint8_t energy_5 : 1;                // bit 5: >=5%
    uint8_t energy_1 : 1;                // bit 6: >=1%
    uint8_t energy_reserved : 1;         // bit 7: 保留位
} buff_t;

typedef struct __packed
{
    uint8_t armor_id : 4;
    uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/* 0x0207 实时射击数据 */
typedef struct __packed
{
    uint8_t reserved_1 : 1;              // bit 0: 保留位
    uint8_t bullet_type_17mm : 1;        // bit 1: 17mm 弹丸
    uint8_t bullet_type_42mm : 1;        // bit 2: 42mm 弹丸
    uint8_t reserved_2 : 5;              // bit 3-7: 保留位
    uint8_t shooter_number;
    uint8_t launching_frequency;
    float initial_speed;
} shoot_data_t;

typedef struct __packed
{
    uint16_t projectile_allowance_17mm;
    uint16_t projectile_allowance_42mm;
    uint16_t remaining_gold_coin;
    uint16_t projectile_allowance_fortress;
} projectile_allowance_t;

/* 0x0209 机器人 RFID 模块状态 */
typedef struct __packed
{
    // 32-bit (uint32_t rfid_status 展开)
    uint32_t ally_base : 1;
    uint32_t ally_central_highland : 1;
    uint32_t enemy_central_highland : 1;
    uint32_t ally_trapezoidal_highland : 1;
    uint32_t enemy_trapezoidal_highland : 1;
    uint32_t ally_fly_ramp_front : 1;
    uint32_t ally_fly_ramp_back : 1;
    uint32_t enemy_fly_ramp_front : 1;
    uint32_t enemy_fly_ramp_back : 1;
    uint32_t ally_central_highland_lower : 1;
    uint32_t ally_central_highland_upper : 1;
    uint32_t enemy_central_highland_lower : 1;
    uint32_t enemy_central_highland_upper : 1;
    uint32_t ally_highway_lower : 1;
    uint32_t ally_highway_upper : 1;
    uint32_t enemy_highway_lower : 1;
    uint32_t enemy_highway_upper : 1;
    uint32_t ally_fortress : 1;
    uint32_t ally_outpost : 1;
    uint32_t ally_supply_zone_not_overlapping : 1;
    uint32_t ally_supply_zone_overlapping : 1;
    uint32_t ally_assembly_zone : 1;
    uint32_t enemy_assembly_zone : 1;
    uint32_t center_buff_zone : 1;
    uint32_t enemy_fortress : 1;
    uint32_t enemy_outpost : 1;
    uint32_t ally_tunnel_highway_lower : 1;
    uint32_t ally_tunnel_highway_middle : 1;
    uint32_t ally_tunnel_highway_upper : 1;
    uint32_t ally_tunnel_trapezoidal_lower : 1;
    uint32_t ally_tunnel_trapezoidal_middle : 1;
    uint32_t ally_tunnel_trapezoidal_upper : 1;

    // 8-bit (uint8_t rfid_status_2 展开)
    uint8_t enemy_tunnel_highway_lower : 1;
    uint8_t enemy_tunnel_highway_middle : 1;
    uint8_t enemy_tunnel_highway_upper : 1;
    uint8_t enemy_tunnel_trapezoidal_lower : 1;
    uint8_t enemy_tunnel_trapezoidal_middle : 1;
    uint8_t enemy_tunnel_trapezoidal_upper : 1;
    uint8_t reserved_tunnel : 2;
} rfid_status_t;

typedef struct __packed
{
    uint8_t dart_launch_opening_status;
    uint8_t reserved;
    uint16_t target_change_time;
    uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

typedef struct __packed
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float reserved_1;
    float reserved_2;
} ground_robot_position_t;

/* 0x020C 雷达标记进度数据 */
typedef struct __packed
{
    uint16_t enemy_hero_mark : 1;          // bit 0
    uint16_t enemy_engineer_mark : 1;      // bit 1
    uint16_t enemy_infantry_3_mark : 1;    // bit 2
    uint16_t enemy_infantry_4_mark : 1;    // bit 3
    uint16_t enemy_aerial_mark : 1;        // bit 4
    uint16_t enemy_sentry_mark : 1;        // bit 5
    uint16_t ally_hero_mark : 1;           // bit 6
    uint16_t ally_engineer_mark : 1;       // bit 7
    uint16_t ally_infantry_3_mark : 1;     // bit 8
    uint16_t ally_infantry_4_mark : 1;     // bit 9
    uint16_t ally_aerial_mark : 1;         // bit 10
    uint16_t ally_sentry_mark : 1;         // bit 11
    uint16_t reserved : 4;                 // bit 12-15
} radar_mark_data_t;

/* 0x020D 哨兵自主决策信息同步 */
typedef struct __packed
{
    // uint32_t sentry_info 展开
    uint32_t sentry_redeemed_projectile_allowance : 11;
    uint32_t sentry_remote_redeemed_projectile_times : 4;
    uint32_t sentry_remote_redeemed_HP_times : 4;
    uint32_t can_confirm_free_revive : 1;
    uint32_t can_redeem_immediate_revive : 1;
    uint32_t cost_for_immediate_revive : 10;
    uint32_t reserved_1 : 1;

    // uint16_t sentry_info_2 展开
    uint16_t is_out_of_combat : 1;
    uint16_t remaining_redeemable_projectile_allowance : 11;
    uint16_t sentry_posture : 2;
    uint16_t can_enter_activating_state : 1;
    uint16_t reserved_2 : 1;
} sentry_info_t;

/* 0x020E 雷达自主决策信息同步 */
typedef struct __packed
{
    uint8_t radar_double_damage_chance : 2;   // bit 0-1
    uint8_t enemy_is_double_damaged : 1;      // bit 2
    uint8_t ally_encryption_level : 2;        // bit 3-4
    uint8_t can_modify_password : 1;          // bit 5
    uint8_t reserved : 2;                     // bit 6-7
} radar_info_t;

typedef struct __packed
{
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[112];
} robot_interaction_data_t;

/* 图形结构 (您原本的代码已经是正确的位域，此处保留) */
typedef struct __packed
{
    uint8_t figure_name[3];
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
} interaction_figure_t;

/* UI字符 */
typedef struct __packed
{
    uint16_t data_id;
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t Character_configuration[15];
    uint8_t Character[30];
} graphic_data_struct_t;

typedef struct __packed
{
    float target_position_x;
    float target_position_y;
    uint8_t cmd_keyboard;
    uint8_t target_robot_id;
    uint16_t cmd_source;
} map_command_t;

/* 0x0120 哨兵自主决策指令 */
typedef struct __packed
{
    uint32_t confirm_resurrection : 1;                      // bit 0
    uint32_t confirm_immediate_resurrection : 1;            // bit 1
    uint32_t redeem_projectile_allowance : 11;              // bit 2-12
    uint32_t remote_redeem_projectile_request_times : 4;    // bit 13-16
    uint32_t remote_redeem_HP_request_times : 4;            // bit 17-20
    uint32_t sentry_posture_cmd : 2;                        // bit 21-22
    uint32_t confirm_activate_energy_mechanism : 1;         // bit 23
    uint32_t reserved : 8;                                  // bit 24-31
} sentry_cmd_t;

/* 0x0121 雷达自主决策指令 */
typedef struct __packed
{
    uint8_t radar_double_damage_cmd;
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
} radar_cmd_t;

typedef struct __packed
{
    uint16_t hero_position_x;
    uint16_t hero_position_y;
    uint16_t engineer_position_x;
    uint16_t engineer_position_y;
    uint16_t infantry_3_position_x;
    uint16_t infantry_3_position_y;
    uint16_t infantry_4_position_x;
    uint16_t infantry_4_position_y;
    uint16_t reserved_1;
    uint16_t reserved_2;
    uint16_t sentry_position_x;
    uint16_t sentry_position_y;
} map_robot_data_t;

typedef struct __packed
{
    uint8_t intention;
    uint16_t start_position_x;
    uint16_t start_position_y;
    int8_t delta_x[49];
    int8_t delta_y[49];
    uint16_t sender_id;
} map_data_t;

typedef struct __packed
{
    uint16_t sender_id;
    uint16_t receiver_id;
    uint8_t user_data[30];
} custom_info_t;

/* 0x0306 自定义控制器键鼠操作 */
typedef struct __packed
{
    uint16_t key_value;
    uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
    uint16_t reserved;
} custom_client_data_t;

typedef struct __packed
{
    game_status_t game_status;
    game_result_t game_result;
    game_robot_HP_t game_robot_HP;

    referee_warning_t referee_warning;
    dart_info_t dart_info;
    robot_status_t robot_status;
    power_heat_data_t power_heat_data;
    robot_pos_t robot_pos;
    buff_t buff;
    hurt_data_t hurt_data;
    shoot_data_t shoot_data;
    projectile_allowance_t projectile_allowance;
    rfid_status_t rfid_status;
    dart_client_cmd_t dart_client_cmd;

    ground_robot_position_t ground_robot_position;
    radar_mark_data_t radar_mark_data;
    sentry_info_t sentry_info;
    radar_info_t radar_info;

    map_command_t map_command;
    event_data_t event_data;
    custom_info_t custom_info;

} User_Data_T;

typedef union
{
    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
    } RX_Data_head;

    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        game_status_t game_status;
        uint16_t frame_tail;
    } RX_Data_game_status;

    struct __packed
    {
        frame_header_R_Typdef frame_header;
        uint16_t read_cmd_id;
        robot_status_t robot_status;
        uint16_t frame_tail;
    } RX_Data_robot_status;

    uint8_t Data[255];

} ALL_RX_Data_T;

extern uint8_t Referee_Rx_Buf[2][REFEREE_RXFRAME_LENGTH];
extern User_Data_T User_data;

void Referee_System_Frame_Update(uint8_t *Buff, uint16_t Size);

void Referee_Send_KeyMouse(custom_client_data_t *control_data);

void Referee_Send_Data(uint16_t cmd_id, uint8_t *p_data, uint16_t len);

#endif