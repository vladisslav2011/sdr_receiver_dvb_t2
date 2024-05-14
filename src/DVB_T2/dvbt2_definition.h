/*
 *  Copyright 2020 Oleg Malyutin.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef DVBT2_DEFINITION
#define DVBT2_DEFINITION

#include <QCoreApplication>
//#include <QDebug>
#include <complex>
#include <math.h>

typedef std::complex<float> complex;

#define TRUE    1
#define FALSE   0
#define M_PI_X_2  (M_PIf32 * 2.0f)
// 8MHz bandwith
#define T_PERIOD            (1.0e-6f * 7.0f / 64.0f)    // elementary period (sec), for 8MHz bandwith
#define SAMPLE_RATE         (1.0f / T_PERIOD)           // sample rate (Hz)
#define HERTZ_PER_RADIAN     (SAMPLE_RATE / M_PI_X_2)   // sample rate (radian)
// 32K mode
#define FFT_32K             32768
#define FRAME_LEN_MAX       68                          //Maximum frame length in OFDM symbols,
                                                        //including P2 and data symbols,
                                                        //for 32K FFT sizes and guard-interval 1/128
#define SYMBOL_TYPE_P1      0
#define SYMBOL_TYPE_P2      1
#define SYMBOL_TYPE_DATA    2
#define SYMBOL_TYPE_FC      3

#define CHIPS               2624
#define MAX_ACTIVE_CARRIERS 27841

#define ROT_QPSK            0.506145483f
#define ROT_QAM16           0.293215314f
#define ROT_QAM64           0.150098316f
#define ROT_QAM256          0.062418810f
#define NORM_FACTOR_QPSK    0.707106781f
#define NORM_FACTOR_QAM16   0.316227766f
#define NORM_FACTOR_QAM64   0.15430335f
#define NORM_FACTOR_QAM256  0.076696499f
// LDPC Code
#define FEC_SIZE_NORMAL 64800
#define FEC_SIZE_SHORT  16200
#define LDPC_ENCODE_TABLE_LENGTH (FEC_SIZE_NORMAL * 10)

// L1 signaling data
#define L1_PRE_CELL 1840

enum dvbt2_code_rate_t {
    C1_2 = 0,
    C3_5,
    C2_3,
    C3_4,
    C4_5,
    C5_6,
};

enum dvbt2_constellation_t {
    MOD_QPSK = 0,
    MOD_16QAM,
    MOD_64QAM,
    MOD_256QAM,
};

enum dvbt2_rotation_t {
    ROTATION_OFF = 0,
    ROTATION_ON,
};

enum dvbt2_fectype_t {
    FECFRAME_SHORT = 0,
    FEC_FRAME_NORMAL,
};

enum dvbt2_streamtype_t {
    STREAMTYPE_TS = 0,
    STREAMTYPE_GS,
    STREAMTYPE_BOTH,
};

enum dvbt2_inputmode_t {
    INPUTMODE_NORMAL = 0,
    INPUTMODE_HIEFF,
};

enum dvbt2_extended_carrier_t {
    CARRIERS_NORMAL = 0,
    CARRIERS_EXTENDED,
};

enum dvbt2_carrier_type_t {
    DATA_CARRIER = 1,
    P2CARRIER,
    P2PAPR_CARRIER,
    TRPAPR_CARRIER,
    SCATTERED_CARRIER,
    CONTINUAL_CARRIER,
    P2CARRIER_INVERTED,
    SCATTERED_CARRIER_INVERTED,
    CONTINUAL_CARRIER_INVERTED,
};

enum dvbt2_preamble_t {
    T2_SISO = 0,
    T2_MISO,
    NON_T2,
    T2_LITE_SISO,
    T2_LITE_MISO,
};

enum dvbt2_fft_mode_t {
    FFTSIZE_2K = 0,
    FFTSIZE_8K,
    FFTSIZE_4K,
    FFTSIZE_1K,
    FFTSIZE_16K,
    FFTSIZE_32K,
    FFTSIZE_8K_T2GI,
    FFTSIZE_32K_T2GI,
    FFTSIZE_16K_T2GI = 11,
};

enum dvbt2_guardinterval_t {
    GI_1_32 = 0,
    GI_1_16,
    GI_1_8,
    GI_1_4,
    GI_1_128,
    GI_19_128,
    GI_19_256,
};

enum dvbt2_papr_t {
    PAPR_OFF = 0,
    PAPR_ACE,
    PAPR_TR,
    PAPR_BOTH,
};

enum dvbt2_l1constellation_t {
    L1_MOD_BPSK = 0,
    L1_MOD_QPSK,
    L1_MOD_16QAM,
    L1_MOD_64QAM,
};

enum dvbt2_pilotpattern_t {
    PP1 = 0,
    PP2,
    PP3,
    PP4,
    PP5,
    PP6,
    PP7,
    PP8,
};

enum dvbt2_version_t {
    VERSION_111 = 0,
    VERSION_121,
    VERSION_131,
};

enum dvbt2_reservedbiasbits_t {
    RESERVED_OFF = 0,
    RESERVED_ON,
};

enum dvbt2_l1scrambled_t {
    L1_SCRAMBLED_OFF = 0,
    L1_SCRAMBLED_ON,
};

enum dvbt2_misogroup_t {
    MISO_TX1 = 0,
    MISO_TX2,
};

enum dvbt2_showlevels_t {
    SHOWLEVELS_OFF = 0,
    SHOWLEVELS_ON,
};

enum dvbt2_inband_t {
    INBAND_OFF = 0,
    INBAND_ON,
};

enum dvbt2_equalization_t {
    EQUALIZATION_OFF = 0,
    EQUALIZATION_ON,
};

enum dvbt2_bandwidth_t {
    BANDWIDTH_1_7_MHZ = 0,
    BANDWIDTH_5_0_MHZ,
    BANDWIDTH_6_0_MHZ,
    BANDWIDTH_7_0_MHZ,
    BANDWIDTH_8_0_MHZ,
    BANDWIDTH_10_0_MHZ,
};

struct dvbt2_parameters{
    int preamble;
    int bandwidth;
    int miso;
    int miso_group;

    int fft_mode;
    int fft_size;
    int guard_interval_mode;
    int guard_interval_size;
    int carrier_mode;
    int l_nulls;
    int pilot_pattern;

    int papr_mode;
    int l1_mod;
    int l1_cod;
    int l1_fec_type;
    int l1_post_size;
    int l1_post_info_size;
    int c_p2;      //Number of available data cells in one P2 symbol
    int n_p2;      //Number of P2 symbols for each FFT size
    int c_data;    //Number of available data cells in one data symbol
    int c_fc;      //Number of available active cells in the frame closing symbol
    int n_fc;      //Number of data cells for the frame-closing symbol
    int k_total;   //Number of OFDM carriers
    int k_ext;     //Number of carriers added on each side of the spectrum in extended carrier mode
    int k_offset;
    int len_frame; //Number of OFDM symbols per T2-frame excluding P1
    int n_data;    //Number of data OFDM symbols per T2-frame, excluding P1 and P2
    int n_t2;
    int l_fc;       // Frame closing symbol = 1 is available, else = 0
    int t2_version;
};
struct l1_presignalling{
    int type = 0;
    int bwt_ext = 0;
    int s1 = 0;
    int s2_field1 = 0;
    int s2_field2 = 0;
    int l1_repetition_flag = 0;
    int guard_interval = 0;
    int papr = 0;
    int l1_post_mod = 0;
    int l1_cod = 0;
    int l1_fec_type = 0;
    int l1_post_size = 0;
    int l1_post_info_size = 0;
    int pilot_pattern = 0;
    int tx_id_availability = 0;
    int cell_id = 0;
    int network_id = 0;
    int t2_system_id = 0;
    int num_t2_frames = 0;
    int num_data_symbols = 0;
    int regen_flag = 0;
    int l1_post_extension = 0;
    int num_rf = 0;
    int current_rf_index = 0;
    int t2_version = 0;
    int l1_post_scrambled = 0;
    int t2_base_lite = 0;
    int reserved = 0;
    int crc_32 = 0;
};
typedef struct a_22{
    int rf_idx = 0;
    int frequency = 0;
}l1_postsignalling_rf;
typedef struct a_23{
    int id = 0;
    int plp_type = 0;
    int plp_payload_type = 0;
    int ff_flag = 0;
    int first_rf_idx = 0;
    int first_frame_idx = 0;
    int plp_group_id = 0;
    int plp_cod = 0;
    int plp_mod = 0;
    int plp_rotation = 0;
    int plp_fec_type = 0;
    int plp_num_blocks_max = 0;
    int frame_interval = 0;
    int time_il_length = 0;
    int time_il_type = 0;
    int in_band_a_flag = 0;
    int in_band_b_flag = 0;
    int reserved_1 = 0;
    int plp_mode = 0;
    int static_flag = 0;
    int static_padding_flag = 0;
}l1_postsignalling_plp;
typedef struct a_24{
    int aux_stream_type = 0;
    int aux_private_config = 0;
}l1_postsignalling_aux;
typedef struct a_25{
    int id = 0;
    int start = 0;
    int num_blocks = 0;
    int reserved_2 = 0;
}dynamic_plp;
typedef struct a_26{
    int frame_idx = 0;
    int sub_slice_interval = 0;
    int type_2_start = 0;
    int l1_change_counter = 0;
    int start_rf_idx = 0;
    int reserved_1 = 0;
    dynamic_plp* plp = nullptr;
    int reserved_3 = 0;
    int* aux_private_dyn  = nullptr;
}l1_postsignalling_dynamic;
struct l1_postsignalling{
    int sub_slices_per_frame = 0;
    int num_plp = 0;
    int num_aux = 0;
    int aux_config_rfu = 0;
    l1_postsignalling_rf* rf = nullptr;
    int fef_type = 0;
    int fef_length = 0;
    int fef_interval = 0;
    l1_postsignalling_plp* plp = nullptr;
    int fef_length_msb = 0;
    int reserved_2 = 0;
    l1_postsignalling_aux* aux = nullptr;
    l1_postsignalling_dynamic dyn;
    l1_postsignalling_dynamic dyn_next;
};
Q_DECLARE_METATYPE(l1_postsignalling)

const unsigned char pn_sequence_table[CHIPS / 8] =
{
    0x4D, 0xC2, 0xAF, 0x7B, 0xD8, 0xC3, 0xC9, 0xA1, 0xE7, 0x6C, 0x9A, 0x09, 0x0A, 0xF1, 0xC3, 0x11,
    0x4F, 0x07, 0xFC, 0xA2, 0x80, 0x8E, 0x94, 0x62, 0xE9, 0xAD, 0x7B, 0x71, 0x2D, 0x6F, 0x4A, 0xC8,
    0xA5, 0x9B, 0xB0, 0x69, 0xCC, 0x50, 0xBF, 0x11, 0x49, 0x92, 0x7E, 0x6B, 0xB1, 0xC9, 0xFC, 0x8C,
    0x18, 0xBB, 0x94, 0x9B, 0x30, 0xCD, 0x09, 0xDD, 0xD7, 0x49, 0xE7, 0x04, 0xF5, 0x7B, 0x41, 0xDE,
    0xC7, 0xE7, 0xB1, 0x76, 0xE1, 0x2C, 0x56, 0x57, 0x43, 0x2B, 0x51, 0xB0, 0xB8, 0x12, 0xDF, 0x0E,
    0x14, 0x88, 0x7E, 0x24, 0xD8, 0x0C, 0x97, 0xF0, 0x93, 0x74, 0xAD, 0x76, 0x27, 0x0E, 0x58, 0xFE,
    0x17, 0x74, 0xB2, 0x78, 0x1D, 0x8D, 0x38, 0x21, 0xE3, 0x93, 0xF2, 0xEA, 0x0F, 0xFD, 0x4D, 0x24,
    0xDE, 0x20, 0xC0, 0x5D, 0x0B, 0xA1, 0x70, 0x3D, 0x10, 0xE5, 0x2D, 0x61, 0xE0, 0x13, 0xD8, 0x37,
    0xAA, 0x62, 0xD0, 0x07, 0xCC, 0x2F, 0xD7, 0x6D, 0x23, 0xA3, 0xE1, 0x25, 0xBD, 0xE8, 0xA9, 0xA7,
    0xC0, 0x2A, 0x98, 0xB7, 0x02, 0x51, 0xC5, 0x56, 0xF6, 0x34, 0x1E, 0xBD, 0xEC, 0xB8, 0x01, 0xAA,
    0xD5, 0xD9, 0xFB, 0x8C, 0xBE, 0xA8, 0x0B, 0xB6, 0x19, 0x09, 0x65, 0x27, 0xA8, 0xC4, 0x75, 0xB3,
    0xD8, 0xDB, 0x28, 0xAF, 0x85, 0x43, 0xA0, 0x0E, 0xC3, 0x48, 0x0D, 0xFF, 0x1E, 0x2C, 0xDA, 0x9F,
    0x98, 0x5B, 0x52, 0x3B, 0x87, 0x90, 0x07, 0xAA, 0x5D, 0x0C, 0xE5, 0x8D, 0x21, 0xB1, 0x86, 0x31,
    0x00, 0x66, 0x17, 0xF6, 0xF7, 0x69, 0xEB, 0x94, 0x7F, 0x92, 0x4E, 0xA5, 0x16, 0x1E, 0xC2, 0xC0,
    0x48, 0x8B, 0x63, 0xED, 0x79, 0x93, 0xBA, 0x8E, 0xF4, 0xE5, 0x52, 0xFA, 0x32, 0xFC, 0x3F, 0x1B,
    0xDB, 0x19, 0x92, 0x39, 0x02, 0xBC, 0xBB, 0xE5, 0xDD, 0xAB, 0xB8, 0x24, 0x12, 0x6E, 0x08, 0x45,
    0x9C, 0xA6, 0xCF, 0xA0, 0x26, 0x7E, 0x52, 0x94, 0xA9, 0x8C, 0x63, 0x25, 0x69, 0x79, 0x1E, 0x60,
    0xEF, 0x65, 0x9A, 0xEE, 0x95, 0x18, 0xCD, 0xF0, 0x8D, 0x87, 0x83, 0x36, 0x90, 0xC1, 0xB7, 0x91,
    0x83, 0xED, 0x12, 0x7E, 0x53, 0x36, 0x0C, 0xD8, 0x65, 0x14, 0x85, 0x9A, 0x28, 0xB5, 0x49, 0x4F,
    0x51, 0xAA, 0x48, 0x82, 0x41, 0x9A, 0x25, 0xA2, 0xD0, 0x1A, 0x5F, 0x47, 0xAA, 0x27, 0x30, 0x1E,
    0x79, 0xA5, 0x37, 0x0C, 0xCB, 0x3E, 0x19, 0x7F
};
//-------------------------------------------------------------------------------------------
const int p2_papr_map_1k[10] =
{
    116, 130, 134, 157, 182, 256, 346, 478, 479, 532
};
//-------------------------------------------------------------------------------------------
const int p2_papr_map_2k[18] =
{
    113, 124, 262, 467, 479, 727, 803, 862, 910, 946,
    980, 1201, 1322, 1342, 1396, 1397, 1562, 1565
};
//-------------------------------------------------------------------------------------------
const int p2_papr_map_4k[36] =
{
    104, 116, 119, 163, 170, 173, 664, 886, 1064, 1151, 1196, 1264, 1531,
    1736, 1951, 1960, 2069, 2098, 2311, 2366, 2473, 2552, 2584, 2585, 2645,
    2774, 2846, 2882, 3004, 3034, 3107, 3127, 3148, 3191, 3283, 3289
};
//-------------------------------------------------------------------------------------------
const int p2_papr_map_8k[72] =
{
    106, 109, 110, 112, 115, 118, 133, 142, 163, 184, 206, 247, 445, 461,
    503, 565, 602, 656, 766, 800, 922, 1094, 1108, 1199, 1258, 1726, 1793,
    1939, 2128, 2714, 3185, 3365, 3541, 3655, 3770, 3863, 4066, 4190, 4282,
    4565, 4628, 4727, 4882, 4885, 5143, 5192, 5210, 5257, 5261, 5459, 5651,
    5809, 5830, 5986, 6020, 6076, 6253, 6269, 6410, 6436, 6467, 6475, 6509,
    6556, 6611, 6674, 6685, 6689, 6691, 6695, 6698, 6701
};
//-------------------------------------------------------------------------------------------
const int p2_papr_map_16k[144] =
{
    104, 106, 107, 109, 110, 112, 113, 115, 116, 118, 119, 121, 122, 125, 128,
    131, 134, 137, 140, 143, 161, 223, 230, 398, 482, 497, 733, 809, 850, 922,
    962, 1196, 1256, 1262, 1559, 1691, 1801, 1819, 1937, 2005, 2095, 2308, 2383,
    2408, 2425, 2428, 2479, 2579, 2893, 2902, 3086, 3554, 4085, 4127, 4139, 4151,
    4163, 4373, 4400, 4576, 4609, 4952, 4961, 5444, 5756, 5800, 6094, 6208, 6658,
    6673, 6799, 7208, 7682, 8101, 8135, 8230, 8692, 8788, 8933, 9323, 9449, 9478,
    9868, 10192, 10261, 10430, 10630, 10685, 10828, 10915, 10930, 10942, 11053,
    11185, 11324, 11369, 11468, 11507, 11542, 11561, 11794, 11912, 11974, 11978,
    12085, 12179, 12193, 12269, 12311, 12758, 12767, 12866, 12938, 12962, 12971,
    13099, 13102, 13105, 13120, 13150, 13280, 13282, 13309, 13312, 13321, 13381,
    13402, 13448, 13456, 13462, 13463, 13466, 13478, 13492, 13495, 13498, 13501,
    13502, 13504, 13507, 13510, 13513, 13514, 13516
};
//-------------------------------------------------------------------------------------------
const int p2_papr_map_32k[288] =
{
    104, 106, 107, 109, 110, 112, 113, 115, 118, 121, 124, 127, 130, 133, 136,
    139, 142, 145, 148, 151, 154, 157, 160, 163, 166, 169, 172, 175, 178, 181,
    184, 187, 190, 193, 196, 199, 202, 205, 208, 211, 404, 452, 455, 467, 509,
    539, 568, 650, 749, 1001, 1087, 1286, 1637, 1823, 1835, 1841, 1889, 1898,
    1901, 2111, 2225, 2252, 2279, 2309, 2315, 2428, 2452, 2497, 2519, 3109, 3154,
    3160, 3170, 3193, 3214, 3298, 3331, 3346, 3388, 3397, 3404, 3416, 3466, 3491,
    3500, 3572, 4181, 4411, 4594, 4970, 5042, 5069, 5081, 5086, 5095, 5104, 5320,
    5465, 5491, 6193, 6541, 6778, 6853, 6928, 6934, 7030, 7198, 7351, 7712, 7826,
    7922, 8194, 8347, 8350, 8435, 8518, 8671, 8861, 8887, 9199, 9980, 10031, 10240,
    10519, 10537, 10573, 10589, 11078, 11278, 11324, 11489, 11642, 12034, 12107, 12184,
    12295, 12635, 12643, 12941, 12995, 13001, 13133, 13172, 13246, 13514, 13522, 13939,
    14362, 14720, 14926, 15338, 15524, 15565, 15662, 15775, 16358, 16613, 16688, 16760,
    17003, 17267, 17596, 17705, 18157, 18272, 18715, 18994, 19249, 19348, 20221, 20855,
    21400, 21412, 21418, 21430, 21478, 21559, 21983, 21986, 22331, 22367, 22370, 22402,
    22447, 22535, 22567, 22571, 22660, 22780, 22802, 22844, 22888, 22907, 23021, 23057,
    23086, 23213, 23240, 23263, 23333, 23369, 23453, 23594, 24143, 24176, 24319, 24325,
    24565, 24587, 24641, 24965, 25067, 25094, 25142, 25331, 25379, 25465, 25553, 25589,
    25594, 25655, 25664, 25807, 25823, 25873, 25925, 25948, 26002, 26008, 26102, 26138,
    26141, 26377, 26468, 26498, 26510, 26512, 26578, 26579, 26588, 26594, 26597, 26608,
    26627, 26642, 26767, 26776, 26800, 26876, 26882, 26900, 26917, 26927, 26951, 26957,
    26960, 26974, 26986, 27010, 27013, 27038, 27044, 27053, 27059, 27061, 27074, 27076,
    27083, 27086, 27092, 27094, 27098, 27103, 27110, 27115, 27118, 27119, 27125, 27128,
    27130, 27133, 27134, 27140, 27143, 27145, 27146, 27148, 27149
};
//-------------------------------------------------------------------------------------------
const int tr_papr_map_1k[10] =
{
    109, 117, 122, 129, 139, 321, 350, 403, 459, 465
};
//-------------------------------------------------------------------------------------------
const int tr_papr_map_2k[18] =
{
    250, 404, 638, 677, 700, 712, 755, 952, 1125, 1145,
    1190, 1276, 1325, 1335, 1406, 1431, 1472, 1481
};
//-------------------------------------------------------------------------------------------
const int tr_papr_map_4k[36] =
{
    170, 219, 405, 501, 597, 654, 661, 745, 995, 1025, 1319, 1361, 1394,
    1623, 1658, 1913, 1961, 1971, 2106, 2117, 2222, 2228, 2246, 2254, 2361,
    2468, 2469, 2482, 2637, 2679, 2708, 2825, 2915, 2996, 3033, 3119
};
//-------------------------------------------------------------------------------------------
const int tr_papr_map_8k[72] =
{
    111, 115, 123, 215, 229, 392, 613, 658, 831, 842, 997, 1503, 1626, 1916,
    1924, 1961, 2233, 2246, 2302, 2331, 2778, 2822, 2913, 2927, 2963, 2994,
    3087, 3162, 3226, 3270, 3503, 3585, 3711, 3738, 3874, 3902, 4013, 4017,
    4186, 4253, 4292, 4339, 4412, 4453, 4669, 4910, 5015, 5030, 5061, 5170,
    5263, 5313, 5360, 5384, 5394, 5493, 5550, 5847, 5901, 5999, 6020, 6165,
    6174, 6227, 6245, 6314, 6316, 6327, 6503, 6507, 6545, 6565
};
//-------------------------------------------------------------------------------------------
const int tr_papr_map_16k[144] =
{
    109, 122, 139, 171, 213, 214, 251, 585, 763, 1012, 1021, 1077, 1148, 1472,
    1792, 1883, 1889, 1895, 1900, 2013, 2311, 2582, 2860, 2980, 3011, 3099, 3143,
    3171, 3197, 3243, 3257, 3270, 3315, 3436, 3470, 3582, 3681, 3712, 3767, 3802,
    3979, 4045, 4112, 4197, 4409, 4462, 4756, 5003, 5007, 5036, 5246, 5483, 5535,
    5584, 5787, 5789, 6047, 6349, 6392, 6498, 6526, 6542, 6591, 6680, 6688, 6785,
    6860, 7134, 7286, 7387, 7415, 7417, 7505, 7526, 7541, 7551, 7556, 7747, 7814,
    7861, 7880, 8045, 8179, 8374, 8451, 8514, 8684, 8698, 8804, 8924, 9027, 9113,
    9211, 9330, 9479, 9482, 9487, 9619, 9829, 10326, 10394, 10407, 10450, 10528,
    10671, 10746, 10774, 10799, 10801, 10912, 11113, 11128, 11205, 11379, 11459,
    11468, 11658, 11776, 11791, 11953, 11959, 12021, 12028, 12135, 12233, 12407,
    12441, 12448, 12470, 12501, 12548, 12642, 12679, 12770, 12788, 12899, 12923,
    12939, 13050, 13103, 13147, 13256, 13339, 13409
};
//-------------------------------------------------------------------------------------------
const int tr_papr_map_32k[288] =
{
    164, 320, 350, 521, 527, 578, 590, 619, 635, 651, 662, 664, 676, 691, 723,
    940, 1280, 1326, 1509, 1520, 1638, 1682, 1805, 1833, 1861, 1891, 1900, 1902,
    1949, 1967, 1978, 1998, 2006, 2087, 2134, 2165, 2212, 2427, 2475, 2555, 2874,
    3067, 3091, 3101, 3146, 3188, 3322, 3353, 3383, 3503, 3523, 3654, 3856, 4150,
    4158, 4159, 4174, 4206, 4318, 4417, 4629, 4631, 4875, 5104, 5106, 5111, 5131,
    5145, 5146, 5177, 5181, 5246, 5269, 5458, 5474, 5500, 5509, 5579, 5810, 5823,
    6058, 6066, 6098, 6411, 6741, 6775, 6932, 7103, 7258, 7303, 7413, 7586, 7591,
    7634, 7636, 7655, 7671, 7675, 7756, 7760, 7826, 7931, 7937, 7951, 8017, 8061,
    8071, 8117, 8317, 8321, 8353, 8806, 9010, 9237, 9427, 9453, 9469, 9525, 9558,
    9574, 9584, 9820, 9973, 10011, 10043, 10064, 10066, 10081, 10136, 10193, 10249,
    10511, 10537, 11083, 11350, 11369, 11428, 11622, 11720, 11924, 11974, 11979, 12944,
    12945, 13009, 13070, 13110, 13257, 13364, 13370, 13449, 13503, 13514, 13520, 13583,
    13593, 13708, 13925, 14192, 14228, 14235, 14279, 14284, 14370, 14393, 14407, 14422,
    14471, 14494, 14536, 14617, 14829, 14915, 15094, 15138, 15155, 15170, 15260, 15283,
    15435, 15594, 15634, 15810, 16178, 16192, 16196, 16297, 16366, 16498, 16501, 16861,
    16966, 17039, 17057, 17240, 17523, 17767, 18094, 18130, 18218, 18344, 18374, 18657,
    18679, 18746, 18772, 18779, 18786, 18874, 18884, 18955, 19143, 19497, 19534, 19679,
    19729, 19738, 19751, 19910, 19913, 20144, 20188, 20194, 20359, 20490, 20500, 20555,
    20594, 20633, 20656, 21099, 21115, 21597, 22139, 22208, 22244, 22530, 22547, 22562,
    22567, 22696, 22757, 22798, 22854, 22877, 23068, 23102, 23141, 23154, 23170, 23202,
    23368, 23864, 24057, 24215, 24219, 24257, 24271, 24325, 24447, 25137, 25590, 25702,
    25706, 25744, 25763, 25811, 25842, 25853, 25954, 26079, 26158, 26285, 26346, 26488,
    26598, 26812, 26845, 26852, 26869, 26898, 26909, 26927, 26931, 26946, 26975, 26991,
    27039
};
//-------------------------------------------------------------------------------------------
const int pp1_cp1[20] =
{
    116, 255, 285, 430, 518, 546, 601, 646, 744, 1662, 1893, 1995, 2322, 3309, 3351,
    3567, 3813, 4032, 5568, 5706
};
//-------------------------------------------------------------------------------------------
const int pp1_cp2[25] =
{
    1022, 1224, 1302, 1371, 1495, 2261, 2551, 2583, 2649, 2833, 2925, 3192, 4266, 5395,
    5710, 5881, 8164, 10568, 11069, 11560, 12631, 12946, 13954, 16745, 21494
};
//-------------------------------------------------------------------------------------------
const int pp1_cp5[44] =
{
    1369, 7013, 7215, 7284, 7649, 7818, 8025, 8382, 8733, 8880, 9249, 9432, 9771, 10107,
    10110, 10398, 10659, 10709, 10785, 10872, 11115, 11373, 11515, 11649, 11652, 12594,
    12627, 12822, 12984, 15760, 16612, 17500, 18358, 19078, 19930, 20261, 20422, 22124,
    22867, 23239, 24934, 25879, 26308, 26674
};
//-------------------------------------------------------------------------------------------
const int pp2_cp1[20] =
{
    116, 318, 390, 430, 474, 518, 601, 646, 708, 726, 1752, 1758, 1944, 2100, 2208, 2466,
    3792, 5322, 5454, 5640
};
//-------------------------------------------------------------------------------------------
const int pp2_cp2[22] =
{
    1022, 1092, 1369, 1416, 1446, 1495, 2598, 2833, 2928, 3144, 4410, 4800, 5710, 5881,
    6018, 6126, 10568, 11515, 12946, 13954, 15559, 16681
};
//-------------------------------------------------------------------------------------------
const int pp2_cp3[2] =
{
    2261, 8164
};
//-------------------------------------------------------------------------------------------
const int pp2_cp4[2] =
{
    10709, 19930
};
//-------------------------------------------------------------------------------------------
const int pp2_cp5[41] =
{
    6744, 7013, 7020, 7122, 7308, 7649, 7674, 7752, 7764, 8154, 8190, 8856, 8922, 9504,
    9702, 9882, 9924, 10032, 10092, 10266, 10302, 10494, 10530, 10716, 11016, 11076,
    11160, 11286, 11436, 11586, 12582, 13002, 17500, 18358, 19078, 22124, 23239, 24073,
    24934, 25879, 26308
};
//-------------------------------------------------------------------------------------------
const int pp2_cp6[88] =
{
    13164, 13206, 13476, 13530, 13536, 13764, 13848, 13938, 13968, 14028, 14190, 14316,
    14526, 14556, 14562, 14658, 14910, 14946, 15048, 15186, 15252, 15468, 15540, 15576,
    15630, 15738, 15840, 16350, 16572, 16806, 17028, 17064, 17250, 17472, 17784, 17838,
    18180, 18246, 18480, 18900, 18960, 19254, 19482, 19638, 19680, 20082, 20310, 20422,
    20454, 20682, 20874, 21240, 21284, 21444, 21450, 21522, 21594, 21648, 21696, 21738,
    22416, 22824, 23016, 23124, 23196, 23238, 23316, 23418, 23922, 23940, 24090, 24168,
    24222, 24324, 24342, 24378, 24384, 24540, 24744, 24894, 24990, 25002, 25194, 25218,
    25260, 25566, 26674, 26944
};
//-------------------------------------------------------------------------------------------
const int pp3_cp1[22] =
{
    116, 318, 342, 426, 430, 518, 582, 601, 646, 816, 1758, 1764, 2400, 3450, 3504,
    3888, 4020, 4932, 5154, 5250, 5292, 5334
};
//-------------------------------------------------------------------------------------------
const int pp3_cp2[20] =
{
    1022, 1495, 2261, 2551, 2802, 2820, 2833, 2922, 4422, 4752, 4884, 5710, 8164,
    10568, 11069, 11560, 12631, 12946, 16745, 21494
};
//-------------------------------------------------------------------------------------------
const int pp3_cp3[1] =
{
    13954
};
//-------------------------------------------------------------------------------------------
const int pp3_cp5[44] =
{
    1369, 5395, 5881, 6564, 6684, 7013, 7649, 8376, 8544, 8718, 8856, 9024, 9132, 9498,
    9774, 9840, 10302, 10512, 10566, 10770, 10914, 11340, 11418, 11730, 11742, 12180,
    12276, 12474, 12486, 15760, 16612, 17500, 18358, 19078, 19930, 20261, 20422, 22124,
    22867, 23239, 24934, 25879, 26308, 26674
};
//-------------------------------------------------------------------------------------------
const int pp3_cp6[49] =
{
    13320, 13350, 13524, 13566, 13980, 14148, 14340, 14964, 14982, 14994, 15462, 15546,
    15984, 16152, 16314, 16344, 16488, 16614, 16650, 16854, 17028, 17130, 17160, 17178,
    17634, 17844, 17892, 17958, 18240, 18270, 18288, 18744, 18900, 18930, 18990, 19014,
    19170, 19344, 19662, 19698, 20022, 20166, 20268, 20376, 20466, 20550, 20562, 20904,
    21468
};
//-------------------------------------------------------------------------------------------
const int pp4_cp1[20] =
{
    108, 116, 144, 264, 288, 430, 518, 564, 636, 646, 828, 2184, 3360, 3396, 3912, 4032,
    4932, 5220, 5676, 5688
};
//-------------------------------------------------------------------------------------------
const int pp4_cp2[23] =
{
    601, 1022, 1092, 1164, 1369, 1392, 1452, 1495, 2261, 2580, 2833, 3072, 4320, 4452,
    5710, 5881, 6048, 10568, 11515, 12946, 13954, 15559, 16681
};
//-------------------------------------------------------------------------------------------
const int pp4_cp3[1] =
{
    8164
};
//-------------------------------------------------------------------------------------------
const int pp4_cp4[2] =
{
    10709, 19930
};
//-------------------------------------------------------------------------------------------
const int pp4_cp5[44] =
{
    6612, 6708, 7013, 7068, 7164, 7224, 7308, 7464, 7649, 7656, 7716, 7752, 7812, 7860,
    8568, 8808, 8880, 9072, 9228, 9516, 9696, 9996, 10560, 10608, 10728, 11148, 11232,
    11244, 11496, 11520, 11664, 11676, 11724, 11916, 17500, 18358, 19078, 21284, 22124,
    23239, 24073, 24934, 25879, 26308
};
//-------------------------------------------------------------------------------------------
const int pp4_cp6[86] =
{
    13080, 13152, 13260, 13380, 13428, 13572, 13884, 13956, 14004, 14016, 14088, 14232,
    14304, 14532, 14568, 14760, 14940, 15168, 15288, 15612, 15684, 15888, 16236, 16320,
    16428, 16680, 16812, 16908, 17184, 17472, 17508, 17580, 17892, 17988, 18000, 18336,
    18480, 18516, 19020, 19176, 19188, 19320, 19776, 19848, 20112, 20124, 20184, 20388,
    20532, 20556, 20676, 20772, 21156, 21240, 21276, 21336, 21384, 21816, 21888, 22068,
    22092, 22512, 22680, 22740, 22800, 22836, 22884, 23304, 23496, 23568, 23640, 24120,
    24168, 24420, 24444, 24456, 24492, 24708, 24864, 25332, 25536, 25764, 25992, 26004,
    26674, 26944
};
//-------------------------------------------------------------------------------------------
const int pp5_cp1[19] =
{
    108, 116, 228, 430, 518, 601, 646, 804, 1644, 1680, 1752, 1800, 1836, 3288, 3660,
    4080, 4932, 4968, 5472
};
//-------------------------------------------------------------------------------------------
const int pp5_cp2[23] =
{
    852, 1022, 1495, 2508, 2551, 2604, 2664, 2736, 2833, 3120, 4248, 4512, 4836, 5710,
    5940, 6108, 8164, 10568, 11069, 11560, 12946, 13954, 21494
};
//-------------------------------------------------------------------------------------------
const int pp5_cp3[3] =
{
    648, 4644, 16745
};
//-------------------------------------------------------------------------------------------
const int pp5_cp4[1] =
{
    12631
};
//-------------------------------------------------------------------------------------------
const int pp5_cp5[44] =
{
    1369, 2261, 5395, 5881, 6552, 6636, 6744, 6900, 7032, 7296, 7344, 7464, 7644, 7649,
    7668, 7956, 8124, 8244, 8904, 8940, 8976, 9216, 9672, 9780, 10224, 10332, 10709,
    10776, 10944, 11100, 11292, 11364, 11496, 11532, 11904, 12228, 12372, 12816, 15760,
    16612, 17500, 19078, 22867, 25879
};
//-------------------------------------------------------------------------------------------
const int pp6_cp5[88] =
{
    116, 384, 408, 518, 601, 646, 672, 960, 1022, 1272, 1344, 1369, 1495, 1800, 2040,
    2261, 2833, 3192, 3240, 3768, 3864, 3984, 4104, 4632, 4728, 4752, 4944, 5184, 5232,
    5256, 5376, 5592, 5616, 5710, 5808, 5881, 6360, 6792, 6960, 7013, 7272, 7344, 7392,
    7536, 7649, 7680, 7800, 8064, 8160, 8164, 8184, 8400, 8808, 8832, 9144, 9648, 9696,
    9912, 10008, 10200, 10488, 10568, 10656, 10709, 11088, 11160, 11515, 11592, 12048,
    12264, 12288, 12312, 12552, 12672, 12946, 13954, 15559, 16681, 17500, 19078, 20422,
    21284, 22124, 23239, 24934, 25879, 26308, 26674
};
//-------------------------------------------------------------------------------------------
const int pp6_cp6[88] =
{
    13080, 13368, 13464, 13536, 13656, 13728, 13824, 14112, 14232, 14448, 14472, 14712,
    14808, 14952, 15000, 15336, 15360, 15408, 15600, 15624, 15648, 16128, 16296, 16320,
    16416, 16536, 16632, 16824, 16848, 17184, 17208, 17280, 17352, 17520, 17664, 17736,
    17784, 18048, 18768, 18816, 18840, 19296, 19392, 19584, 19728, 19752, 19776, 20136,
    20184, 20208, 20256, 21096, 21216, 21360, 21408, 21744, 21768, 22200, 22224, 22320,
    22344, 22416, 22848, 22968, 23016, 23040, 23496, 23688, 23904, 24048, 24168, 24360,
    24408, 24984, 25152, 25176, 25224, 25272, 25344, 25416, 25488, 25512, 25536, 25656,
    25680, 25752, 25992, 26016
};
//-------------------------------------------------------------------------------------------
const int pp7_cp1[15] =
{
    264, 360, 1848, 2088, 2112, 2160, 2256, 2280, 3936, 3960, 3984, 5016, 5136, 5208,
    5664
};
//-------------------------------------------------------------------------------------------
const int pp7_cp2[30] =
{
    116, 430, 518, 601, 646, 1022, 1296, 1368, 1369, 1495, 2833, 3024, 4416, 4608,
    4776, 5710, 5881, 6168, 7013, 8164, 10568, 10709, 11515, 12946, 15559, 23239,
    24934, 25879, 26308, 26674
};
//-------------------------------------------------------------------------------------------
const int pp7_cp3[5] =
{
    456, 480, 2261, 6072, 17500
};
//-------------------------------------------------------------------------------------------
const int pp7_cp4[3] =
{
    1008, 6120, 13954
};
//-------------------------------------------------------------------------------------------
const int pp7_cp5[35] =
{
    6984, 7032, 7056, 7080, 7152, 7320, 7392, 7536, 7649, 7704, 7728, 7752, 8088, 8952,
    9240, 9288, 9312, 9480, 9504, 9840, 9960, 10320, 10368, 10728, 10752, 11448, 11640,
    11688, 11808, 12192, 12240, 12480, 12816, 16681, 22124
};
//-------------------------------------------------------------------------------------------
const int pp7_cp6[92] =
{
    13416, 13440, 13536, 13608, 13704, 13752, 14016, 14040, 14112, 14208, 14304, 14376,
    14448, 14616, 14712, 14760, 14832, 14976, 15096, 15312, 15336, 15552, 15816, 15984,
    16224, 16464, 16560, 17088, 17136, 17256, 17352, 17400, 17448, 17544, 17928, 18048,
    18336, 18456, 18576, 18864, 19032, 19078, 19104, 19320, 19344, 19416, 19488, 19920,
    19930, 19992, 20424, 20664, 20808, 21168, 21284, 21360, 21456, 21816, 22128, 22200,
    22584, 22608, 22824, 22848, 22944, 22992, 23016, 23064, 23424, 23448, 23472, 23592,
    24192, 24312, 24360, 24504, 24552, 24624, 24648, 24672, 24768, 24792, 25080, 25176,
    25224, 25320, 25344, 25584, 25680, 25824, 26064, 26944
};
//-------------------------------------------------------------------------------------------
const int pp8_cp4[47] =
{
    116, 132, 180, 430, 518, 601, 646, 1022, 1266, 1369, 1495, 2261, 2490, 2551, 2712,
    2833, 3372, 3438, 4086, 4098, 4368, 4572, 4614, 4746, 4830, 4968, 5395, 5710, 5881,
    7649, 8164, 10568, 11069, 11560, 12631, 12946, 13954, 15760, 16612, 16745, 17500,
    19078, 19930, 21494, 22867, 25879, 26308
};
//-------------------------------------------------------------------------------------------
const int pp8_cp5[39] =
{
    6720, 6954, 7013, 7026, 7092, 7512, 7536, 7596, 7746, 7758, 7818, 7986, 8160, 8628,
    9054, 9096, 9852, 9924, 10146, 10254, 10428, 10704, 11418, 11436, 11496, 11550,
    11766, 11862, 12006, 12132, 12216, 12486, 12762, 18358, 20261, 20422, 22124,
    23239, 24934
};
//-------------------------------------------------------------------------------------------
const int pp8_cp6[89] =
{
    10709, 11515, 13254, 13440, 13614, 13818, 14166, 14274, 14304, 14364, 14586, 14664,
    15030, 15300, 15468, 15474, 15559, 15732, 15774, 16272, 16302, 16428, 16500, 16662,
    16681, 16872, 17112, 17208, 17862, 18036, 18282, 18342, 18396, 18420, 18426, 18732,
    19050, 19296, 19434, 19602, 19668, 19686, 19728, 19938, 20034, 21042, 21120, 21168,
    21258, 21284, 21528, 21594, 21678, 21930, 21936, 21990, 22290, 22632, 22788, 23052,
    23358, 23448, 23454, 23706, 23772, 24048, 24072, 24073, 24222, 24384, 24402, 24444,
    24462, 24600, 24738, 24804, 24840, 24918, 24996, 25038, 25164, 25314, 25380, 25470,
    25974, 26076, 26674, 26753, 26944
};
//-------------------------------------------------------------------------------------------
const int pp2_8k[4] =
{
    6820, 6847, 6869, 6898
};
//-------------------------------------------------------------------------------------------
const int pp3_8k[2] =
{
    6820, 6869
};
//-------------------------------------------------------------------------------------------
const int pp4_8k[2] =
{
    6820, 6869
};
//-------------------------------------------------------------------------------------------
const int pp7_8k[5] =
{
    6820, 6833, 6869, 6887, 6898
};
//-------------------------------------------------------------------------------------------
const int pp8_8k[5] =
{
    6820, 6833, 6869, 6887, 6898
};
//-------------------------------------------------------------------------------------------
const int pp1_16k[4] =
{
    3636, 13724, 13790, 13879
};
//-------------------------------------------------------------------------------------------
const int pp2_16k[2] =
{
    13636, 13790
};
//-------------------------------------------------------------------------------------------
const int pp3_16k[2] =
{
    13636, 13790
};
//-------------------------------------------------------------------------------------------
const int pp4_16k[2] =
{
    13636, 13790
};
//-------------------------------------------------------------------------------------------
const int pp5_16k[2] =
{
    13636, 13790
};
//-------------------------------------------------------------------------------------------
const int pp6_16k[2] =
{
    13636, 13790
};
//-------------------------------------------------------------------------------------------
const int pp7_16k[3] =
{
    13636, 13724, 13879
};
//-------------------------------------------------------------------------------------------
const int pp8_16k[3] =
{
    13636, 13724, 13879
};
//-------------------------------------------------------------------------------------------
const int pp2_32k[2] =
{
    27268, 27688
};
//-------------------------------------------------------------------------------------------
const int pp4_32k[2] =
{
    27268, 27688
};
//-------------------------------------------------------------------------------------------
const int pp6_32k[4] =
{
    27268, 27448, 27688, 27758
};
//-------------------------------------------------------------------------------------------
const int pp7_32k[2] =
{
    27268, 27688
};
//-------------------------------------------------------------------------------------------
const int pp8_32k[6] =
{
    27268, 27368, 27448, 27580, 27688, 27758
};
//-------------------------------------------------------------------------------------------
const int mux16[8] =
{
    7, 1, 3, 5, 2, 4, 6, 0
};

const int mux64[12] =
{
    11, 8, 5, 2, 10, 7, 4, 1, 9, 6, 3, 0
};
//-------------------------------------------------------------------------------------------

void dvbt2_p2_parameters_init(dvbt2_parameters &_dvbt2);
void dvbt2_bwt_ext_parameters_init(dvbt2_parameters &_dvbt2);
void dvbt2_data_parameters_init(dvbt2_parameters &_dvbt2);

#endif // DVBT2_DEFINITION

