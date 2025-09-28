#ifndef DFPLAYER_MINI_H
#define DFPLAYER_MINI_H

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

// DFP Player Commands
#define DFPLAYER_CMD_NEXT                   0x01
#define DFPLAYER_CMD_PREV                   0x02
#define DFPLAYER_CMD_PLAY                   0x03
#define DFPLAYER_CMD_VOLUME_UP              0x04
#define DFPLAYER_CMD_VOLUME_DOWN            0x05
#define DFPLAYER_CMD_VOLUME                 0x06
#define DFPLAYER_CMD_EQ                     0x07
#define DFPLAYER_CMD_PLAYBACK_MODE          0x08
#define DFPLAYER_CMD_SOURCE                 0x09
#define DFPLAYER_CMD_STANDBY                0x0A
#define DFPLAYER_CMD_NORMAL                 0x0B
#define DFPLAYER_CMD_RESET                  0x0C
#define DFPLAYER_CMD_PLAYBACK               0x0D
#define DFPLAYER_CMD_PAUSE                  0x0E
#define DFPLAYER_CMD_FOLDER                 0x0F
#define DFPLAYER_CMD_VOLUME_ADJUST          0x10
#define DFPLAYER_CMD_REPEAT_PLAY            0x11
#define DFPLAYER_CMD_PLAYBACK_DEVICE        0x12
#define DFPLAYER_CMD_INTERRUPT              0x13
#define DFPLAYER_CMD_STOP                   0x16
#define DFPLAYER_CMD_REPEAT_FOLDER          0x17
#define DFPLAYER_CMD_RANDOM_ALL             0x18
#define DFPLAYER_CMD_REPEAT_CURRENT         0x19
#define DFPLAYER_CMD_SET_DAC                0x1A
#define DFPLAYER_CMD_PLAYBACK_MODE_2        0x1B
#define DFPLAYER_CMD_PLAYBACK_MODE_3        0x1C
#define DFPLAYER_CMD_PLAYBACK_MODE_4        0x1D
#define DFPLAYER_CMD_PLAYBACK_MODE_5        0x1E
#define DFPLAYER_CMD_PLAYBACK_MODE_6        0x1F
#define DFPLAYER_CMD_PLAYBACK_MODE_7        0x20
#define DFPLAYER_CMD_PLAYBACK_MODE_8        0x21
#define DFPLAYER_CMD_PLAYBACK_MODE_9        0x22
#define DFPLAYER_CMD_PLAYBACK_MODE_10       0x23
#define DFPLAYER_CMD_PLAYBACK_MODE_11       0x24
#define DFPLAYER_CMD_PLAYBACK_MODE_12       0x25
#define DFPLAYER_CMD_PLAYBACK_MODE_13       0x26
#define DFPLAYER_CMD_PLAYBACK_MODE_14       0x27
#define DFPLAYER_CMD_PLAYBACK_MODE_15       0x28
#define DFPLAYER_CMD_PLAYBACK_MODE_16       0x29
#define DFPLAYER_CMD_PLAYBACK_MODE_17       0x2A
#define DFPLAYER_CMD_PLAYBACK_MODE_18       0x2B
#define DFPLAYER_CMD_PLAYBACK_MODE_19       0x2C
#define DFPLAYER_CMD_PLAYBACK_MODE_20       0x2D
#define DFPLAYER_CMD_PLAYBACK_MODE_21       0x2E
#define DFPLAYER_CMD_PLAYBACK_MODE_22       0x2F
#define DFPLAYER_CMD_PLAYBACK_MODE_23       0x30
#define DFPLAYER_CMD_PLAYBACK_MODE_24       0x31
#define DFPLAYER_CMD_PLAYBACK_MODE_25       0x32
#define DFPLAYER_CMD_PLAYBACK_MODE_26       0x33
#define DFPLAYER_CMD_PLAYBACK_MODE_27       0x34
#define DFPLAYER_CMD_PLAYBACK_MODE_28       0x35
#define DFPLAYER_CMD_PLAYBACK_MODE_29       0x36
#define DFPLAYER_CMD_PLAYBACK_MODE_30       0x37
#define DFPLAYER_CMD_PLAYBACK_MODE_31       0x38
#define DFPLAYER_CMD_PLAYBACK_MODE_32       0x39
#define DFPLAYER_CMD_PLAYBACK_MODE_33       0x3A
#define DFPLAYER_CMD_PLAYBACK_MODE_34       0x3B
#define DFPLAYER_CMD_PLAYBACK_MODE_35       0x3C
#define DFPLAYER_CMD_PLAYBACK_MODE_36       0x3D
#define DFPLAYER_CMD_PLAYBACK_MODE_37       0x3E
#define DFPLAYER_CMD_PLAYBACK_MODE_38       0x3F
#define DFPLAYER_CMD_PLAYBACK_MODE_39       0x40
#define DFPLAYER_CMD_PLAYBACK_MODE_40       0x41
#define DFPLAYER_CMD_PLAYBACK_MODE_41       0x42
#define DFPLAYER_CMD_PLAYBACK_MODE_42       0x43
#define DFPLAYER_CMD_PLAYBACK_MODE_43       0x44
#define DFPLAYER_CMD_PLAYBACK_MODE_44       0x45
#define DFPLAYER_CMD_PLAYBACK_MODE_45       0x46
#define DFPLAYER_CMD_PLAYBACK_MODE_46       0x47
#define DFPLAYER_CMD_PLAYBACK_MODE_47       0x48
#define DFPLAYER_CMD_PLAYBACK_MODE_48       0x49
#define DFPLAYER_CMD_PLAYBACK_MODE_49       0x4A
#define DFPLAYER_CMD_PLAYBACK_MODE_50       0x4B
#define DFPLAYER_CMD_PLAYBACK_MODE_51       0x4C
#define DFPLAYER_CMD_PLAYBACK_MODE_52       0x4D
#define DFPLAYER_CMD_PLAYBACK_MODE_53       0x4E
#define DFPLAYER_CMD_PLAYBACK_MODE_54       0x4F
#define DFPLAYER_CMD_PLAYBACK_MODE_55       0x50
#define DFPLAYER_CMD_PLAYBACK_MODE_56       0x51
#define DFPLAYER_CMD_PLAYBACK_MODE_57       0x52
#define DFPLAYER_CMD_PLAYBACK_MODE_58       0x53
#define DFPLAYER_CMD_PLAYBACK_MODE_59       0x54
#define DFPLAYER_CMD_PLAYBACK_MODE_60       0x55
#define DFPLAYER_CMD_PLAYBACK_MODE_61       0x56
#define DFPLAYER_CMD_PLAYBACK_MODE_62       0x57
#define DFPLAYER_CMD_PLAYBACK_MODE_63       0x58
#define DFPLAYER_CMD_PLAYBACK_MODE_64       0x59
#define DFPLAYER_CMD_PLAYBACK_MODE_65       0x5A
#define DFPLAYER_CMD_PLAYBACK_MODE_66       0x5B
#define DFPLAYER_CMD_PLAYBACK_MODE_67       0x5C
#define DFPLAYER_CMD_PLAYBACK_MODE_68       0x5D
#define DFPLAYER_CMD_PLAYBACK_MODE_69       0x5E
#define DFPLAYER_CMD_PLAYBACK_MODE_70       0x5F
#define DFPLAYER_CMD_PLAYBACK_MODE_71       0x60
#define DFPLAYER_CMD_PLAYBACK_MODE_72       0x61
#define DFPLAYER_CMD_PLAYBACK_MODE_73       0x62
#define DFPLAYER_CMD_PLAYBACK_MODE_74       0x63
#define DFPLAYER_CMD_PLAYBACK_MODE_75       0x64
#define DFPLAYER_CMD_PLAYBACK_MODE_76       0x65
#define DFPLAYER_CMD_PLAYBACK_MODE_77       0x66
#define DFPLAYER_CMD_PLAYBACK_MODE_78       0x67
#define DFPLAYER_CMD_PLAYBACK_MODE_79       0x68
#define DFPLAYER_CMD_PLAYBACK_MODE_80       0x69
#define DFPLAYER_CMD_PLAYBACK_MODE_81       0x6A
#define DFPLAYER_CMD_PLAYBACK_MODE_82       0x6B
#define DFPLAYER_CMD_PLAYBACK_MODE_83       0x6C
#define DFPLAYER_CMD_PLAYBACK_MODE_84       0x6D
#define DFPLAYER_CMD_PLAYBACK_MODE_85       0x6E
#define DFPLAYER_CMD_PLAYBACK_MODE_86       0x6F
#define DFPLAYER_CMD_PLAYBACK_MODE_87       0x70
#define DFPLAYER_CMD_PLAYBACK_MODE_88       0x71
#define DFPLAYER_CMD_PLAYBACK_MODE_89       0x72
#define DFPLAYER_CMD_PLAYBACK_MODE_90       0x73
#define DFPLAYER_CMD_PLAYBACK_MODE_91       0x74
#define DFPLAYER_CMD_PLAYBACK_MODE_92       0x75
#define DFPLAYER_CMD_PLAYBACK_MODE_93       0x76
#define DFPLAYER_CMD_PLAYBACK_MODE_94       0x77
#define DFPLAYER_CMD_PLAYBACK_MODE_95       0x78
#define DFPLAYER_CMD_PLAYBACK_MODE_96       0x79
#define DFPLAYER_CMD_PLAYBACK_MODE_97       0x7A
#define DFPLAYER_CMD_PLAYBACK_MODE_98       0x7B
#define DFPLAYER_CMD_PLAYBACK_MODE_99       0x7C
#define DFPLAYER_CMD_PLAYBACK_MODE_100      0x7D
#define DFPLAYER_CMD_PLAYBACK_MODE_101      0x7E
#define DFPLAYER_CMD_PLAYBACK_MODE_102      0x7F
#define DFPLAYER_CMD_PLAYBACK_MODE_103      0x80
#define DFPLAYER_CMD_PLAYBACK_MODE_104      0x81
#define DFPLAYER_CMD_PLAYBACK_MODE_105      0x82
#define DFPLAYER_CMD_PLAYBACK_MODE_106      0x83
#define DFPLAYER_CMD_PLAYBACK_MODE_107      0x84
#define DFPLAYER_CMD_PLAYBACK_MODE_108      0x85
#define DFPLAYER_CMD_PLAYBACK_MODE_109      0x86
#define DFPLAYER_CMD_PLAYBACK_MODE_110      0x87
#define DFPLAYER_CMD_PLAYBACK_MODE_111      0x88
#define DFPLAYER_CMD_PLAYBACK_MODE_112      0x89
#define DFPLAYER_CMD_PLAYBACK_MODE_113      0x8A
#define DFPLAYER_CMD_PLAYBACK_MODE_114      0x8B
#define DFPLAYER_CMD_PLAYBACK_MODE_115      0x8C
#define DFPLAYER_CMD_PLAYBACK_MODE_116      0x8D
#define DFPLAYER_CMD_PLAYBACK_MODE_117      0x8E
#define DFPLAYER_CMD_PLAYBACK_MODE_118      0x8F
#define DFPLAYER_CMD_PLAYBACK_MODE_119      0x90
#define DFPLAYER_CMD_PLAYBACK_MODE_120      0x91
#define DFPLAYER_CMD_PLAYBACK_MODE_121      0x92
#define DFPLAYER_CMD_PLAYBACK_MODE_122      0x93
#define DFPLAYER_CMD_PLAYBACK_MODE_123      0x94
#define DFPLAYER_CMD_PLAYBACK_MODE_124      0x95
#define DFPLAYER_CMD_PLAYBACK_MODE_125      0x96
#define DFPLAYER_CMD_PLAYBACK_MODE_126      0x97
#define DFPLAYER_CMD_PLAYBACK_MODE_127      0x98
#define DFPLAYER_CMD_PLAYBACK_MODE_128      0x99
#define DFPLAYER_CMD_PLAYBACK_MODE_129      0x9A
#define DFPLAYER_CMD_PLAYBACK_MODE_130      0x9B
#define DFPLAYER_CMD_PLAYBACK_MODE_131      0x9C
#define DFPLAYER_CMD_PLAYBACK_MODE_132      0x9D
#define DFPLAYER_CMD_PLAYBACK_MODE_133      0x9E
#define DFPLAYER_CMD_PLAYBACK_MODE_134      0x9F
#define DFPLAYER_CMD_PLAYBACK_MODE_135      0xA0
#define DFPLAYER_CMD_PLAYBACK_MODE_136      0xA1
#define DFPLAYER_CMD_PLAYBACK_MODE_137      0xA2
#define DFPLAYER_CMD_PLAYBACK_MODE_138      0xA3
#define DFPLAYER_CMD_PLAYBACK_MODE_139      0xA4
#define DFPLAYER_CMD_PLAYBACK_MODE_140      0xA5
#define DFPLAYER_CMD_PLAYBACK_MODE_141      0xA6
#define DFPLAYER_CMD_PLAYBACK_MODE_142      0xA7
#define DFPLAYER_CMD_PLAYBACK_MODE_143      0xA8
#define DFPLAYER_CMD_PLAYBACK_MODE_144      0xA9
#define DFPLAYER_CMD_PLAYBACK_MODE_145      0xAA
#define DFPLAYER_CMD_PLAYBACK_MODE_146      0xAB
#define DFPLAYER_CMD_PLAYBACK_MODE_147      0xAC
#define DFPLAYER_CMD_PLAYBACK_MODE_148      0xAD
#define DFPLAYER_CMD_PLAYBACK_MODE_149      0xAE
#define DFPLAYER_CMD_PLAYBACK_MODE_150      0xAF
#define DFPLAYER_CMD_PLAYBACK_MODE_151      0xB0
#define DFPLAYER_CMD_PLAYBACK_MODE_152      0xB1
#define DFPLAYER_CMD_PLAYBACK_MODE_153      0xB2
#define DFPLAYER_CMD_PLAYBACK_MODE_154      0xB3
#define DFPLAYER_CMD_PLAYBACK_MODE_155      0xB4
#define DFPLAYER_CMD_PLAYBACK_MODE_156      0xB5
#define DFPLAYER_CMD_PLAYBACK_MODE_157      0xB6
#define DFPLAYER_CMD_PLAYBACK_MODE_158      0xB7
#define DFPLAYER_CMD_PLAYBACK_MODE_159      0xB8
#define DFPLAYER_CMD_PLAYBACK_MODE_160      0xB9
#define DFPLAYER_CMD_PLAYBACK_MODE_161      0xBA
#define DFPLAYER_CMD_PLAYBACK_MODE_162      0xBB
#define DFPLAYER_CMD_PLAYBACK_MODE_163      0xBC
#define DFPLAYER_CMD_PLAYBACK_MODE_164      0xBD
#define DFPLAYER_CMD_PLAYBACK_MODE_165      0xBE
#define DFPLAYER_CMD_PLAYBACK_MODE_166      0xBF
#define DFPLAYER_CMD_PLAYBACK_MODE_167      0xC0
#define DFPLAYER_CMD_PLAYBACK_MODE_168      0xC1
#define DFPLAYER_CMD_PLAYBACK_MODE_169      0xC2
#define DFPLAYER_CMD_PLAYBACK_MODE_170      0xC3
#define DFPLAYER_CMD_PLAYBACK_MODE_171      0xC4
#define DFPLAYER_CMD_PLAYBACK_MODE_172      0xC5
#define DFPLAYER_CMD_PLAYBACK_MODE_173      0xC6
#define DFPLAYER_CMD_PLAYBACK_MODE_174      0xC7
#define DFPLAYER_CMD_PLAYBACK_MODE_175      0xC8
#define DFPLAYER_CMD_PLAYBACK_MODE_176      0xC9
#define DFPLAYER_CMD_PLAYBACK_MODE_177      0xCA
#define DFPLAYER_CMD_PLAYBACK_MODE_178      0xCB
#define DFPLAYER_CMD_PLAYBACK_MODE_179      0xCC
#define DFPLAYER_CMD_PLAYBACK_MODE_180      0xCD
#define DFPLAYER_CMD_PLAYBACK_MODE_181      0xCE
#define DFPLAYER_CMD_PLAYBACK_MODE_182      0xCF
#define DFPLAYER_CMD_PLAYBACK_MODE_183      0xD0
#define DFPLAYER_CMD_PLAYBACK_MODE_184      0xD1
#define DFPLAYER_CMD_PLAYBACK_MODE_185      0xD2
#define DFPLAYER_CMD_PLAYBACK_MODE_186      0xD3
#define DFPLAYER_CMD_PLAYBACK_MODE_187      0xD4
#define DFPLAYER_CMD_PLAYBACK_MODE_188      0xD5
#define DFPLAYER_CMD_PLAYBACK_MODE_189      0xD6
#define DFPLAYER_CMD_PLAYBACK_MODE_190      0xD7
#define DFPLAYER_CMD_PLAYBACK_MODE_191      0xD8
#define DFPLAYER_CMD_PLAYBACK_MODE_192      0xD9
#define DFPLAYER_CMD_PLAYBACK_MODE_193      0xDA
#define DFPLAYER_CMD_PLAYBACK_MODE_194      0xDB
#define DFPLAYER_CMD_PLAYBACK_MODE_195      0xDC
#define DFPLAYER_CMD_PLAYBACK_MODE_196      0xDD
#define DFPLAYER_CMD_PLAYBACK_MODE_197      0xDE
#define DFPLAYER_CMD_PLAYBACK_MODE_198      0xDF
#define DFPLAYER_CMD_PLAYBACK_MODE_199      0xE0
#define DFPLAYER_CMD_PLAYBACK_MODE_200      0xE1
#define DFPLAYER_CMD_PLAYBACK_MODE_201      0xE2
#define DFPLAYER_CMD_PLAYBACK_MODE_202      0xE3
#define DFPLAYER_CMD_PLAYBACK_MODE_203      0xE4
#define DFPLAYER_CMD_PLAYBACK_MODE_204      0xE5
#define DFPLAYER_CMD_PLAYBACK_MODE_205      0xE6
#define DFPLAYER_CMD_PLAYBACK_MODE_206      0xE7
#define DFPLAYER_CMD_PLAYBACK_MODE_207      0xE8
#define DFPLAYER_CMD_PLAYBACK_MODE_208      0xE9
#define DFPLAYER_CMD_PLAYBACK_MODE_209      0xEA
#define DFPLAYER_CMD_PLAYBACK_MODE_210      0xEB
#define DFPLAYER_CMD_PLAYBACK_MODE_211      0xEC
#define DFPLAYER_CMD_PLAYBACK_MODE_212      0xED
#define DFPLAYER_CMD_PLAYBACK_MODE_213      0xEE
#define DFPLAYER_CMD_PLAYBACK_MODE_214      0xEF
#define DFPLAYER_CMD_PLAYBACK_MODE_215      0xF0
#define DFPLAYER_CMD_PLAYBACK_MODE_216      0xF1
#define DFPLAYER_CMD_PLAYBACK_MODE_217      0xF2
#define DFPLAYER_CMD_PLAYBACK_MODE_218      0xF3
#define DFPLAYER_CMD_PLAYBACK_MODE_219      0xF4
#define DFPLAYER_CMD_PLAYBACK_MODE_220      0xF5
#define DFPLAYER_CMD_PLAYBACK_MODE_221      0xF6
#define DFPLAYER_CMD_PLAYBACK_MODE_222      0xF7
#define DFPLAYER_CMD_PLAYBACK_MODE_223      0xF8
#define DFPLAYER_CMD_PLAYBACK_MODE_224      0xF9
#define DFPLAYER_CMD_PLAYBACK_MODE_225      0xFA
#define DFPLAYER_CMD_PLAYBACK_MODE_226      0xFB
#define DFPLAYER_CMD_PLAYBACK_MODE_227      0xFC
#define DFPLAYER_CMD_PLAYBACK_MODE_228      0xFD
#define DFPLAYER_CMD_PLAYBACK_MODE_229      0xFE
#define DFPLAYER_CMD_PLAYBACK_MODE_230      0xFF

// DFP Player Response Types
#define DFPLAYER_RESPONSE_ERROR             0x40
#define DFPLAYER_RESPONSE_STATUS            0x3F
#define DFPLAYER_RESPONSE_PLAYBACK          0x3D
#define DFPLAYER_RESPONSE_VOLUME            0x3C
#define DFPLAYER_RESPONSE_EQ                0x3B
#define DFPLAYER_RESPONSE_PLAYBACK_MODE     0x3A
#define DFPLAYER_RESPONSE_SOURCE            0x39
#define DFPLAYER_RESPONSE_FOLDER            0x38
#define DFPLAYER_RESPONSE_TRACK             0x37
#define DFPLAYER_RESPONSE_TIME              0x36
#define DFPLAYER_RESPONSE_TOTAL_TRACKS      0x35
#define DFPLAYER_RESPONSE_TOTAL_FOLDERS     0x34
#define DFPLAYER_RESPONSE_TOTAL_FILES       0x33
#define DFPLAYER_RESPONSE_TOTAL_TIME        0x32
#define DFPLAYER_RESPONSE_TOTAL_VOLUME      0x31
#define DFPLAYER_RESPONSE_TOTAL_EQ          0x30

// DFP Player Error Codes
#define DFPLAYER_ERROR_FILE_NOT_FOUND       0x01
#define DFPLAYER_ERROR_ADVERTISE            0x02
#define DFPLAYER_ERROR_SERIAL_RECEIVING     0x03
#define DFPLAYER_ERROR_CHECKSUM             0x04
#define DFPLAYER_ERROR_FILE_INDEX           0x05
#define DFPLAYER_ERROR_FILE_NAME            0x06
#define DFPLAYER_ERROR_ADVERTISE_2          0x07
#define DFPLAYER_ERROR_ADVERTISE_3          0x08
#define DFPLAYER_ERROR_ADVERTISE_4          0x09
#define DFPLAYER_ERROR_ADVERTISE_5          0x0A

// DFP Player Sources
#define DFPLAYER_SOURCE_U_DISK              0x01
#define DFPLAYER_SOURCE_SD                  0x02
#define DFPLAYER_SOURCE_AUX                 0x03
#define DFPLAYER_SOURCE_SLEEP               0x04
#define DFPLAYER_SOURCE_FLASH               0x05

// DFP Player EQ Settings
#define DFPLAYER_EQ_NORMAL                  0x00
#define DFPLAYER_EQ_POP                     0x01
#define DFPLAYER_EQ_ROCK                    0x02
#define DFPLAYER_EQ_JAZZ                    0x03
#define DFPLAYER_EQ_CLASSIC                 0x04
#define DFPLAYER_EQ_BASS                    0x05

// DFP Player Playback Modes
#define DFPLAYER_MODE_REPEAT                0x00
#define DFPLAYER_MODE_FOLDER_REPEAT         0x01
#define DFPLAYER_MODE_SINGLE_REPEAT         0x02
#define DFPLAYER_MODE_RANDOM                0x03

// DFP Player Command Structure
typedef struct {
    uint8_t start_byte;      // 0x7E
    uint8_t version;         // 0xFF
    uint8_t length;          // 0x06
    uint8_t command;         // Command
    uint8_t feedback;        // 0x01 for feedback, 0x00 for no feedback
    uint8_t param1;          // Parameter 1
    uint8_t param2;          // Parameter 2
    uint8_t checksum1;       // Checksum
    uint8_t checksum2;       // Checksum
    uint8_t end_byte;        // 0xEF
} dfplayer_cmd_t;

// DFP Player Response Structure
typedef struct {
    uint8_t start_byte;      // 0x7E
    uint8_t version;         // 0xFF
    uint8_t length;          // 0x06
    uint8_t response_type;   // Response type
    uint8_t feedback;        // 0x01
    uint8_t param1;          // Parameter 1
    uint8_t param2;          // Parameter 2
    uint8_t checksum1;       // Checksum
    uint8_t checksum2;       // Checksum
    uint8_t end_byte;        // 0xEF
} dfplayer_response_t;

// DFP Player Status
typedef struct {
    bool is_playing;
    bool is_paused;
    bool is_stopped;
    uint8_t current_track;
    uint8_t current_volume;
    uint8_t current_eq;
    uint8_t current_source;
    uint8_t total_tracks;
    uint8_t total_folders;
    uint8_t total_files;
    uint16_t current_time;
    uint16_t total_time;
} dfplayer_status_t;

// Function Declarations
esp_err_t dfplayer_init(uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);
esp_err_t dfplayer_send_command(uint8_t command, uint8_t param1, uint8_t param2, bool feedback);
esp_err_t dfplayer_send_command_with_response(uint8_t command, uint8_t param1, uint8_t param2, dfplayer_response_t *response);
esp_err_t dfplayer_play(uint8_t track);
esp_err_t dfplayer_play_folder(uint8_t folder, uint8_t track);
esp_err_t dfplayer_pause(void);
esp_err_t dfplayer_resume(void);
esp_err_t dfplayer_stop(void);
esp_err_t dfplayer_next(void);
esp_err_t dfplayer_prev(void);
esp_err_t dfplayer_set_volume(uint8_t volume);
esp_err_t dfplayer_volume_up(void);
esp_err_t dfplayer_volume_down(void);
esp_err_t dfplayer_set_eq(uint8_t eq);
esp_err_t dfplayer_set_source(uint8_t source);
esp_err_t dfplayer_set_playback_mode(uint8_t mode);
esp_err_t dfplayer_reset(void);
esp_err_t dfplayer_standby(void);
esp_err_t dfplayer_normal(void);
esp_err_t dfplayer_get_status(dfplayer_status_t *status);
const char* dfplayer_get_error_description(uint8_t error_code);
uint16_t dfplayer_calculate_checksum(uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif // DFPLAYER_MINI_H 