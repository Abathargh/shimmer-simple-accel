/*
*   shimmer-listener presentation protocol
*
*   this is a 112B struct that is used to inform the BT master
*   about the format of the data that is being transmitted during 
*   the communication.
*
*
*/

#define MAX_FMT_LEN 10
#define MAX_STR_LEN 10
#define MAX_KEY_NUM 10

typedef char key_string[MAX_STR_LEN];

typedef struct {
    uint8_t framesize;
    uint8_t chunklen;
    char format[MAX_FMT_LEN];
    key_string keys[MAX_KEY_NUM];
} frameinfo;