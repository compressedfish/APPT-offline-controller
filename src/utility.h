#ifndef __cplusplus
extern "C" {
#endif

extern int clamp(int n, int minn, int maxn);
extern bool split(char* input, char** next, char delim);
extern void parse_integers(char* s, int* nums, uint8_t maxnums);
extern void print(char *i);

#ifndef __cplusplus
}
#endif