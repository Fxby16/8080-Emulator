#ifndef UTILS_H
#define UTILS_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

struct binary_data{
    uint8_t* ptr;
    size_t size;
};

extern bool calc_parity(uint8_t n);
extern bool sum_set_ac(uint8_t op1, uint8_t op2);
extern bool sub_set_ac(uint8_t op1, uint8_t op2);
extern struct binary_data load_binary(const char* filepath); //returned pointer needs to be freed
extern const char* inst_from_opcode(uint8_t opcode);

#endif