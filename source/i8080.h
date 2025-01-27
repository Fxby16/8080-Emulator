#ifndef i8080_H
#define i8080_H

#include <stdint.h>
#include <stdbool.h>

#define MEM_SIZE (1 << 16)
#define NUM_IN_PORTS 256
#define NUM_OUT_PORTS 256

#define M 110 //memory reference operand. address HL.

enum{
    B, C, D, E, H, L, A, NUM_REGISTERS
};

#define B_C 0
#define D_E 1
#define H_L 2
#define SP 3
#define PSW 4

/*
CY = carry
P = parity
AC = auxiliar carry
Z = zero flag
S = sign flag
*/
enum flags{
    CY, P = 2, AC = 4, Z = 6, S
};

typedef struct{
    uint8_t registers[NUM_REGISTERS];
    uint8_t memory[MEM_SIZE];
    uint8_t in_ports[NUM_IN_PORTS];
    uint8_t out_ports[NUM_OUT_PORTS];
    uint16_t pc;
    uint16_t sp;
    uint8_t status_register;
    bool int_enable;
} cpu_data;

extern cpu_data cpu;

extern void reset_cpu();
extern void execute(uint16_t pc);
extern void update_flags_aux(uint16_t result, bool auxiliary_carry);
extern void update_flags_cy(uint16_t result);
extern void update_flags_aux_cy(uint16_t result, bool auxiliary_carry);
extern void dump_cpu();

#endif