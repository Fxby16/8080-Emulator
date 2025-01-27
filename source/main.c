#include "i8080.h"
#include "utils.h"

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

int main()
{
    reset_cpu();

    struct binary_data tmp = load_binary("resources/test.bin");

    printf("Loaded %u bytes\n", tmp.size);

    memcpy(&cpu.memory[0x1000], tmp.ptr, tmp.size);
    free(tmp.ptr);

    execute(0x1000);

    printf("0x2000: %u\n", cpu.memory[0x2000]);
    printf("0x2001: %u\n", cpu.memory[0x2001]);
    printf("0x2002: %u\n", cpu.memory[0x2002]);

    printf("A: %u\n", cpu.registers[A]);

    return 0;
}