#include "i8080.h"
#include "instructions.h"
#include "utils.h"

#include <string.h>
#include <assert.h>
#include <stdio.h>

cpu_data cpu;

void reset_cpu()
{
    memset(cpu.registers, 0, sizeof(cpu.registers[0]) * NUM_REGISTERS);
    memset(cpu.memory, 0, sizeof(cpu.memory[0]) * MEM_SIZE);
    memset(cpu.in_ports, 0, sizeof(cpu.in_ports[0]) * NUM_IN_PORTS);
    memset(cpu.out_ports, 0, sizeof(cpu.out_ports[0]) * NUM_OUT_PORTS);
    cpu.pc = 0;
    cpu.sp = 0;
    cpu.status_register = 0;
    cpu.int_enable = false;
}

void execute(uint16_t pc)
{
    cpu.pc = pc;

    while(true){
        uint8_t opcode = cpu.memory[cpu.pc++];
        uint8_t dst_reg = (opcode >> 3) & 0x07;
        uint8_t src_reg = opcode & 0x07;
        uint8_t dst_reg_pair = (opcode >> 4) & 0x03;
        
        printf("%s - PC: %x\n", inst_from_opcode(opcode), cpu.pc);

        switch(opcode){
            case 0x00: // NOP
                break;
            case 0x01: // LXI B, D16
                LXI(B_C, (cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x02: // STAX B
                STAX(B_C);
                break;
            case 0x03: // INX B
                INX(B_C);
                break;
            case 0x04: // INR B
                INR(B);
                break;
            case 0x05: // DCR B
                DCR(B);
                break;
            case 0x06: // MVI B, D8
                MVI(B, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x07: // RLC
                RLC();
                break;
            case 0x09: // DAD B
                DAD(B_C);
                break;
            case 0x0A: // LDAX B
                LDAX(B_C);
                break;
            case 0x0B: // DCX B
                DCX(B_C);
                break;
            case 0x0C: // INR C
                INR(C);
                break;
            case 0x0D: // DCR C
                DCR(C);
                break;
            case 0x0E: // MVI C, D8
                MVI(C, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x0F: // RRC
                RRC();
                break;
            case 0x11: // LXI D, D16
                LXI(D_E, (cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x12: // STAX D
                STAX(D_E);
                break;
            case 0x13: // INX D
                INX(D_E);
                break;
            case 0x14: // INR D
                INR(D);
                break;
            case 0x15: // DCR D
                DCR(D);
                break;
            case 0x16: // MVI D, D8
                MVI(D, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x17: // RAL
                RAL();
                break;
            case 0x19: // DAD D
                DAD(D_E);
                break;
            case 0x1A: // LDAX D
                LDAX(D_E);
                break;
            case 0x1B: // DCX D
                DCX(D_E);
                break;
            case 0x1C: // INR E
                INR(E);
                break;
            case 0x1D: // DCR E
                DCR(E);
                break;
            case 0x1E: // MVI E, D8
                MVI(E, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x1F: // RAR
                RAR();
                break;
            case 0x21: // LXI H, D16
                LXI(H_L, (cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x22: // SHLD adr
                SHLD((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x23: // INX H
                INX(H_L);
                break;
            case 0x24: // INR H
                INR(H);
                break;
            case 0x25: // DCR H
                DCR(H);
                break;
            case 0x26: // MVI H, D8
                MVI(H, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x27: // DAA
                DAA();
                break;
            case 0x29: // DAD H
                DAD(H_L);
                break;
            case 0x2A: // LHLD adr
                LHLD((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x2B: // DCX H
                DCX(H_L);
                break;
            case 0x2C: // INR L
                INR(L);
                break;
            case 0x2D: // DCR L
                DCR(L);
                break;
            case 0x2E: // MVI L, D8
                MVI(L, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x2F: // CMA
                CMA();
                break;
            case 0x31: // LXI SP, D16
                LXI(SP, (cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x32: // STA adr
                STA((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x33: // INX SP
                INX(SP);
                break;
            case 0x34: // INR M
                INR(M);
                break;
            case 0x35: // DCR M
                DCR(M);
                break;
            case 0x36: // MVI M, D8
                MVI(M, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x37: // STC
                STC();
                break;
            case 0x39: // DAD SP
                DAD(SP);
                break;
            case 0x3A: // LDA adr
                LDA((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0x3B: // DCX SP
                DCX(SP);
                break;
            case 0x3C: // INR A
                INR(A);
                break;
            case 0x3D: // DCR A
                DCR(A);
                break;
            case 0x3E: // MVI A, D8
                MVI(A, cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0x3F: // CMC
                CMC();
                break;
            case 0x40: // MOV B, B
                MOV(B, B);
                break;
            case 0x41: // MOV B, C
                MOV(B, C);
                break;
            case 0x42: // MOV B, D
                MOV(B, D);
                break;
            case 0x43: // MOV B, E
                MOV(B, E);
                break;
            case 0x44: // MOV B, H
                MOV(B, H);
                break;
            case 0x45: // MOV B, L
                MOV(B, L);
                break;
            case 0x46: // MOV B, M
                MOV(B, M);
                break;
            case 0x47: // MOV B, A
                MOV(B, A);
                break;
            case 0x48: // MOV C, B
                MOV(C, B);
                break;
            case 0x49: // MOV C, C
                MOV(C, C);
                break;
            case 0x4A: // MOV C, D
                MOV(C, D);
                break;
            case 0x4B: // MOV C, E
                MOV(C, E);
                break;
            case 0x4C: // MOV C, H
                MOV(C, H);
                break;
            case 0x4D: // MOV C, L
                MOV(C, L);
                break;
            case 0x4E: // MOV C, M
                MOV(C, M);
                break;
            case 0x4F: // MOV C, A
                MOV(C, A);
                break;
            case 0x50: // MOV D, B
                MOV(D, B);
                break;
            case 0x51: // MOV D, C
                MOV(D, C);
                break;
            case 0x52: // MOV D, D
                MOV(D, D);
                break;
            case 0x53: // MOV D, E
                MOV(D, E);
                break;
            case 0x54: // MOV D, H
                MOV(D, H);
                break;
            case 0x55: // MOV D, L
                MOV(D, L);
                break;
            case 0x56: // MOV D, M
                MOV(D, M);
                break;
            case 0x57: // MOV D, A
                MOV(D, A);
                break;
            case 0x58: // MOV E, B
                MOV(E, B);
                break;
            case 0x59: // MOV E, C
                MOV(E, C);
                break;
            case 0x5A: // MOV E, D
                MOV(E, D);
                break;
            case 0x5B: // MOV E, E
                MOV(E, E);
                break;
            case 0x5C: // MOV E, H
                MOV(E, H);
                break;
            case 0x5D: // MOV E, L
                MOV(E, L);
                break;
            case 0x5E: // MOV E, M
                MOV(E, M);
                break;
            case 0x5F: // MOV E, A
                MOV(E, A);
                break;
            case 0x60: // MOV H, B
                MOV(H, B);
                break;
            case 0x61: // MOV H, C
                MOV(H, C);
                break;
            case 0x62: // MOV H, D
                MOV(H, D);
                break;
            case 0x63: // MOV H, E
                MOV(H, E);
                break;
            case 0x64: // MOV H, H
                MOV(H, H);
                break;
            case 0x65: // MOV H, L
                MOV(H, L);
                break;
            case 0x66: // MOV H, M
                MOV(H, M);
                break;
            case 0x67: // MOV H, A
                MOV(H, A);
                break;
            case 0x68: // MOV L, B
                MOV(L, B);
                break;
            case 0x69: // MOV L, C
                MOV(L, C);
                break;
            case 0x6A: // MOV L, D
                MOV(L, D);
                break;
            case 0x6B: // MOV L, E
                MOV(L, E);
                break;
            case 0x6C: // MOV L, H
                MOV(L, H);
                break;
            case 0x6D: // MOV L, L
                MOV(L, L);
                break;
            case 0x6E: // MOV L, M
                MOV(L, M);
                break;
            case 0x6F: // MOV L, A
                MOV(L, A);
                break;
            case 0x70: // MOV M, B
                MOV(M, B);
                break;
            case 0x71: // MOV M, C
                MOV(M, C);
                break;
            case 0x72: // MOV M, D
                MOV(M, D);
                break;
            case 0x73: // MOV M, E
                MOV(M, E);
                break;
            case 0x74: // MOV M, H
                MOV(M, H);
                break;
            case 0x75: // MOV M, L
                MOV(M, L);
                break;
            case 0x76: // HLT
                return;
                break;
            case 0x77: // MOV M, A
                MOV(M, A);
                break;
            case 0x78: // MOV A, B
                MOV(A, B);
                break;
            case 0x79: // MOV A, C
                MOV(A, C);
                break;
            case 0x7A: // MOV A, D
                MOV(A, D);
                break;
            case 0x7B: // MOV A, E
                MOV(A, E);
                break;
            case 0x7C: // MOV A, H
                MOV(A, H);
                break;
            case 0x7D: // MOV A, L
                MOV(A, L);
                break;
            case 0x7E: // MOV A, M
                MOV(A, M);
                break;
            case 0x7F: // MOV A, A
                MOV(A, A);
                break;
            case 0x80: // ADD B
                ADD(B);
                break;
            case 0x81: // ADD C
                ADD(C);
                break;
            case 0x82: // ADD D
                ADD(D);
                break;
            case 0x83: // ADD E
                ADD(E);
                break;
            case 0x84: // ADD H
                ADD(H);
                break;
            case 0x85: // ADD L
                ADD(L);
                break;
            case 0x86: // ADD M
                ADD(M);
                break;
            case 0x87: // ADD A
                ADD(A);
                break;
            case 0x88: // ADC B
                ADC(B);
                break;
            case 0x89: // ADC C
                ADC(C);
                break;
            case 0x8A: // ADC D
                ADC(D);
                break;
            case 0x8B: // ADC E
                ADC(E);
                break;
            case 0x8C: // ADC H
                ADC(H);
                break;
            case 0x8D: // ADC L
                ADC(L);
                break;
            case 0x8E: // ADC M
                ADC(M);
                break;
            case 0x8F: // ADC A
                ADC(A);
                break;
            case 0x90: // SUB B
                SUB(B);
                break;
            case 0x91: // SUB C
                SUB(C);
                break;
            case 0x92: // SUB D
                SUB(D);
                break;
            case 0x93: // SUB E
                SUB(E);
                break;
            case 0x94: // SUB H
                SUB(H);
                break;
            case 0x95: // SUB L
                SUB(L);
                break;
            case 0x96: // SUB M
                SUB(M);
                break;
            case 0x97: // SUB A
                SUB(A);
                break;
            case 0x98: // SBB B
                SBB(B);
                break;
            case 0x99: // SBB C
                SBB(C);
                break;
            case 0x9A: // SBB D
                SBB(D);
                break;
            case 0x9B: // SBB E
                SBB(E);
                break;
            case 0x9C: // SBB H
                SBB(H);
                break;
            case 0x9D: // SBB L
                SBB(L);
                break;
            case 0x9E: // SBB M
                SBB(M);
                break;
            case 0x9F: // SBB A
                SBB(A);
                break;
            case 0xA0: // ANA B
                ANA(B);
                break;
            case 0xA1: // ANA C
                ANA(C);
                break;
            case 0xA2: // ANA D
                ANA(D);
                break;
            case 0xA3: // ANA E
                ANA(E);
                break;
            case 0xA4: // ANA H
                ANA(H);
                break;
            case 0xA5: // ANA L
                ANA(L);
                break;
            case 0xA6: // ANA M
                ANA(M);
                break;
            case 0xA7: // ANA A
                ANA(A);
                break;
            case 0xA8: // XRA B
                XRA(B);
                break;
            case 0xA9: // XRA C
                XRA(C);
                break;
            case 0xAA: // XRA D
                XRA(D);
                break;
            case 0xAB: // XRA E
                XRA(E);
                break;
            case 0xAC: // XRA H
                XRA(H);
                break;
            case 0xAD: // XRA L
                XRA(L);
                break;
            case 0xAE: // XRA M
                XRA(M);
                break;
            case 0xAF: // XRA A
                XRA(A);
                break;
            case 0xB0: // ORA B
                ORA(B);
                break;
            case 0xB1: // ORA C
                ORA(C);
                break;
            case 0xB2: // ORA D
                ORA(D);
                break;
            case 0xB3: // ORA E
                ORA(E);
                break;
            case 0xB4: // ORA H
                ORA(H);
                break;
            case 0xB5: // ORA L
                ORA(L);
                break;
            case 0xB6: // ORA M
                ORA(M);
                break;
            case 0xB7: // ORA A
                ORA(A);
                break;
            case 0xB8: // CMP B
                CMP(B);
                break;
            case 0xB9: // CMP C
                CMP(C);
                break;
            case 0xBA: // CMP D
                CMP(D);
                break;
            case 0xBB: // CMP E
                CMP(E);
                break;
            case 0xBC: // CMP H
                CMP(H);
                break;
            case 0xBD: // CMP L
                CMP(L);
                break;
            case 0xBE: // CMP M
                CMP(M);
                break;
            case 0xBF: // CMP A
                CMP(A);
                break;
            case 0xC0: // RNZ
                RNZ();
                break;
            case 0xC1: // POP B
                POP(B_C);
                break;
            case 0xC2: // JNZ adr
                JNZ((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xC3: // JMP adr
                JMP((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xC4: // CNZ adr
                CNZ((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xC5: // PUSH B
                PUSH(B_C);
                break;
            case 0xC6: // ADI D8
                ADI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xC7: // RST 0
                RST(0);
                break;
            case 0xC8: // RZ
                RZ();
                break;
            case 0xC9: // RET
                RET();
                break;
            case 0xCA: // JZ adr
                JZ((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xCC: // CZ adr
                CZ((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xCD: // CALL adr
                CALL((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xCE: // ACI D8
                ACI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xCF: // RST 1
                RST(1);
                break;
            case 0xD0: // RNC
                RNC();
                break;
            case 0xD1: // POP D
                POP(D_E);
                break;
            case 0xD2: // JNC adr
                JNC((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xD3: // OUT D8
                OUT(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xD4: // CNC adr
                CNC((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xD5: // PUSH D
                PUSH(D_E);
                break;
            case 0xD6: // SUI D8
                SUI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xD7: // RST 2
                RST(2);
                break;
            case 0xD8: // RC
                RC();
                break;
            case 0xDA: // JC adr
                JC((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xDB: // IN D8
                IN(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xDC: // CC adr
                CC((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xDE: // SBI D8
                SBI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xDF: // RST 3
                RST(3);
                break;
            case 0xE0: // RPO
                RPO();
                break;
            case 0xE1: // POP H
                POP(H_L);
                break;
            case 0xE2: // JPO adr
                JPO((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xE3: // XTHL
                XTHL();
                break;
            case 0xE4: // CPO adr
                CPO((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xE5: // PUSH H
                PUSH(H_L);
                break;
            case 0xE6: // ANI D8
                ANI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xE7: // RST 4
                RST(4);
                break;
            case 0xE8: // RPE
                RPE();
                break;
            case 0xE9: // PCHL
                PCHL();
                break;
            case 0xEA: // JPE adr
                JPE((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xEB: // XCHG
                XCHG();
                break;
            case 0xEC: // CPE adr
                CPE((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xEE: // XRI D8
                XRI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xEF: // RST 5
                RST(5);
                break;
            case 0xF0: // RP
                RP();
                break;
            case 0xF1: // POP PSW
                POP(PSW);
                break;
            case 0xF2: // JP adr
                JP((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xF3: // DI
                DI();
                break;
            case 0xF4: // CP adr
                CP((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xF5: // PUSH PSW
                PUSH(PSW);
                break;
            case 0xF6: // ORI D8
                ORI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xF7: // RST 6
                RST(6);
                break;
            case 0xF8: // RM
                RM();
                break;
            case 0xF9: // SPHL
                SPHL();
                break;
            case 0xFA: // JM adr
                JM((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xFB: // EI
                EI();
                break;
            case 0xFC: // CM adr
                CM((cpu.memory[cpu.pc + 1] << 8) | cpu.memory[cpu.pc]);
                cpu.pc += 2;
                break;
            case 0xFE: // CPI D8
                CPI(cpu.memory[cpu.pc]);
                cpu.pc++;
                break;
            case 0xFF: // RST 7
                RST(7);
                break;
            default:
                dump_cpu();
                assert(0); // Unimplemented opcode
                break;
        }

        dump_cpu();

        assert(cpu.pc < MEM_SIZE);
    }
}

void update_flags_aux(uint16_t result, bool auxiliary_carry)
{
    //Zero flag (Z)
    if(result == 0){
        cpu.status_register |= (1 << Z);
    }else{
        cpu.status_register &= ~(1 << Z);
    }

    //Sign flag (S)
    if(result & 0x80){
        cpu.status_register |= (1 << S);
    }else{
        cpu.status_register &= ~(1 << S);
    }

    //Parity flag (P)
    bool parity = calc_parity(result);
    if(!parity){
        cpu.status_register |= (1 << P);
    }else{
        cpu.status_register &= ~(1 << P);
    }

    // Auxiliary Carry flag (AC) (carry from bit 3 to bit 4)
    if(auxiliary_carry){
        cpu.status_register |= (1 << AC);
    }else{
        cpu.status_register &= ~(1 << AC);
    }
}

void update_flags_cy(uint16_t result)
{
    //Zero flag (Z)
    if(result == 0){
        cpu.status_register |= (1 << Z);
    }else{
        cpu.status_register &= ~(1 << Z);
    }

    //Sign flag (S)
    if(result & 0x80){
        cpu.status_register |= (1 << S);
    }else{
        cpu.status_register &= ~(1 << S);
    }

    //Parity flag (P)
    bool parity = calc_parity(result);
    if(!parity){
        cpu.status_register |= (1 << P);
    }else{
        cpu.status_register &= ~(1 << P);
    }

    // Carry flag (CY)
    if(result > 0xFF){
        cpu.status_register |= (1 << CY);
    }else{
        cpu.status_register &= ~(1 << CY);
    }
}

void update_flags_aux_cy(uint16_t result, bool auxiliary_carry)
{
    //Zero flag (Z)
    if(result == 0){
        cpu.status_register |= (1 << Z);
    }else{
        cpu.status_register &= ~(1 << Z);
    }

    //Sign flag (S)
    if(result & 0x80){
        cpu.status_register |= (1 << S);
    }else{
        cpu.status_register &= ~(1 << S);
    }

    //Parity flag (P)
    bool parity = calc_parity(result);
    if(!parity){
        cpu.status_register |= (1 << P);
    }else{
        cpu.status_register &= ~(1 << P);
    }

    // Auxiliary Carry flag (AC) (carry from bit 3 to bit 4)
    if(auxiliary_carry){
        cpu.status_register |= (1 << AC);
    }else{
        cpu.status_register &= ~(1 << AC);
    }

    // Carry flag (CY)
    if(result > 0xFF){
        cpu.status_register |= (1 << CY);
    }else{
        cpu.status_register &= ~(1 << CY);
    }
}

void dump_cpu()
{
    printf("Registers:\n");
    printf("B: %x\n", cpu.registers[B]);
    printf("C: %x\n", cpu.registers[C]);
    printf("D: %x\n", cpu.registers[D]);
    printf("E: %x\n", cpu.registers[E]);
    printf("H: %x\n", cpu.registers[H]);
    printf("L: %x\n", cpu.registers[L]);
    printf("A: %x\n", cpu.registers[A]);
    printf("PC: %x\n", cpu.pc);
    printf("SP: %x\n", cpu.sp);
    printf("Status Register: %x\n", cpu.status_register);
    printf("Interrupt Enable: %d\n", cpu.int_enable);
    printf("\n");
}