#include "i8080.h"
#include "instructions.h"

#include <assert.h>

void CMC()
{
    cpu.status_register ^= (1 << CY);
}

void STC()
{
    cpu.status_register |= (1 << CY);
}

void INR(uint8_t dest)
{
    uint16_t result = 0;
    bool auxiliary_carry = false;

    if(dest == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        auxiliary_carry = (cpu.memory[mem_address] & 0x0F) == 0x0F;
        result = cpu.memory[mem_address] + 1;
        cpu.memory[mem_address] = result & 0xFF;
    }else{
        uint8_t* reg = 0;
        if(dest < NUM_REGISTERS){
            reg = &cpu.registers[dest];
        }

        auxiliary_carry = ((*reg) & 0x0F) == 0x0F;
        result = (*reg) + 1;
        (*reg) = result & 0xFF;
    }

    update_flags_aux(result, auxiliary_carry);
}

void DCR(uint8_t dest)
{
    uint16_t result = 0;
    bool auxiliary_carry = false;

    if(dest == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        auxiliary_carry = (cpu.memory[mem_address] & 0x0F) == 0;
        result = cpu.memory[mem_address] - 1;
        cpu.memory[mem_address] = result & 0xFF;
    }else{
        uint8_t* reg = 0;
        if(dest < NUM_REGISTERS){
            reg = &cpu.registers[dest];
        }

        auxiliary_carry = ((*reg) & 0x0F) == 0;
        result = (*reg) - 1;
        (*reg) = result & 0xFF;
    }

    update_flags_aux(result, auxiliary_carry);
}

void CMA()
{
    cpu.registers[A] = ~cpu.registers[A];
}

void DAA()
{
    uint16_t acc = cpu.registers[A];
    bool carry = cpu.status_register & (1 << CY);
    bool auxiliary_carry = cpu.status_register & (1 << AC);

    if((acc & 0x0F) > 9 || auxiliary_carry){
        acc += 0x06;
        if((acc & 0xF0) < 0x10){
            auxiliary_carry = true;
        }
    }

    if((acc >> 4) > 9 || carry){
        acc += 0x60;
    }

    cpu.registers[A] = acc & 0xFF;

    update_flags_aux_cy(acc, auxiliary_carry);
}

void MOV(uint8_t dest, uint8_t src)
{
    if(dest == M){
        if(src < NUM_REGISTERS){
            uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
            cpu.memory[mem_address] = cpu.registers[src];
        }
    }else if(dest < NUM_REGISTERS){
        if(src < NUM_REGISTERS){
            cpu.registers[dest] = cpu.registers[src];
        }else if(src == M){
            uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
            cpu.registers[dest] = cpu.memory[mem_address];
        }
    }
}

void STAX(uint8_t dest)
{
    uint16_t mem_address = 0;

    if(dest == B_C){
        mem_address = (cpu.registers[B] << 8) | cpu.registers[C];
    }else if(dest == D_E){
        mem_address = (cpu.registers[D] << 8) | cpu.registers[E];
    }

    cpu.memory[mem_address] = cpu.registers[A];
}

void LDAX(uint8_t src)
{
    uint16_t mem_address = 0;

    if(src == B_C){
        mem_address = (cpu.registers[B] << 8) | cpu.registers[C];
    }else if(src == D_E){
        mem_address = (cpu.registers[D] << 8) | cpu.registers[E];
    }

    cpu.registers[A] = cpu.memory[mem_address];
}

void ADD(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] + cpu.memory[mem_address];
        bool auxiliary_carry = ((cpu.registers[A] & 0x0F) + (cpu.memory[mem_address] & 0x0F)) > 0x0F;
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] + cpu.registers[src];
        bool auxiliary_carry = ((cpu.registers[A] & 0x0F) + (cpu.registers[src] & 0x0F)) > 0x0F;
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }
}

void ADC(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] + cpu.memory[mem_address] + ((cpu.status_register & (1 << CY)) > 0);
        bool auxiliary_carry = ((cpu.registers[A] & 0x0F) + (cpu.memory[mem_address] & 0x0F) + ((cpu.status_register & (1 << CY)) > 0)) > 0x0F;
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] + cpu.registers[src] + ((cpu.status_register & (1 << CY)) > 0);
        bool auxiliary_carry = ((cpu.registers[A] & 0x0F) + (cpu.registers[src] & 0x0F) + ((cpu.status_register & (1 << CY)) > 0)) > 0x0F;
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }
}

void SUB(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] - cpu.memory[mem_address];
        bool auxiliary_carry = (cpu.registers[A] & 0x0F) < (cpu.memory[mem_address] & 0x0F);
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] - cpu.registers[src];
        bool auxiliary_carry = (cpu.registers[A] & 0x0F) < (cpu.registers[src] & 0x0F);
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }
}

void SBB(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] - cpu.memory[mem_address] - ((cpu.status_register & (1 << CY)) > 0);
        bool auxiliary_carry = (cpu.registers[A] & 0x0F) < (cpu.memory[mem_address] & 0x0F) + ((cpu.status_register & (1 << CY)) > 0);
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] - cpu.registers[src] - ((cpu.status_register & (1 << CY)) > 0);
        bool auxiliary_carry = (cpu.registers[A] & 0x0F) < (cpu.registers[src] & 0x0F) + ((cpu.status_register & (1 << CY)) > 0);
        cpu.registers[A] = result & 0xFF;
        update_flags_aux_cy(result, auxiliary_carry);
    }
}

void ANA(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] & cpu.memory[mem_address];
        cpu.registers[A] = result & 0xFF;
        update_flags_cy(result);
        cpu.status_register &= ~(1 << CY); 
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] & cpu.registers[src];
        cpu.registers[A] = result & 0xFF;
        update_flags_cy(result);
        cpu.status_register &= ~(1 << CY); 
    }
}

void XRA(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] ^ cpu.memory[mem_address];
        cpu.registers[A] = result & 0xFF;
        update_flags_cy(result);
        cpu.status_register &= ~(1 << CY);
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] ^ cpu.registers[src];
        cpu.registers[A] = result & 0xFF;
        update_flags_cy(result);
        cpu.status_register &= ~(1 << CY);
    }
}

void ORA(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] | cpu.memory[mem_address];
        cpu.registers[A] = result & 0xFF;
        update_flags_cy(result);
        cpu.status_register &= ~(1 << CY);
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] | cpu.registers[src];
        cpu.registers[A] = result & 0xFF;
        update_flags_cy(result);
        cpu.status_register &= ~(1 << CY);
    }
}

void CMP(uint8_t src)
{
    if(src == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        uint16_t result = cpu.registers[A] - cpu.memory[mem_address];
        update_flags_aux_cy(result, (cpu.registers[A] & 0x0F) < (cpu.memory[mem_address] & 0x0F));
    }else if(src < NUM_REGISTERS){
        uint16_t result = cpu.registers[A] - cpu.registers[src];
        update_flags_aux_cy(result, (cpu.registers[A] & 0x0F) < (cpu.registers[src] & 0x0F));
    }
}

void RLC()
{
    uint8_t carry = (cpu.registers[A] & 0x80) >> 7;
    cpu.registers[A] = (cpu.registers[A] << 1) | carry;
    cpu.status_register = (cpu.status_register & ~(1 << CY)) | (carry << CY);
}

void RRC()
{
    uint8_t carry = cpu.registers[A] & 0x01;
    cpu.registers[A] = (cpu.registers[A] >> 1) | (carry << 7);
    cpu.status_register = (cpu.status_register & ~(1 << CY)) | (carry << CY);
}

void RAL()
{
    uint8_t carry = (cpu.status_register & (1 << CY)) >> CY;
    cpu.status_register = (cpu.status_register & ~(1 << CY)) | ((cpu.registers[A] & 0x80) >> 7);
    cpu.registers[A] = (cpu.registers[A] << 1) | carry;
}

void RAR()
{
    uint8_t carry = (cpu.status_register & (1 << CY)) << 7;
    cpu.status_register = (cpu.status_register & ~(1 << CY)) | (cpu.registers[A] & 0x01);
    cpu.registers[A] = (cpu.registers[A] >> 1) | carry;
}

void PUSH(uint8_t src)
{
    switch(src){
        case B_C:
            cpu.memory[--cpu.sp] = cpu.registers[B];
            cpu.memory[--cpu.sp] = cpu.registers[C];
            break;
        case D_E:
            cpu.memory[--cpu.sp] = cpu.registers[D];
            cpu.memory[--cpu.sp] = cpu.registers[E];
            break;
        case H_L:
            cpu.memory[--cpu.sp] = cpu.registers[H];
            cpu.memory[--cpu.sp] = cpu.registers[L];
            break;
        case PSW:
            cpu.memory[--cpu.sp] = cpu.registers[A];
            cpu.memory[--cpu.sp] = cpu.status_register;
            break;
        default:
            assert(0); //invalid src
    }
}

void POP(uint8_t dest)
{
    switch(dest){
        case B_C:
            cpu.registers[C] = cpu.memory[cpu.sp++];
            cpu.registers[B] = cpu.memory[cpu.sp++];
            break;
        case D_E:
            cpu.registers[E] = cpu.memory[cpu.sp++];
            cpu.registers[D] = cpu.memory[cpu.sp++];
            break;
        case H_L:
            cpu.registers[L] = cpu.memory[cpu.sp++];
            cpu.registers[H] = cpu.memory[cpu.sp++];
            break;
        case PSW:
            cpu.status_register = cpu.memory[cpu.sp++];
            cpu.registers[A] = cpu.memory[cpu.sp++];
            break;
        default:
            assert(0); //invalid dest
    }
}

void DAD(uint8_t src)
{
    uint32_t result = 0;
    uint16_t hl = (cpu.registers[H] << 8) | cpu.registers[L];

    switch(src){
        case B_C:
            result = hl + ((cpu.registers[B] << 8) | cpu.registers[C]);
            break;
        case D_E:
            result = hl + ((cpu.registers[D] << 8) | cpu.registers[E]);
            break;
        case H_L:
            result = hl + ((cpu.registers[H] << 8) | cpu.registers[L]);
            break;
        case SP:
            result = hl + cpu.sp;
            break;
    }

    cpu.registers[H] = (result >> 8) & 0xFF;
    cpu.registers[L] = result & 0xFF;

    if(result > 0xFFFF){
        cpu.status_register |= (1 << CY);
    }else{
        cpu.status_register &= ~(1 << CY);
    }
}

void INX(uint8_t dest)
{
    uint16_t result = 0;

    switch(dest){
        case B_C:
            result = ((cpu.registers[B] << 8) | cpu.registers[C]) + 1;
            cpu.registers[B] = (result >> 8) & 0xFF;
            cpu.registers[C] = result & 0xFF;
            break;
        case D_E:
            result = ((cpu.registers[D] << 8) | cpu.registers[E]) + 1;
            cpu.registers[D] = (result >> 8) & 0xFF;
            cpu.registers[E] = result & 0xFF;
            break;
        case H_L:
            result = ((cpu.registers[H] << 8) | cpu.registers[L]) + 1;
            cpu.registers[H] = (result >> 8) & 0xFF;
            cpu.registers[L] = result & 0xFF;
            break;
        case SP:
            cpu.sp++;
            break;
        default:
            assert(0); //invalid dest
    }
}

void DCX(uint8_t dest)
{
    uint16_t result = 0;

    switch(dest){
        case B_C:
            result = ((cpu.registers[B] << 8) | cpu.registers[C]) - 1;
            cpu.registers[B] = (result >> 8) & 0xFF;
            cpu.registers[C] = result & 0xFF;
            break;
        case D_E:
            result = ((cpu.registers[D] << 8) | cpu.registers[E]) - 1;
            cpu.registers[D] = (result >> 8) & 0xFF;
            cpu.registers[E] = result & 0xFF;
            break;
        case H_L:
            result = ((cpu.registers[H] << 8) | cpu.registers[L]) - 1;
            cpu.registers[H] = (result >> 8) & 0xFF;
            cpu.registers[L] = result & 0xFF;
            break;
        case SP:
            cpu.sp--;
            break;
        default:
            assert(0); //invalid dest
    }
}

void XCHG()
{
    uint8_t temp = cpu.registers[H];
    cpu.registers[H] = cpu.registers[D];
    cpu.registers[D] = temp;

    temp = cpu.registers[L];
    cpu.registers[L] = cpu.registers[E];
    cpu.registers[E] = temp;
}

void XTHL()
{
    uint8_t temp = cpu.memory[cpu.sp];
    cpu.memory[cpu.sp] = cpu.registers[L];
    cpu.registers[L] = temp;

    temp = cpu.memory[cpu.sp + 1];
    cpu.memory[cpu.sp + 1] = cpu.registers[H];
    cpu.registers[H] = temp;
}

void SPHL()
{
    cpu.sp = (cpu.registers[H] << 8) | cpu.registers[L];
}

void LXI(uint8_t dest, uint16_t data)
{
    switch(dest){
        case B_C:
            cpu.registers[C] = data & 0xFF;
            cpu.registers[B] = (data >> 8) & 0xFF;
            break;
        case D_E:
            cpu.registers[E] = data & 0xFF;
            cpu.registers[D] = (data >> 8) & 0xFF;
            break;
        case H_L:
            cpu.registers[L] = data & 0xFF;
            cpu.registers[H] = (data >> 8) & 0xFF;
            break;
        case SP:
            cpu.sp = data;
            break;
        default:
            assert(0); //invalid dest
    }
}

void MVI(uint8_t dest, uint8_t data)
{
    if(dest == M){
        uint16_t mem_address = (cpu.registers[H] << 8) | cpu.registers[L];
        cpu.memory[mem_address] = data;
    }else if(dest < NUM_REGISTERS){
        cpu.registers[dest] = data;
    }
}

void ADI(uint8_t data)
{
    uint16_t result = cpu.registers[A] + data;
    bool auxiliary_carry = ((cpu.registers[A] & 0x0F) + (data & 0x0F)) > 0x0F;
    cpu.registers[A] = result & 0xFF;
    update_flags_aux_cy(result, auxiliary_carry);
}

void ACI(uint8_t data)
{
    uint16_t result = cpu.registers[A] + data + ((cpu.status_register & (1 << CY)) > 0);
    bool auxiliary_carry = ((cpu.registers[A] & 0x0F) + (data & 0x0F) + ((cpu.status_register & (1 << CY)) > 0)) > 0x0F;
    cpu.registers[A] = result & 0xFF;
    update_flags_aux_cy(result, auxiliary_carry);
}

void SUI(uint8_t data)
{
    uint16_t result = cpu.registers[A] - data;
    bool auxiliary_carry = (cpu.registers[A] & 0x0F) < (data & 0x0F);
    cpu.registers[A] = result & 0xFF;
    update_flags_aux_cy(result, auxiliary_carry);
}

void SBI(uint8_t data)
{
    uint16_t result = cpu.registers[A] - data - ((cpu.status_register & (1 << CY)) > 0);
    bool auxiliary_carry = (cpu.registers[A] & 0x0F) < (data & 0x0F) + ((cpu.status_register & (1 << CY)) > 0);
    cpu.registers[A] = result & 0xFF;
    update_flags_aux_cy(result, auxiliary_carry);
}

void ANI(uint8_t data)
{
    uint16_t result = cpu.registers[A] & data;
    cpu.registers[A] = result;
    update_flags_cy(result);
    cpu.status_register &= ~(1 << CY);
}

void XRI(uint8_t data)
{
    uint16_t result = cpu.registers[A] ^ data;
    cpu.registers[A] = result;
    update_flags_cy(result);
    cpu.status_register &= ~(1 << CY);
}

void ORI(uint8_t data)
{
    uint16_t result = cpu.registers[A] | data;
    cpu.registers[A] = result;
    update_flags_cy(result);
    cpu.status_register &= ~(1 << CY);
}

void CPI(uint8_t data)
{
    uint16_t result = cpu.registers[A] - data;
    update_flags_aux_cy(result, (cpu.registers[A] & 0x0F) < (data & 0x0F));
}

void STA(uint16_t address)
{
    cpu.memory[address] = cpu.registers[A];
}

void LDA(uint16_t address)
{
    cpu.registers[A] = cpu.memory[address];
}

void SHLD(uint16_t address)
{
    cpu.memory[address] = cpu.registers[L];
    cpu.memory[address + 1] = cpu.registers[H];
}

void LHLD(uint16_t address)
{
    cpu.registers[L] = cpu.memory[address];
    cpu.registers[H] = cpu.memory[address + 1];
}

void PCHL()
{
    cpu.pc = (cpu.registers[H] << 8) | cpu.registers[L];
}

void JMP(uint16_t address)
{
    cpu.pc = address;
}

void JC(uint16_t address)
{
    if(cpu.status_register & (1 << CY)){
        cpu.pc = address;
    }
}

void JNC(uint16_t address)
{
    if(!(cpu.status_register & (1 << CY))){
        cpu.pc = address;
    }
}

void JZ(uint16_t address)
{
    if(cpu.status_register & (1 << Z)){
        cpu.pc = address;
    }
}

void JNZ(uint16_t address)
{
    if(!(cpu.status_register & (1 << Z))){
        cpu.pc = address;
    }
}

void JM(uint16_t address)
{
    if(cpu.status_register & (1 << S)){
        cpu.pc = address;
    }
}

void JP(uint16_t address)
{
    if(!(cpu.status_register & (1 << S))){
        cpu.pc = address;
    }
}

void JPE(uint16_t address)
{
    if(cpu.status_register & (1 << P)){
        cpu.pc = address;
    }
}

void JPO(uint16_t address)
{
    if(!(cpu.status_register & (1 << P))){
        cpu.pc = address;
    }
}

void CALL(uint16_t address)
{
    cpu.memory[--cpu.sp] = (cpu.pc >> 8) & 0xFF;
    cpu.memory[--cpu.sp] = cpu.pc & 0xFF;
    cpu.pc = address;
}

void CC(uint16_t address)
{
    if(cpu.status_register & (1 << CY)){
        CALL(address);
    }
}

void CNC(uint16_t address)
{
    if(!(cpu.status_register & (1 << CY))){
        CALL(address);
    }
}

void CZ(uint16_t address)
{
    if(cpu.status_register & (1 << Z)){
        CALL(address);
    }
}

void CNZ(uint16_t address)
{
    if(!(cpu.status_register & (1 << Z))){
        CALL(address);
    }
}

void CM(uint16_t address)
{
    if(cpu.status_register & (1 << S)){
        CALL(address);
    }
}

void CP(uint16_t address)
{
    if(!(cpu.status_register & (1 << S))){
        CALL(address);
    }
}

void CPE(uint16_t address)
{
    if(cpu.status_register & (1 << P)){
        CALL(address);
    }
}

void CPO(uint16_t address)
{
    if(!(cpu.status_register & (1 << P))){
        CALL(address);
    }
}

void RET()
{
    cpu.pc = cpu.memory[cpu.sp++];
    cpu.pc |= cpu.memory[cpu.sp++] << 8;
}

void RC()
{
    if(cpu.status_register & (1 << CY)){
        RET();
    }
}

void RNC()
{
    if(!(cpu.status_register & (1 << CY))){
        RET();
    }
}

void RZ()
{
    if(cpu.status_register & (1 << Z)){
        RET();
    }
}

void RNZ()
{
    if(!(cpu.status_register & (1 << Z))){
        RET();
    }
}

void RM()
{
    if(cpu.status_register & (1 << S)){
        RET();
    }
}

void RP()
{
    if(!(cpu.status_register & (1 << S))){
        RET();
    }
}

void RPE()
{
    if(cpu.status_register & (1 << P)){
        RET();
    }
}

void RPO()
{
    if(!(cpu.status_register & (1 << P))){
        RET();
    }
}

void RST(uint8_t address)
{
    cpu.memory[--cpu.sp] = (cpu.pc >> 8) & 0xFF;
    cpu.memory[--cpu.sp] = cpu.pc & 0xFF;
    cpu.pc = address << 3;
}

void IN(uint8_t port)
{
    cpu.registers[A] = cpu.in_ports[port];
}

void OUT(uint8_t port)
{
    cpu.out_ports[port] = cpu.registers[A];
}

void EI()
{
    cpu.int_enable = true;
}

void DI()
{
    cpu.int_enable = false;
}