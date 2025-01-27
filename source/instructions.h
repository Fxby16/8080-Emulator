#ifndef INSTRUCTIONS_H
#define INSTRUCTIONS_H

#include <stdint.h>

extern void CMC();
extern void STC();
extern void INR(uint8_t dest);
extern void DCR(uint8_t dest);
extern void CMA();
extern void DAA();
extern void MOV(uint8_t dest, uint8_t src);
extern void STAX(uint8_t dest); //expecting B or D
extern void LDAX(uint8_t src); //expecting B or D
extern void ADD(uint8_t src);
extern void ADC(uint8_t src);
extern void SUB(uint8_t src);
extern void SBB(uint8_t src);
extern void ANA(uint8_t src);
extern void XRA(uint8_t src);
extern void ORA(uint8_t src);
extern void CMP(uint8_t src);
extern void RLC();
extern void RRC();
extern void RAL();
extern void RAR();
extern void PUSH(uint8_t src);
extern void POP(uint8_t dest);
extern void DAD(uint8_t src);
extern void INX(uint8_t dest);
extern void DCX(uint8_t dest);
extern void XCHG();
extern void XTHL();
extern void SPHL();
extern void LXI(uint8_t dest, uint16_t data);
extern void MVI(uint8_t dest, uint8_t data);
extern void ADI(uint8_t data);
extern void ACI(uint8_t data);
extern void SUI(uint8_t data);
extern void SBI(uint8_t data);
extern void ANI(uint8_t data);
extern void XRI(uint8_t data);
extern void ORI(uint8_t data);
extern void CPI(uint8_t data);
extern void STA(uint16_t address);
extern void LDA(uint16_t address);
extern void SHLD(uint16_t address);
extern void LHLD(uint16_t address);
extern void PCHL();
extern void JMP(uint16_t address);
extern void JC(uint16_t address);
extern void JNC(uint16_t address);
extern void JZ(uint16_t address);
extern void JNZ(uint16_t address);
extern void JM(uint16_t address);
extern void JP(uint16_t address);
extern void JPE(uint16_t address);
extern void JPO(uint16_t address);
extern void CALL(uint16_t address);
extern void CC(uint16_t address);
extern void CNC(uint16_t address);
extern void CZ(uint16_t address);
extern void CNZ(uint16_t address);
extern void CM(uint16_t address);
extern void CP(uint16_t address);
extern void CPE(uint16_t address);
extern void CPO(uint16_t address);
extern void RET();
extern void RC();
extern void RNC();
extern void RZ();
extern void RNZ();
extern void RM();
extern void RP();
extern void RPE();
extern void RPO();
extern void RST(uint8_t address);
extern void IN(uint8_t port);
extern void OUT(uint8_t port);
extern void EI();
extern void DI();

#endif