#include <stdio.h>
#include <stdint.h>
#include "opcodes.h"

// ----- DEFINES --------------------------------
#define BOOT_ROM_SIZE 1000000
#define BOOT_ROM_PATH "bootrom.bin"
#define RAM_SIZE 0xFFFF
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define INPUT_ADDR 0xF000 #The byte that changes according to input

#define F_Z (1 << 7) // Zero flag
#define F_N (1 << 6) // Negative Flag
#define F_H (1 << 5) // Half-carry
#define F_C (1 << 4) // Carry

enum Input
{
    INP_START,
    INP_SELECT,
    INP_B,
    INP_A,
    INP_RIGHT,
    INP_DOWN,
    INP_LEFT,
    INP_UP,
};

// ----- GLOBALS --------------------------------
uint8_t ram[RAM_SIZE];

uint8_t registerSpace[7];

uint8_t *A = (uint8_t *)(registerSpace + 0);
uint8_t *B = (uint8_t *)(registerSpace + 1);
uint8_t *C = (uint8_t *)(registerSpace + 2);
uint8_t *D = (uint8_t *)(registerSpace + 3);
uint8_t *E = (uint8_t *)(registerSpace + 4);
uint8_t *H = (uint8_t *)(registerSpace + 5);
uint8_t *L = (uint8_t *)(registerSpace + 6);

uint16_t *BC = (uint16_t *)(registerSpace + 1);
uint16_t *CD = (uint16_t *)(registerSpace + 2);
uint16_t *HL = (uint16_t *)(registerSpace + 5);

uint16_t SP;
uint16_t PC = 0x0000;
uint8_t F;

// ----- PROTOTYPES  --------------------------------

void loadBootRom(uint8_t *where_to_store)
{
    FILE *fp = fopen(BOOT_ROM_PATH, "r");
    if (fp != NULL)
    {
        size_t newLen = fread(where_to_store, sizeof(uint8_t), BOOT_ROM_SIZE, fp);
        if (ferror(fp) != 0)
        {
            fputs("Error reading file", stderr);
        }
        else
        {
            where_to_store[newLen++] = '\0'; /* Just to be safe. */
        }
        fclose(fp);
    }
}

void main()
{
    // Load the Boot ROM
    uint8_t bootRom[BOOT_ROM_SIZE + 1];
    loadBootRom(bootRom);

    while (1)
    {
        // Read instruction
        uint8_t instructionLength = 0;
        uint8_t* nextOpcode = (uint8_t*)(bootRom + PC);
        switch(*nextOpcode) {
            case LDI_B: {
                
                break;
            }
            case LDI_C: {
                
                break;
            }
            case LDI_D: {
                
                break;
            }
            case LDI_E: {
                
                break;
            }
            case LDI_H: {
                
                break;
            }
            case LDI_L: {
                
                break;
            }
            case LDI_HL: {
                
                break;
            }
            case LDI_A: {
                
                break;
            }
            case LDX_BC: {
                
                break;
            }
            case LDX_DE: {
                
                break;
            }
            case LDX_HL: {
                
                break;
            }
            case LDX_SP: {
                
                break;
            }
            case PUSH_B: {
                
                break;
            }
            case PUSH_C: {
                
                break;
            }
            case PUSH_D: {
                
                break;
            }
            case PUSH_E: {
                
                break;
            }
            case PUSH_H: {
                
                break;
            }
            case PUSH_L: {
                
                break;
            }
            case PUSH_HL_PTR: {
                
                break;
            }
            case PUSH_A: {
                
                break;
            }
            case PUSH_BC: {
                
                break;
            }
            case PUSH_DE: {
                
                break;
            }
            case PUSH_HL: {
                
                break;
            }
            case POP_B: {
                
                break;
            }
            case POP_C: {
                
                break;
            }
            case POP_D: {
                
                break;
            }
            case POP_E: {
                
                break;
            }
            case POP_H: {
                
                break;
            }
            case POP_L: {
                
                break;
            }
            case POP_HL_PTR: {
                
                break;
            }
            case POP_A: {
                
                break;
            }
            case POP_BC: {
                
                break;
            }
            case POP_DE: {
                
                break;
            }
            case POP_HL: {
                
                break;
            }
            case MOV_B_B: {
                
                break;
            }
            case MOV_B_C: {
                
                break;
            }
            case MOV_B_D: {
                
                break;
            }
            case MOV_B_E: {
                
                break;
            }
            case MOV_B_H: {
                
                break;
            }
            case MOV_B_L: {
                
                break;
            }
            case MOV_B_HL: {
                
                break;
            }
            case MOV_B_A: {
                
                break;
            }
            case MOV_C_B: {
                
                break;
            }
            case MOV_C_C: {
                
                break;
            }
            case MOV_C_D: {
                
                break;
            }
            case MOV_C_E: {
                
                break;
            }
            case MOV_C_H: {
                
                break;
            }
            case MOV_C_L: {
                
                break;
            }
            case MOV_C_HL: {
                
                break;
            }
            case MOV_C_A: {
                
                break;
            }
            case MOV_D_B: {
                
                break;
            }
            case MOV_D_C: {
                
                break;
            }
            case MOV_D_D: {
                
                break;
            }
            case MOV_D_E: {
                
                break;
            }
            case MOV_D_H: {
                
                break;
            }
            case MOV_D_L: {
                
                break;
            }
            case MOV_D_HL: {
                
                break;
            }
            case MOV_D_A: {
                
                break;
            }
            case MOV_E_B: {
                
                break;
            }
            case MOV_E_C: {
                
                break;
            }
            case MOV_E_D: {
                
                break;
            }
            case MOV_E_E: {
                
                break;
            }
            case MOV_E_H: {
                
                break;
            }
            case MOV_E_L: {
                
                break;
            }
            case MOV_E_HL: {
                
                break;
            }
            case MOV_E_A: {
                
                break;
            }
            case MOV_H_B: {
                
                break;
            }
            case MOV_H_C: {
                
                break;
            }
            case MOV_H_D: {
                
                break;
            }
            case MOV_H_E: {
                
                break;
            }
            case MOV_H_H: {
                
                break;
            }
            case MOV_H_L: {
                
                break;
            }
            case MOV_H_HL: {
                
                break;
            }
            case MOV_H_A: {
                
                break;
            }
            case MOV_L_B: {
                
                break;
            }
            case MOV_L_C: {
                
                break;
            }
            case MOV_L_D: {
                
                break;
            }
            case MOV_L_E: {
                
                break;
            }
            case MOV_L_H: {
                
                break;
            }
            case MOV_L_L: {
                
                break;
            }
            case MOV_L_HL: {
                
                break;
            }
            case MOV_L_A: {
                
                break;
            }
            case MOV_HL_B: {
                
                break;
            }
            case MOV_HL_C: {
                
                break;
            }
            case MOV_HL_D: {
                
                break;
            }
            case MOV_HL_E: {
                
                break;
            }
            case MOV_HL_H: {
                
                break;
            }
            case MOV_HL_L: {
                
                break;
            }
            case HCF: {
                
                break;
            }
            case MOV_HL_A: {
                
                break;
            }
            case MOV_A_B: {
                
                break;
            }
            case MOV_A_C: {
                
                break;
            }
            case MOV_A_D: {
                
                break;
            }
            case MOV_A_E: {
                
                break;
            }
            case MOV_A_H: {
                
                break;
            }
            case MOV_A_L: {
                
                break;
            }
            case MOV_A_HL: {
                
                break;
            }
            case MOV_A_A: {
                
                break;
            }
            case MOV_HL_BC: {
                
                break;
            }
            case MOV_HL_DE: {
                
                break;
            }
            case CLRFLAG: {
                
                break;
            }
            case SETFLAG_Z_1: {
                
                break;
            }
            case SETFLAG_Z_0: {
                
                break;
            }
            case SETFLAG_N_1: {
                
                break;
            }
            case SETFLAG_N_0: {
                
                break;
            }
            case SETFLAG_H_1: {
                
                break;
            }
            case SETFLAG_H_0: {
                
                break;
            }
            case SETFLAG_C_1: {
                
                break;
            }
            case SETFLAG_C_0: {
                
                break;
            }
            case ADD_B: {
                
                break;
            }
            case ADD_C: {
                
                break;
            }
            case ADD_D: {
                
                break;
            }
            case ADD_E: {
                
                break;
            }
            case ADD_H: {
                
                break;
            }
            case ADD_L: {
                
                break;
            }
            case ADD_HL: {
                
                break;
            }
            case ADD_A: {
                
                break;
            }
            case ADDI: {
                
                break;
            }
            case ADDX_BC: {
                
                break;
            }
            case ADDX_DE: {
                
                break;
            }
            case ADDX_HL: {
                
                break;
            }
            case SUB_B: {
                
                break;
            }
            case SUB_C: {
                
                break;
            }
            case SUB_D: {
                
                break;
            }
            case SUB_E: {
                
                break;
            }
            case SUB_H: {
                
                break;
            }
            case SUB_L: {
                
                break;
            }
            case SUB_HL: {
                
                break;
            }
            case SUB_A: {
                
                break;
            }
            case SUBI: {
                
                break;
            }
            case INC_B: {
                
                break;
            }
            case INC_C: {
                
                break;
            }
            case INC_D: {
                
                break;
            }
            case INC_E: {
                
                break;
            }
            case INC_H: {
                
                break;
            }
            case INC_L: {
                
                break;
            }
            case INC_HL: {
                
                break;
            }
            case INC_A: {
                
                break;
            }
            case INX_BC: {
                
                break;
            }
            case INX_DE: {
                
                break;
            }
            case INX_HL: {
                
                break;
            }
            case DEC_B: {
                
                break;
            }
            case DEC_C: {
                
                break;
            }
            case DEC_D: {
                
                break;
            }
            case DEC_E: {
                
                break;
            }
            case DEC_H: {
                
                break;
            }
            case DEC_L: {
                
                break;
            }
            case DEC_HL: {
                
                break;
            }
            case DEC_A: {
                
                break;
            }
            case AND_B: {
                
                break;
            }
            case AND_C: {
                
                break;
            }
            case AND_D: {
                
                break;
            }
            case AND_E: {
                
                break;
            }
            case AND_H: {
                
                break;
            }
            case AND_L: {
                
                break;
            }
            case AND_HL: {
                
                break;
            }
            case AND_A: {
                
                break;
            }
            case ANDI: {
                
                break;
            }
            case OR_B: {
                
                break;
            }
            case OR_C: {
                
                break;
            }
            case OR_D: {
                
                break;
            }
            case OR_E: {
                
                break;
            }
            case OR_H: {
                
                break;
            }
            case OR_L: {
                
                break;
            }
            case OR_HL: {
                
                break;
            }
            case OR_A: {
                
                break;
            }
            case ORI: {
                
                break;
            }
            case XOR_B: {
                
                break;
            }
            case XOR_C: {
                
                break;
            }
            case XOR_D: {
                
                break;
            }
            case XOR_E: {
                
                break;
            }
            case XOR_H: {
                
                break;
            }
            case XOR_L: {
                
                break;
            }
            case XOR_HL: {
                
                break;
            }
            case XOR_A: {
                
                break;
            }
            case XORI: {
                
                break;
            }
            case CMP_B: {
                
                break;
            }
            case CMP_C: {
                
                break;
            }
            case CMP_D: {
                
                break;
            }
            case CMP_E: {
                
                break;
            }
            case CMP_H: {
                
                break;
            }
            case CMP_L: {
                
                break;
            }
            case CMP_HL: {
                
                break;
            }
            case CMP_A: {
                
                break;
            }
            case CMPI_A: {
                
                break;
            }
            case CMPS_B: {
                
                break;
            }
            case CMPS_C: {
                
                break;
            }
            case CMPS_D: {
                
                break;
            }
            case CMPS_E: {
                
                break;
            }
            case CMPS_H: {
                
                break;
            }
            case CMPS_L: {
                
                break;
            }
            case CMPS_HL: {
                
                break;
            }
            case CMPS_A: {
                
                break;
            }
            case SIN: {
                
                break;
            }
            case SOUT: {
                
                break;
            }
            case CLRSCR: {
                
                break;
            }
            case DRAW: {
                
                break;
            }
            case JMP: {
                
                break;
            }
            case JMPZ: {
                
                break;
            }
            case JMPNZ: {
                
                break;
            }
            case JMPN: {
                
                break;
            }
            case JMPNN: {
                
                break;
            }
            case JMPH: {
                
                break;
            }
            case JMPNH: {
                
                break;
            }
            case JMPC: {
                
                break;
            }
            case JMPNC: {
                
                break;
            }
            case JMP_CC: {
                
                break;
            }
            case JMPZ_CC: {
                
                break;
            }
            case JMPNZ_CC: {
                
                break;
            }
            case JMPN_CC: {
                
                break;
            }
            case JMPNN_CC: {
                
                break;
            }
            case JMPH_CC: {
                
                break;
            }
            case JMPNH_CC: {
                
                break;
            }
            case JMPC_CC: {
                
                break;
            }
            case JMPNC_CC: {
                
                break;
            }
            case CALL: {
                
                break;
            }
            case RET: {
                
                break;
            }
            case NOP: {
                
                break;
            }
            default:
                break;
        }

        // Update program counter
        PC++;
    }
}
