#define LDI_B 0x20
#define LDI_C 0x30
#define LDI_D 0x40
#define LDI_E 0x50
#define LDI_H 0x60
#define LDI_L 0x70
#define LDI_HL 0x80
#define LDI_A 0x90
#define LDX_BC 0x21
#define LDX_DE 0x31
#define LDX_HL 0x41
#define LDX_SP 0x22
#define PUSH_B 0x81
#define PUSH_C 0x91
#define PUSH_D 0xA1
#define PUSH_E 0xB1
#define PUSH_H 0xC1
#define PUSH_L 0xD1
#define PUSH_HL_PTR 0xC0
#define PUSH_A 0xD0
#define PUSH_BC 0x51
#define PUSH_DE 0x61
#define PUSH_HL 0x71
#define POP_B 0x82
#define POP_C 0x92
#define POP_D 0xA2
#define POP_E 0xB2
#define POP_H 0xC2
#define POP_L 0xD2
#define POP_HL_PTR 0xC3
#define POP_A 0xD3
#define POP_BC 0x52
#define POP_DE 0x62
#define POP_HL 0x72
#define MOV_B_B 0x09
#define MOV_B_C 0x19
#define MOV_B_D 0x29
#define MOV_B_E 0x39
#define MOV_B_H 0x49
#define MOV_B_L 0x59
#define MOV_B_HL 0x69
#define MOV_B_A 0x79
#define MOV_C_B 0x89
#define MOV_C_C 0x99
#define MOV_C_D 0xA9
#define MOV_C_E 0xB9
#define MOV_C_H 0xC9
#define MOV_C_L 0xD9
#define MOV_C_HL 0xE9
#define MOV_C_A 0xF9
#define MOV_D_B 0x0A
#define MOV_D_C 0x1A
#define MOV_D_D 0x2A
#define MOV_D_E 0x3A
#define MOV_D_H 0x4A
#define MOV_D_L 0x5A
#define MOV_D_HL 0x6A
#define MOV_D_A 0x7A
#define MOV_E_B 0x8A
#define MOV_E_C 0x9A
#define MOV_E_D 0xAA
#define MOV_E_E 0xBA
#define MOV_E_H 0xCA
#define MOV_E_L 0xDA
#define MOV_E_HL 0xEA
#define MOV_E_A 0xFA
#define MOV_H_B 0x0B
#define MOV_H_C 0x1B
#define MOV_H_D 0x2B
#define MOV_H_E 0x3B
#define MOV_H_H 0x4B
#define MOV_H_L 0x5B
#define MOV_H_HL 0x6B
#define MOV_H_A 0x7B
#define MOV_L_B 0x8B
#define MOV_L_C 0x9B
#define MOV_L_D 0xAB
#define MOV_L_E 0xBB
#define MOV_L_H 0xCB
#define MOV_L_L 0xDB
#define MOV_L_HL 0xEB
#define MOV_L_A 0xFB
#define MOV_HL_B 0x0C
#define MOV_HL_C 0x1C
#define MOV_HL_D 0x2C
#define MOV_HL_E 0x3C
#define MOV_HL_H 0x4C
#define MOV_HL_L 0x5C
#define HCF 0x6C
#define MOV_HL_A 0x7C
#define MOV_A_B 0x8C
#define MOV_A_C 0x9C
#define MOV_A_D 0xAC
#define MOV_A_E 0xBC
#define MOV_A_H 0xCC
#define MOV_A_L 0xDC
#define MOV_A_HL 0xEC
#define MOV_A_A 0xFC
#define MOV_HL_BC 0xED
#define MOV_HL_DE 0xFD
#define CLRFLAG 0x08
#define SETFLAG_Z_1 0x18
#define SETFLAG_Z_0 0x28
#define SETFLAG_N_1 0x38
#define SETFLAG_N_0 0x48
#define SETFLAG_H_1 0x58
#define SETFLAG_H_0 0x68
#define SETFLAG_C_1 0x78
#define SETFLAG_C_0 0x88
#define ADD_B 0x04
#define ADD_C 0x14
#define ADD_D 0x24
#define ADD_E 0x34
#define ADD_H 0x44
#define ADD_L 0x54
#define ADD_HL 0x64
#define ADD_A 0x74
#define ADDI 0xA7
#define ADDX_BC 0x83
#define ADDX_DE 0x93
#define ADDX_HL 0xA3
#define SUB_B 0x84
#define SUB_C 0x94
#define SUB_D 0xA4
#define SUB_E 0xB4
#define SUB_H 0xC4
#define SUB_L 0xD4
#define SUB_HL 0xE4
#define SUB_A 0xF4
#define SUBI 0xB7
#define INC_B 0x03
#define INC_C 0x13
#define INC_D 0x23
#define INC_E 0x33
#define INC_H 0x43
#define INC_L 0x53
#define INC_HL 0x63
#define INC_A 0x73
#define INX_BC 0xA8
#define INX_DE 0xB8
#define INX_HL 0xC8
#define DEC_B 0x07
#define DEC_C 0x17
#define DEC_D 0x27
#define DEC_E 0x37
#define DEC_H 0x47
#define DEC_L 0x57
#define DEC_HL 0x67
#define DEC_A 0x77
#define AND_B 0x05
#define AND_C 0x15
#define AND_D 0x25
#define AND_E 0x35
#define AND_H 0x45
#define AND_L 0x55
#define AND_HL 0x65
#define AND_A 0x75
#define ANDI 0xC7
#define OR_B 0x85
#define OR_C 0x95
#define OR_D 0xA5
#define OR_E 0xB5
#define OR_H 0xC5
#define OR_L 0xD5
#define OR_HL 0xE5
#define OR_A 0xF5
#define ORI 0xD7
#define XOR_B 0x06
#define XOR_C 0x16
#define XOR_D 0x26
#define XOR_E 0x36
#define XOR_H 0x46
#define XOR_L 0x56
#define XOR_HL 0x66
#define XOR_A 0x76
#define XORI 0xE7
#define CMP_B 0x86
#define CMP_C 0x96
#define CMP_D 0xA6
#define CMP_E 0xB6
#define CMP_H 0xC6
#define CMP_L 0xD6
#define CMP_HL 0xE6
#define CMP_A 0xF6
#define CMPI_A 0xF7
#define CMPS_B 0x0D
#define CMPS_C 0x1D
#define CMPS_D 0x2D
#define CMPS_E 0x3D
#define CMPS_H 0x4D
#define CMPS_L 0x5D
#define CMPS_HL 0x6D
#define CMPS_A 0x7D
#define SIN 0xE0
#define SOUT 0xE1
#define CLRSCR 0xF0
#define DRAW 0xF1
#define JMP 0x0F
#define JMPZ 0x1F
#define JMPNZ 0x2F
#define JMPN 0x3F
#define JMPNN 0x4F
#define JMPH 0x5F
#define JMPNH 0x6F
#define JMPC 0x7F
#define JMPNC 0x8F
#define JMP_CC 0x9F
#define JMPZ_CC 0xAF
#define JMPNZ_CC 0xBF
#define JMPN_CC 0xCF
#define JMPNN_CC 0xDF
#define JMPH_CC 0xEF
#define JMPNH_CC 0xFF
#define JMPC_CC 0xEE
#define JMPNC_CC 0xFE
#define CALL 0x1E
#define RET 0x0E
#define NOP 0x00
