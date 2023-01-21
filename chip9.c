#include <stdio.h>
#include <stdint.h>

// ----- DEFINES --------------------------------
#define BOOT_ROM_SIZE 1000000
#define BOOT_ROM_PATH "bootrom.bin"
#define RAM_SIZE 0xFFFF
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define INPUT_ADDR 0xF000 #The byte that changes according to input

// OPCODES
#define LDX_BC 0x21
#define LDX_DE 0x31
#define LDX_HL 0x41
#define LDX_SP 0x22

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
            case LDX_SP:
                // 2 more bytes expected
                instructionLength = 2;
                break;
            default:
                break;
        }

        // Update program counter
        PC++;
    }
}
