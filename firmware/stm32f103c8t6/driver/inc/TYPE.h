#ifndef __TYPE_H
#define __TYPE_H
#include <stdint.h>

typedef union{
    uint32_t REG;
    struct {
        uint32_t BIT0:   1;
        uint32_t BIT1:   1;
        uint32_t BIT2:   1;
        uint32_t BIT3:   1;
        uint32_t BIT4:   1;
        uint32_t BIT5:   1;
        uint32_t BIT6:   1;
        uint32_t BIT7:   1;
        uint32_t BIT8:   1;
        uint32_t BIT9:   1;
        uint32_t BIT10:  1;
        uint32_t BIT11:  1;
        uint32_t BIT12:  1;
        uint32_t BIT13:  1;
        uint32_t BIT14:  1;
        uint32_t BIT15:  1;
        uint32_t BIT16:  1;
        uint32_t BIT17:  1;
        uint32_t BIT18:  1;
        uint32_t BIT19:  1;
        uint32_t BIT20:  1;
        uint32_t BIT21:  1;
        uint32_t BIT22:  1;
        uint32_t BIT23:  1;
        uint32_t BIT24:  1;
        uint32_t BIT25:  1;
        uint32_t BIT26:  1;
        uint32_t BIT27:  1;
        uint32_t BIT28:  1;
        uint32_t BIT29:  1;
        uint32_t BIT30:  1;
        uint32_t BIT31:  1;
    }BITS;
} __32BIT;

#endif