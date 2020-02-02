//
//  Linuxfuncs.hpp
//  VoodooI2CSynaptics
//
//  Created by PHBZ on 2020/1/3.
//  Copyright © 2020年 Alexandre Daoud. All rights reserved.
//

#ifndef Linuxfuncs_h
#define Linuxfuncs_h

#define __round_mask(x, y) ((__typeof__(x))((y)-1))
#define round_down(x, y) ((x) & ~__round_mask(x, y))
#define unlikely(x) __builtin_expect(!!(x), 0)

unsigned int hweight32(unsigned int w)
{
#ifdef CONFIG_ARCH_HAS_FAST_MULTIPLIER
    w -= (w >> 1) & 0x55555555;
    w =  (w & 0x33333333) + ((w >> 2) & 0x33333333);
    w =  (w + (w >> 4)) & 0x0f0f0f0f;
    return (w * 0x01010101) >> 24;
#else
    unsigned int res = w - ((w >> 1) & 0x55555555);
    res = (res & 0x33333333) + ((res >> 2) & 0x33333333);
    res = (res + (res >> 4)) & 0x0F0F0F0F;
    res = res + (res >> 8);
    return (res + (res >> 16)) & 0x000000FF;
#endif
}

unsigned long hweight64(uint64_t w)
{
#if BITS_PER_LONG == 32
    return __sw_hweight32((unsigned int)(w >> 32)) +
    __sw_hweight32((unsigned int)w);
#elif BITS_PER_LONG == 64
#ifdef CONFIG_ARCH_HAS_FAST_MULTIPLIER
    w -= (w >> 1) & 0x5555555555555555ul;
    w =  (w & 0x3333333333333333ul) + ((w >> 2) & 0x3333333333333333ul);
    w =  (w + (w >> 4)) & 0x0f0f0f0f0f0f0f0ful;
    return (w * 0x0101010101010101ul) >> 56;
#else
    uint64_t res = w - ((w >> 1) & 0x5555555555555555ul);
    res = (res & 0x3333333333333333ul) + ((res >> 2) & 0x3333333333333333ul);
    res = (res + (res >> 4)) & 0x0F0F0F0F0F0F0F0Ful;
    res = res + (res >> 8);
    res = res + (res >> 16);
    return (res + (res >> 32)) & 0x00000000000000FFul;
#endif
#endif
}

unsigned long hweight_long(unsigned long w)
{
    return sizeof(w) == 4 ? hweight32(w) : hweight64(w);
}


void bitmap_set(unsigned long *map, unsigned int start, int len)
{
    unsigned long *p = map + BIT_WORD(start);
    const unsigned int size = start + len;
    int bits_to_set = BITS_PER_LONG - (start % BITS_PER_LONG);
    unsigned long mask_to_set = BITMAP_FIRST_WORD_MASK(start);
    
    while (len - bits_to_set >= 0) {
        *p |= mask_to_set;
        len -= bits_to_set;
        bits_to_set = BITS_PER_LONG;
        mask_to_set = ~0UL;
        p++;
    }
    if (len) {
        mask_to_set &= BITMAP_LAST_WORD_MASK(size);
        *p |= mask_to_set;
    }
}

int bitmap_weight(const unsigned long *bitmap, unsigned int bits)
{
    unsigned int k, lim = bits/BITS_PER_LONG;
    int w = 0;
    
    for (k = 0; k < lim; k++)
        w += hweight_long(bitmap[k]);
    
    if (bits % BITS_PER_LONG)
        w += hweight_long(bitmap[k] & BITMAP_LAST_WORD_MASK(bits));
    
    return w;
}

/**
 * __ffs - find first bit in word.
 * @word: The word to search
 *
 * Undefined if no bit exists, so code should check against 0 first.
 */
static unsigned long myffs(unsigned long word)
{
    int num = 0;
    
#if BITS_PER_LONG == 64
    if ((word & 0xffffffff) == 0) {
        num += 32;
        word >>= 32;
    }
#endif
    if ((word & 0xffff) == 0) {
        num += 16;
        word >>= 16;
    }
    if ((word & 0xff) == 0) {
        num += 8;
        word >>= 8;
    }
    if ((word & 0xf) == 0) {
        num += 4;
        word >>= 4;
    }
    if ((word & 0x3) == 0) {
        num += 2;
        word >>= 2;
    }
    if ((word & 0x1) == 0)
        num += 1;
    return num;
}

unsigned long find_first_bit(const unsigned long *addr, unsigned long size)
{
    unsigned long idx;
    
    for (idx = 0; idx * BITS_PER_LONG < size; idx++) {
        if (addr[idx])
            return min(idx * BITS_PER_LONG + myffs(addr[idx]), size);
    }
    
    return size;
}

static inline unsigned long _find_next_bit(const unsigned long *addr1, const unsigned long *addr2, unsigned long nbits, unsigned long start, unsigned long invert)
{
    unsigned long tmp;
    
    if (unlikely(start >= nbits)){
        return nbits;
    }
    
    tmp = addr1[start / BITS_PER_LONG];
    if (addr2)
        tmp &= addr2[start / BITS_PER_LONG];
    tmp ^= invert;
    
    /* Handle 1st word. */
    tmp &= BITMAP_FIRST_WORD_MASK(start);
    start = round_down(start, BITS_PER_LONG);
    
    while (!tmp) {
        start += BITS_PER_LONG;
        if (start >= nbits)
            return nbits;
        
        tmp = addr1[start / BITS_PER_LONG];
        if (addr2)
            tmp &= addr2[start / BITS_PER_LONG];
        tmp ^= invert;
    }
    
    return min(start + myffs(tmp), nbits);
}


/*
 * Find the next set bit in a memory region.
 */
unsigned long find_next_bit(const unsigned long *addr, unsigned long size,
                            unsigned long offset)
{
    return _find_next_bit(addr, NULL, size, offset, 0UL);
}



#endif /* Linuxfuncs_h */
