 #define BIT(a) (1UL << (a))
#define MY_ISR(vector) \
    void vector(void) __attribute__((signal, used, externally_visible)); \
    void vector(void)
