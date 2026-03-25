#define COUNTOF(x) (sizeof(x)/sizeof(x[0U])) //keep this at top
#define LIB_BUFFER_FIFO_CREATE(name, type, elements) \
    struct fifo_##name##_S { \
        type buffer[elements]; \
        size_t startPos; \
        size_t endPos; \
    } name

#define LIB_BUFFER_FIFO_RESERVE(name, n) \
    do { \
        size_t _n = (size_t)(n); \
        if (_n != 0U) { \
            (name)->endPos = ((name)->endPos + _n) % COUNTOF((name)->buffer); \
        } \
    } while (0)

#define LIB_BUFFER_FIFO_INSERT(name, val) \
    do { \
        (name)->buffer[(name)->endPos] = (val); \
        (name)->endPos = ((name)->endPos + 1U) % COUNTOF((name)->buffer); \
    } while (0)

#define LIB_BUFFER_FIFO_PEEK(name) (name)->buffer[(name)->startPos]
#define LIB_BUFFER_FIFO_PEEKN(name, index) \
    (name)->buffer[({ \
        int32_t _pos = (int32_t)(name)->startPos + (int32_t)(index); \
        int32_t _len = (int32_t)COUNTOF((name)->buffer); \
        _pos %= _len; \
        if (_pos < 0) { _pos += _len; } \
        _pos; \
    })]
#define LIB_BUFFER_FIFO_PEEKEND(name) (name)->buffer[(name)->endPos]

#define LIB_BUFFER_FIFO_POP(name) \
        LIB_BUFFER_FIFO_PEEK(name); \
        (name)->startPos = ((name)->startPos + 1U) % COUNTOF((name)->buffer) \

#define LIB_BUFFER_FIFO_GETMAXCONTINUOUS(name) \
    ((name)->endPos < (name)->startPos ? \
        ((name)->startPos - (name)->endPos - 1U) : \
        ((COUNTOF((name)->buffer) - (name)->endPos) - ((name)->startPos == 0U ? 1U : 0U)))

#define LIB_BUFFER_FIFO_GETLENGTH(name) \
    (((name)->endPos + COUNTOF((name)->buffer) - (name)->startPos) % COUNTOF((name)->buffer))

#define LIB_BUFFER_FIFO_CLEAR(name) LIB_BUFFER_CLEAR(name); \
                                    (name)->endPos = 0U; (name)->start