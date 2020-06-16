
#ifndef VCO_DEMI_ASSERTS_H
#define VCO_DEMI_ASSERTS_H

#include <esp32/rom/ets_sys.h>
#include <esp_compiler.h>
#include <stdint.h>
#include <stdbool.h>

#define configASSERT(a) if (unlikely(!(a))) {                                     \
        ets_printf("%s:%d (%s)- assert failed!\n", __FILE__, __LINE__,  \
                   __FUNCTION__);                                       \
        abort();                                                        \
        };

#endif //VCO_DEMI_ASSERTS_H
