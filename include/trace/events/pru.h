#undef TRACE_SYSTEM
#define TRACE_SYSTEM pru

#if !defined(_TRACE_PRU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_PRU_H

#include <linux/tracepoint.h>

TRACE_EVENT(
    /* "hello" is the subsystem name, "world" is the event name */
    pru_call,

    /* tracepoint function prototype */
    TP_PROTO(int foo, int bar),

    /* arguments for this tracepoint */
    TP_ARGS(foo, bar),

    /* LTTng doesn't need those */
    TP_STRUCT__entry(),
    TP_fast_assign(),
    TP_printk("", 0)
);

#endif

/* this part must be outside protection */
#include <trace/define_trace.h>
