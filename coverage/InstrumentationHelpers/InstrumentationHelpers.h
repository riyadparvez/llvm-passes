
/*
 * called by the instrumentation before executing main
 */
extern "C"
void nop_init(uint64_t mainbb);

extern "C"
void nop_transition(uint64_t bb, int distance);

extern "C"
void nop_bb(uint64_t bb);

extern "C"
void nop_indirect_call();
