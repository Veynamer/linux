#ifndef _WCN_PROCFS
#define _WCN_PROCFS

#include <misc/wcn_bus.h>

#define MDBG_WRITE_SIZE			(64)
#define MDBG_ASSERT_SIZE		(1024)
#define MDBG_LOOPCHECK_SIZE		(128)
#define MDBG_AT_CMD_SIZE		(128)

void mdbg_fs_channel_init(void);
void mdbg_fs_channel_destroy(void);
int proc_fs_init(void);
int mdbg_memory_alloc(void);
void proc_fs_exit(void);
int get_loopcheck_status(void);
void wakeup_loopcheck_int(void);
void loopcheck_ready_clear(void);
void loopcheck_ready_set(void);
void mdbg_assert_interface(char *str);
void wcn_assert_interface_async(enum wcn_source_type type, char *str);
int mdbg_assert_read(int channel, struct mbuf_t *head,
		     struct mbuf_t *tail, int num);
int mdbg_loopcheck_read(int channel, struct mbuf_t *head,
			struct mbuf_t *tail, int num);
int mdbg_at_cmd_read(int channel, struct mbuf_t *head,
		     struct mbuf_t *tail, int num);
#endif
