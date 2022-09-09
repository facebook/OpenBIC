#ifndef VERSION_H
#define VERSION_H

/*
    FIRMWARE REVISION_1
        [bit 0-3] BOARD_ID
        [bit 4-7] PROJECT_STAGE
            0x00 POC
            0x01 EVT
            0x02 DVT
            0x03 PVT
            0x04 MP

    FIRMWARE REVISION_2
        Count of release firmware at each stage.
*/
#define GET_FW_VERSION1(board_id, stage) ((board_id & 0x0F) | (stage << 4))

typedef enum {
	POC = 0,
	EVT,
	DVT,
	PVT,
	MP,
} PROJECT_STAGE;

#endif
