#include "cmsis_os2.h"
#include "hal_i2c.h"
#include "ipmi.h"
#include "kcs.h"
#include "libutil.h"
#include "plat_ipmb.h"
#include "plat_i2c.h"
#include "timer.h"
#include <kernel.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
#include "plat_ipmb.h"
/*
 * If MAX_IPMB_IDX which define by plat_ipmb.h is not equal to zero 
 * then compile ipmb.c to avoid creating redundant memory space.
 */
#if MAX_IPMB_IDX

static struct k_mutex mutex_id[MAX_IPMB_IDX]; // mutex for sequence linked list insert/find
static struct k_mutex mutex_send_req, mutex_send_res, mutex_read;
static const struct device *dev_ipmb[I2C_BUS_MAX_NUM];

char __aligned(4) ipmb_txqueue_buffer[MAX_IPMB_IDX][IPMB_TXQUEUE_LEN * sizeof(struct ipmi_msg_cfg)];
char __aligned(4) ipmb_rxqueue_buffer[MAX_IPMB_IDX][IPMB_RXQUEUE_LEN * sizeof(struct ipmi_msg)];
struct k_msgq ipmb_txqueue[MAX_IPMB_IDX];
struct k_msgq ipmb_rxqueue[MAX_IPMB_IDX];

struct k_thread IPMB_SeqTimeout;
K_KERNEL_STACK_MEMBER(IPMB_SeqTimeout_stack, IPMB_SEQ_TIMEOUT_STACK_SIZE);

K_THREAD_STACK_EXTERN(ipmb_rx_stack);
K_THREAD_STACK_EXTERN(ipmb_tx_stack);
K_THREAD_STACK_ARRAY_DEFINE(ipmb_rx_stacks, MAX_IPMB_IDX, IPMB_RX_STACK_SIZE);
K_THREAD_STACK_ARRAY_DEFINE(ipmb_tx_stacks, MAX_IPMB_IDX, IPMB_TX_STACK_SIZE);
static struct k_thread IPMB_TX[MAX_IPMB_IDX];
static struct k_thread IPMB_RX[MAX_IPMB_IDX];
static k_tid_t IPMB_RX_ID[MAX_IPMB_IDX];
static k_tid_t IPMB_TX_ID[MAX_IPMB_IDX];

IPMB_config *IPMB_config_table;
ipmi_msg_cfg *P_start[MAX_IPMB_IDX],
	*P_temp; // pointer for sequence queue(link list)
static unsigned int seq_current_count[MAX_IPMB_IDX] = { 0 };
static uint32_t tick_fix;
static uint32_t sys_tick_freq;

static uint8_t current_seq[MAX_IPMB_IDX]; // Sequence in BIC for sending
	// sequence to other IPMB devices
static bool seq_table[MAX_IPMB_IDX][SEQ_NUM]; // Sequence table in BIC for register record

ipmb_error validate_checksum(uint8_t *buffer, uint8_t buffer_len);
ipmb_error ipmb_encode(uint8_t *buffer, ipmi_msg *msg);
ipmb_error ipmb_decode(ipmi_msg *msg, uint8_t *buffer, uint8_t len);
ipmb_error ipmb_notify_client(ipmi_msg_cfg *msg_cfg);

uint8_t IPMB_inf_index_map[RESERVED]; // map IPMB source/target interface to bus

/* Map channel to index */
void channel_index_mapping(void)
{
	uint8_t channel, index_num;
	memset(IPMB_inf_index_map, RESERVED, sizeof(IPMB_inf_index_map));
	for (index_num = 0; IPMB_config_table[index_num].index != RESERVED_IDX; index_num++) {
		for (channel = 0; channel < RESERVED; channel++) {
			if (IPMB_config_table[index_num].channel == channel) {
				IPMB_inf_index_map[channel] = index_num;
				break;
			}
		}
	}
}

uint8_t calculate_checksum(uint8_t *buffer, uint8_t range)
{
	uint8_t checksum = 0;
	uint8_t i;

	checksum -= buffer[0] << 1; // Use 7bit address for I2C transection, but 8bit
		// address for checksum calculate

	for (i = 1; i < range; i++) {
		checksum -= buffer[i];
	}

	return checksum;
}

ipmb_error validate_checksum(uint8_t *buffer, uint8_t buffer_len)
{
	if (buffer == NULL) {
		return IPMB_ERROR_UNKNOWN;
	}

	uint8_t header_checksum = buffer[2];
	uint8_t msg_checksum = buffer[buffer_len - 1];
	uint8_t calc_header_checksum = calculate_checksum(buffer, IPMI_HEADER_CHECKSUM_POSITION);
	uint8_t calc_msg_checksum = calculate_checksum(buffer, buffer_len - 1);

	if (header_checksum == calc_header_checksum) {
		if (msg_checksum == calc_msg_checksum) {
			return IPMB_ERROR_SUCCESS;
		}

		return IPMB_ERROR_HDR_CHECKSUM;
	}
	return IPMB_ERROR_MSG_CHECKSUM;
}

void unregister_seq(uint8_t index, uint8_t seq_num)
{
	seq_table[index][seq_num] = 0;
}

void register_seq(uint8_t index, uint8_t seq_num)
{
	seq_table[index][seq_num] = 1;
}

uint8_t get_free_seq(uint8_t index)
{
	uint8_t count = 0;

	do {
		current_seq[index] = (current_seq[index] + 1) & 0x3f;
		if (!seq_table[index][current_seq[index]]) {
			break;
		}

		count++; // break in case no free seq found
		if (count == SEQ_NUM) {
			printf("IPMB[%x] free seq not found!!\n", index);
			break;
		}
	} while (1);

	return current_seq[index];
}

/* Record IPMB request for checking response sequence and finding source
 * sequence for bridge command */
void insert_req_ipmi_msg(ipmi_msg_cfg *pnode, ipmi_msg *msg, uint8_t index)
{
	ipmi_msg_cfg *ptr_start = pnode;
	int ret;

	ret = k_mutex_lock(&mutex_id[index], K_MSEC(1000));
	if (ret) {
		printf("[%s] Failed to lock the mutex(%d)\n", __func__, ret);
		return;
	} else {
		// No more room, remove the first node.
		if (seq_current_count[index] == MAX_SEQ_QUENE) {
			ipmi_msg_cfg *temp;
			temp = pnode->next;
			if (temp == NULL) {
				printf("[%s] Not a circular linked list", __func__);
				k_mutex_unlock(&mutex_id[index]);
				return;
			}

			pnode->next = temp->next;
			SAFE_FREE(temp);
			seq_current_count[index]--;
		}

		while (pnode && pnode->next != ptr_start) {
			pnode = pnode->next;
		}

		if (pnode == NULL) {
			printf("[%s] Not a circular linked list", __func__);
			k_mutex_unlock(&mutex_id[index]);
			return;
		}

		/* Allocate memory for the new node and put data in it.*/
		pnode->next = malloc(sizeof(ipmi_msg_cfg));
		if (pnode->next == NULL) {
			k_mutex_unlock(&mutex_id[index]);
			return;
		}
		pnode = pnode->next;

		msg->timestamp = osKernelGetSysTimerCount();
		memcpy(&pnode->buffer, msg, sizeof(ipmi_msg));
		register_seq(index, pnode->buffer.seq_target);

		pnode->next = ptr_start;
		seq_current_count[index]++;
		k_mutex_unlock(&mutex_id[index]);
		return;
	}
}

/* Find if any IPMB request record match receiving response */
bool find_req_ipmi_msg(ipmi_msg_cfg *pnode, ipmi_msg *msg, uint8_t index)
{
	ipmi_msg_cfg *ptr_start = pnode;
	int ret;

	ret = k_mutex_lock(&mutex_id[index], K_MSEC(1000));
	if (ret) {
		printf("[%s] Failed to lock the mutex(%d)\n", __func__, ret);
		return false;
	}

	while ((pnode != NULL && pnode->next != ptr_start) &&
	       (((pnode->next)->buffer.netfn != (msg->netfn - 1)) ||
		((pnode->next)->buffer.cmd != msg->cmd) ||
		((pnode->next)->buffer.seq_target != msg->seq_target))) {
		pnode = pnode->next;
	}

	if (pnode == NULL) {
		printf("pnode list should be circular, list end found\n");
		k_mutex_unlock(&mutex_id[index]);
		return false;
	}

	if (pnode->next == ptr_start) {
		printf("no req match recv resp\n");
		printf("node netfn: %x,cmd: %x, seq_t: %x\n", (pnode->next)->buffer.netfn,
		       (pnode->next)->buffer.cmd, (pnode->next)->buffer.seq_target);
		printf("msg netfn: %x,cmd: %x, seq_t: %x\n\n", msg->netfn, msg->cmd,
		       msg->seq_target);
		k_mutex_unlock(&mutex_id[index]);
		return false;
	}

	/* Now pointer points to a node and the node next to it has to be removed */
	ipmi_msg_cfg *temp;
	temp = pnode->next;

	if (temp == NULL) {
		printf("pnode list should be circular, list end found\n");
		k_mutex_unlock(&mutex_id[index]);
		return false;
	}

	// find source sequence for responding
	msg->seq_source = temp->buffer.seq_source;
	unregister_seq(index, temp->buffer.seq_target);

	msg->InF_source = temp->buffer.InF_source;
	msg->InF_target = temp->buffer.InF_target;
	/*temp points to the node which has to be removed*/
	pnode->next = temp->next;
	/*We removed the node which is next to the pointer (which is also temp) */

	/* Because we deleted the node, we no longer require the memory used for it
   * free() will deallocate the memory.
   */
	SAFE_FREE(temp);
	seq_current_count[index]--;

	k_mutex_unlock(&mutex_id[index]);
	return true;
}

void IPMB_TXTask(void *pvParameters, void *arvg0, void *arvg1)
{
	if (pvParameters == NULL) {
		return;
	}

	struct ipmi_msg_cfg *current_msg_tx;
	IPMB_config ipmb_cfg;
	I2C_MSG *i2c_msg;
	uint8_t ipmb_buffer_tx[IPMI_MSG_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH];
	uint8_t ret = 0;

	memcpy(&ipmb_cfg, (IPMB_config *)pvParameters, sizeof(IPMB_config));

	while (1) {
		current_msg_tx = (struct ipmi_msg_cfg *)malloc(sizeof(struct ipmi_msg_cfg));
		if (current_msg_tx == NULL) {
			k_msleep(10);
			continue;
		}

		k_msgq_get(&ipmb_txqueue[ipmb_cfg.index], (ipmi_msg_cfg *)current_msg_tx,
			   K_FOREVER); // Wait for OS queue send interrupt

		if (IS_RESPONSE(current_msg_tx->buffer)) { // Send a response message
			if (current_msg_tx->retries > IPMB_TX_RETRY_TIME) {
				printf("Reach the MAX retry times for sending a response message, "
				       "source(%d)\n",
				       current_msg_tx->buffer.InF_source);
				goto cleanup;
			}

			// Fix IPMB target address
			current_msg_tx->buffer.dest_addr = ipmb_cfg.channel_target_address;
			// Encode the IPMB message
			ipmb_encode(&ipmb_buffer_tx[0], &current_msg_tx->buffer);
			uint8_t resp_tx_size =
				current_msg_tx->buffer.data_len + IPMB_RESP_HEADER_LENGTH;

			if (ipmb_cfg.interface == I2C_IF) {
				int retry = 0;
				do {
					i2c_msg = malloc(sizeof(I2C_MSG));
					if (i2c_msg == NULL) {
						k_msleep(10);
					} else {
						break;
					}
					retry++;
				} while (retry < MEM_ALLOCATE_RETRY_TIME);
				if (i2c_msg == NULL) {
					printf("[%s] Failed to allocate memory for I2C resp msg\n",
					       __func__);
					goto cleanup;
				}

				i2c_msg->bus = ipmb_cfg.bus;
				i2c_msg->target_addr = ipmb_cfg.channel_target_address;
				i2c_msg->tx_len = resp_tx_size;
				memcpy(&i2c_msg->data[0], &ipmb_buffer_tx[1], resp_tx_size);
				ret = i2c_master_write(i2c_msg, I2C_RETRY_TIME);
				SAFE_FREE(i2c_msg);
			} else {
				printf("[%s] Unsupported interface(%d) for index(%d)\n", __func__,
				       ipmb_cfg.interface, ipmb_cfg.index);
				goto cleanup;
			}

			if (ret) {
				// Message couldn't be transmitted right now, increase retry counter and
				// try again later
				current_msg_tx->retries++;
				k_msgq_put(&ipmb_txqueue[ipmb_cfg.index], current_msg_tx,
					   K_NO_WAIT);
				k_msleep(IPMB_RETRY_DELAY_MS);
			} else {
				if (DEBUG_IPMB) {
					printf("[%s] Send a response message, from(%d) to(%d) netfn(0x%x) "
					       "cmd(0x%x) CC(0x%x)\n",
					       __func__, current_msg_tx->buffer.InF_source,
					       current_msg_tx->buffer.InF_target,
					       current_msg_tx->buffer.netfn,
					       current_msg_tx->buffer.cmd,
					       current_msg_tx->buffer.completion_code);
					printf("response data[%d](",
					       current_msg_tx->buffer.data_len);
					for (int i = 0; i < current_msg_tx->buffer.data_len + 1;
					     i++) {
						printf(" %x", current_msg_tx->buffer.data[i]);
					}
					printf(")\n");
				}
			}

		} else { // Send a request message (bridge)
			ipmb_encode(&ipmb_buffer_tx[0], &current_msg_tx->buffer);

			uint8_t req_tx_size =
				current_msg_tx->buffer.data_len + IPMB_REQ_HEADER_LENGTH;

			if (ipmb_cfg.interface == I2C_IF) {
				int retry = 0;
				do {
					i2c_msg = malloc(sizeof(I2C_MSG));
					if (i2c_msg == NULL) {
						k_msleep(10);
					} else {
						break;
					}
					retry++;
				} while (retry < MEM_ALLOCATE_RETRY_TIME);
				if (i2c_msg == NULL) {
					printf("[%s] Failed to allocate memory for I2C resp msg\n",
					       __func__);
					goto cleanup;
				}

				i2c_msg->bus = ipmb_cfg.bus;
				i2c_msg->target_addr = ipmb_cfg.channel_target_address;
				i2c_msg->tx_len = req_tx_size;
				memcpy(&i2c_msg->data[0], &ipmb_buffer_tx[1], req_tx_size);

				current_msg_tx->buffer.seq_target = current_msg_tx->buffer.seq;
				insert_req_ipmi_msg(P_start[ipmb_cfg.index],
						    &current_msg_tx->buffer, ipmb_cfg.index);
				if (DEBUG_IPMB) {
					printf("[%s] Send a request message, from(%d) to(%d) netfn(0x%x) "
					       "cmd(0x%x) CC(0x%x)\n",
					       __func__, current_msg_tx->buffer.InF_source,
					       current_msg_tx->buffer.InF_target,
					       current_msg_tx->buffer.netfn,
					       current_msg_tx->buffer.cmd,
					       current_msg_tx->buffer.completion_code);
					printf("request data[%d](",
					       current_msg_tx->buffer.data_len);
					for (int i = 0; i < current_msg_tx->buffer.data_len + 1;
					     i++) {
						printf("%x ", current_msg_tx->buffer.data[i]);
					}
					printf(")\n");
				}

				ret = i2c_master_write(i2c_msg, I2C_RETRY_TIME);
				SAFE_FREE(i2c_msg);
			} else {
				printf("[%s] Unsupported interface(%d) for index(%d)\n", __func__,
				       ipmb_cfg.interface, ipmb_cfg.index);
				goto cleanup;
			}

			if (ret) {
				find_req_ipmi_msg(P_start[ipmb_cfg.index],
						  &(current_msg_tx->buffer), ipmb_cfg.index);

				current_msg_tx->retries += 1;

				if (current_msg_tx->retries > IPMB_TX_RETRY_TIME) {
					if (current_msg_tx->buffer.InF_source == RESERVED) {
						printf("[%s] The request message is from RESERVED\n",
						       __func__);
					} else if (current_msg_tx->buffer.InF_source == SELF) {
						printf("[%s] Failed to send command\n", __func__);
					} else if (current_msg_tx->buffer.InF_source == HOST_KCS) {
#ifdef CONFIG_IPMI_KCS_ASPEED
						uint8_t *kcs_buff;
						kcs_buff = malloc(KCS_BUFF_SIZE * sizeof(uint8_t));
						if (kcs_buff ==
						    NULL) { // allocate fail, retry allocate
							k_msleep(10);
							kcs_buff = malloc(KCS_BUFF_SIZE *
									  sizeof(uint8_t));
							if (kcs_buff == NULL) {
								printf("IPMB_TXTask: Fail to malloc for kcs_buff\n");
								SAFE_FREE(current_msg_tx);
								continue;
							}
						}
						current_msg_tx->buffer.completion_code =
							CC_CAN_NOT_RESPOND;
						kcs_buff[0] = current_msg_tx->buffer.netfn << 2;
						kcs_buff[1] = current_msg_tx->buffer.cmd;
						kcs_buff[2] =
							current_msg_tx->buffer.completion_code;
						if (current_msg_tx->buffer.data_len > 0) {
							memcpy(&kcs_buff[3],
							       &current_msg_tx->buffer.data[0],
							       current_msg_tx->buffer.data_len);
						}

						kcs_write(kcs_buff,
							  current_msg_tx->buffer.data_len +
								  3); // data len + netfn + cmd + cc
						SAFE_FREE(kcs_buff);
#endif
					} else {
						// Return the error code(node busy) to the source channel
						current_msg_tx->buffer.data_len = 0;
						current_msg_tx->buffer.netfn = NETFN_OEM_1S_REQ;
						current_msg_tx->buffer.cmd = CMD_OEM_1S_MSG_OUT;
						current_msg_tx->buffer.completion_code =
							CC_NODE_BUSY;

						if (ipmb_send_response(
							    &current_msg_tx->buffer,
							    IPMB_inf_index_map[current_msg_tx->buffer
										       .InF_source]) !=
						    IPMB_ERROR_SUCCESS) {
							printf("[%s][%d] Failed to send IPMB response message",
							       __func__, __LINE__);
						}
					}

					printf("[%s] Failed to send a request message, from(%d) to(%d) "
					       "netfn(0x%x) cmd(0x%x)\n",
					       __func__, current_msg_tx->buffer.InF_source,
					       current_msg_tx->buffer.InF_target,
					       current_msg_tx->buffer.netfn,
					       current_msg_tx->buffer.cmd);
				} else {
					k_msgq_put(&ipmb_txqueue[ipmb_cfg.index], current_msg_tx,
						   K_NO_WAIT);
					k_msleep(IPMB_RETRY_DELAY_MS);
				}
			}
		}

	cleanup:
		SAFE_FREE(current_msg_tx);
		k_msleep(IPMB_POLLING_TIME_MS);
	}
}

void IPMB_RXTask(void *pvParameters, void *arvg0, void *arvg1)
{
	if (pvParameters == NULL) {
		return;
	}

	struct ipmi_msg_cfg *current_msg_rx;
	struct IPMB_config ipmb_cfg;
	struct ipmb_msg *ipmb_msg = NULL;
	uint8_t *ipmb_buffer_rx;
	uint8_t rx_len;
	static uint16_t i = 0;
	int ret;

	memcpy(&ipmb_cfg, (IPMB_config *)pvParameters, sizeof(IPMB_config));

	if (DEBUG_IPMB) {
		printf("[%s] IPMB RXTask thread, bus(%d) index(%d)\n", __func__, ipmb_cfg.bus,
		       ipmb_cfg.index);
	}

	while (1) {
		current_msg_rx = (struct ipmi_msg_cfg *)malloc(sizeof(struct ipmi_msg_cfg));
		if (current_msg_rx == NULL) {
			k_msleep(10); // allocate fail, retry later
			continue;
		}
		ipmb_buffer_rx =
			malloc((IPMI_MSG_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH) * sizeof(uint8_t));
		if (ipmb_buffer_rx == NULL) {
			SAFE_FREE(current_msg_rx);
			k_msleep(10); // allocate fail, retry later
			continue;
		}

		rx_len = 0;
		if (ipmb_cfg.interface == I2C_IF) {
			ret = ipmb_slave_read(dev_ipmb[ipmb_cfg.bus], &ipmb_msg, &rx_len);
			if (!ret) {
				memcpy(ipmb_buffer_rx, (uint8_t *)ipmb_msg, rx_len);
				ipmb_buffer_rx[0] = ipmb_buffer_rx[0] >> 1;
			} else {
				goto cleanup;
			}
		} else {
			printf("[%s] Unsupported interface(%d) for index(%d)\n", __func__,
			       ipmb_cfg.interface, ipmb_cfg.index);
		}

		if (rx_len > 0) {
			if (DEBUG_IPMB) {
				printf("Received an IPMB message from bus(%d) data[%d](",
				       ipmb_cfg.bus, rx_len);
				for (i = 0; i < rx_len; i++) {
					printf("0x%x ", ipmb_buffer_rx[i + 1]);
				}
				printf(")\n");
			}

			/* Perform a checksum test on the message, if it doesn't pass, just ignore
       * it */
			if (validate_checksum(ipmb_buffer_rx, rx_len) != IPMB_ERROR_SUCCESS) {
				printf("Invalid IPMB message checksum, index(%d)\n",
				       ipmb_cfg.index);
				goto cleanup;
			}

			/* Clear our local buffer before writing new data into it */
			ipmb_error ret;
			ret = ipmb_decode(&(current_msg_rx->buffer), ipmb_buffer_rx, rx_len);

			if (ret != IPMB_ERROR_SUCCESS) {
				printf("Failed to decode IPMI message, ret(%d)\n", ret);
			}

			if (DEBUG_IPMB) {
				printf("Decode the IPMB message, netfn(0x%x) Cmd(0x%x) Data[%d](",
				       current_msg_rx->buffer.netfn, current_msg_rx->buffer.cmd,
				       current_msg_rx->buffer.data_len);
				for (i = 0; i < current_msg_rx->buffer.data_len; i++) {
					printf("0x%x ", current_msg_rx->buffer.data[i]);
				}
				printf(")\n");
			}

			if (IS_RESPONSE(current_msg_rx->buffer)) { // Response message
				/* Find the corresponding request message*/
				current_msg_rx->buffer.seq_target = current_msg_rx->buffer.seq;
				if (find_req_ipmi_msg(P_start[ipmb_cfg.index],
						      &(current_msg_rx->buffer), ipmb_cfg.index)) {
					if (DEBUG_IPMB) {
						printf("Found the corresponding request message, \
								from(0x%x) to(0x%x) target_seq_num(%d)\n",
						       current_msg_rx->buffer.InF_source,
						       current_msg_rx->buffer.InF_target,
						       current_msg_rx->buffer.seq_target);
					}

					if (current_msg_rx->buffer.InF_source ==
					    SELF) { // Send from other thread
						struct ipmi_msg current_msg;
						current_msg =
							(struct ipmi_msg)current_msg_rx->buffer;
						k_msgq_put(
							&ipmb_rxqueue[IPMB_inf_index_map
									      [current_msg_rx->buffer
										       .InF_target]],
							&current_msg, K_NO_WAIT);
					} else if (current_msg_rx->buffer.InF_source == HOST_KCS) {
#ifdef CONFIG_IPMI_KCS_ASPEED
						uint8_t *kcs_buff;
						int retry = 0;
						do {
							kcs_buff = malloc(KCS_BUFF_SIZE *
									  sizeof(uint8_t));
							if (kcs_buff == NULL) {
								k_msleep(10);
							} else {
								break;
							}
							retry++;
						} while (retry < MEM_ALLOCATE_RETRY_TIME);
						if (kcs_buff == NULL) {
							printf("[%s] Failed to allocate memory for I2C resp msg\n",
							       __func__);
							goto cleanup;
						}
						kcs_buff[0] = current_msg_rx->buffer.netfn << 2;
						kcs_buff[1] = current_msg_rx->buffer.cmd;
						kcs_buff[2] =
							current_msg_rx->buffer.completion_code;
						if (current_msg_rx->buffer.data_len > 0) {
							memcpy(&kcs_buff[3],
							       &current_msg_rx->buffer.data[0],
							       current_msg_rx->buffer.data_len);
						}

						kcs_write(kcs_buff,
							  current_msg_rx->buffer.data_len +
								  3); // data len + netfn + cmd + cc
						SAFE_FREE(kcs_buff);
#endif
					} else if (current_msg_rx->buffer.InF_source == ME_IPMB) {
						ipmi_msg *bridge_msg =
							(ipmi_msg *)malloc(sizeof(ipmi_msg));
						memset(bridge_msg, 0, sizeof(ipmi_msg));

						bridge_msg->netfn =
							current_msg_rx->buffer.netfn -
							1; // fix response netfn and would be shift
						// later in ipmb_send_response
						bridge_msg->cmd = current_msg_rx->buffer.cmd;
						bridge_msg->completion_code =
							current_msg_rx->buffer.completion_code;
						bridge_msg->seq = current_msg_rx->buffer.seq_source;
						bridge_msg->data_len =
							current_msg_rx->buffer.data_len;
						memcpy(&bridge_msg->data[0],
						       &current_msg_rx->buffer.data[0],
						       current_msg_rx->buffer.data_len);

						if (DEBUG_IPMB) {
							printf("Send the response message to ME, source_seq_num(%d), "
							       "target_seq_num(%d)\n",
							       current_msg_rx->buffer.seq_source,
							       current_msg_rx->buffer.seq_target);
							printf("response data[%d](",
							       bridge_msg->data_len);
							for (i = 0; i < bridge_msg->data_len; i++) {
								printf("%x ", bridge_msg->data[i]);
							}
							printf(")\n");
						}

						if (ipmb_send_response(
							    bridge_msg,
							    IPMB_inf_index_map[current_msg_rx->buffer
										       .InF_source]) !=
						    IPMB_ERROR_SUCCESS) {
							printf("[%s][%d] Failed to send IPMB response message",
							       __func__, __LINE__);
						}

						SAFE_FREE(bridge_msg);
					} else { // Bridge response to other fru
						ipmi_msg *bridge_msg =
							(ipmi_msg *)malloc(sizeof(ipmi_msg));
						memset(bridge_msg, 0, sizeof(ipmi_msg));

						bridge_msg->data[0] =
							IANA_ID &
							0xFF; // Move target response to bridge response data
						bridge_msg->data[1] = (IANA_ID >> 8) & 0xFF;
						bridge_msg->data[2] = (IANA_ID >> 16) & 0xFF;
						bridge_msg->data[3] =
							IPMB_config_table[ipmb_cfg.index]
								.channel; // return response source as request target
						bridge_msg->data[4] =
							current_msg_rx->buffer
								.netfn; // Move target response to bridge response data
						bridge_msg->data[5] = current_msg_rx->buffer.cmd;
						bridge_msg->data[6] =
							current_msg_rx->buffer.completion_code;
						bridge_msg->data_len =
							current_msg_rx->buffer.data_len +
							7; // add 7 byte len for bridge header
						memcpy(&bridge_msg->data[7],
						       &current_msg_rx->buffer.data[0],
						       current_msg_rx->buffer.data_len);
						bridge_msg->netfn =
							NETFN_OEM_1S_REQ; // Add bridge response header
						bridge_msg->cmd = CMD_OEM_1S_MSG_OUT;
						bridge_msg->completion_code = CC_SUCCESS;
						bridge_msg->seq = current_msg_rx->buffer.seq_source;

						if (DEBUG_IPMB) {
							printf("Send the response message to the source(%d), "
							       "source_seq_num(%d), target_seq_num(%d)\n",
							       current_msg_rx->buffer.InF_source,
							       current_msg_rx->buffer.seq_source,
							       current_msg_rx->buffer.seq_target);
							printf("response data[%d](",
							       bridge_msg->data_len);
							for (i = 0; i < bridge_msg->data_len; i++) {
								printf("%x ", bridge_msg->data[i]);
							}
							printf(")\n");
						}

						if (ipmb_send_response(
							    bridge_msg,
							    IPMB_inf_index_map[current_msg_rx->buffer
										       .InF_source]) !=
						    IPMB_ERROR_SUCCESS) {
							printf("[%s][%d] Failed to send IPMB response message",
							       __func__, __LINE__);
						}

						SAFE_FREE(bridge_msg);
					}
				}

			} else { // Request message
				if (IPMB_config_table[ipmb_cfg.index].channel == ME_IPMB &&
				    (!pal_request_msg_to_BIC_from_ME(current_msg_rx->buffer.netfn,
								     /* Special Case:
                 * ME sends standard IPMI commands to BMC but not BIC.
                 * So, BIC should bridge ME request to BMC directly,
                 * instead of calling IPMI handler.
                 */
								     current_msg_rx->buffer.cmd))) {
					ipmi_msg *bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
					memset(bridge_msg, 0, sizeof(ipmi_msg));

					bridge_msg->data_len = current_msg_rx->buffer.data_len;
					bridge_msg->seq_source = current_msg_rx->buffer.seq;
					bridge_msg->InF_target = BMC_IPMB;
					bridge_msg->InF_source = ME_IPMB;
					bridge_msg->netfn = current_msg_rx->buffer.netfn;
					bridge_msg->cmd = current_msg_rx->buffer.cmd;
					if (bridge_msg->data_len != 0) {
						memcpy(&bridge_msg->data[0],
						       &current_msg_rx->buffer.data[0],
						       current_msg_rx->buffer.data_len);
					}

					if (ipmb_send_request(bridge_msg,
							      IPMB_inf_index_map[BMC_IPMB]) !=
					    IPMB_ERROR_SUCCESS) {
						printf("[%s] Failed to send the request message to BMC from ME",
						       __func__);
						bridge_msg->completion_code = CC_TIMEOUT;
						if (ipmb_send_response(
							    bridge_msg,
							    IPMB_inf_index_map
								    [bridge_msg->InF_source]) !=
						    IPMB_ERROR_SUCCESS) {
							printf("[%s][%d] Failed to send IPMB response message",
							       __func__, __LINE__);
						}
					}
					SAFE_FREE(bridge_msg);
				} else {
					/* The received message is a request
           * Record sequence number for later response
           */
					current_msg_rx->buffer.seq_source =
						current_msg_rx->buffer.seq;
					/* Record source interface for later bridge response */
					current_msg_rx->buffer.InF_source =
						IPMB_config_table[ipmb_cfg.index].channel;
					/* Notify the client about the new request */
					if (DEBUG_IPMB) {
						printf("[%s] Received a request message, netfn(0x%x) InfS(0x%x) "
						       "seq_s(%d)\n",
						       __func__, current_msg_rx->buffer.netfn,
						       current_msg_rx->buffer.InF_source,
						       current_msg_rx->buffer.seq_source);
					}
					ipmb_notify_client(current_msg_rx);
				}
			}
		}
	cleanup:
		SAFE_FREE(current_msg_rx);
		SAFE_FREE(ipmb_buffer_rx);
		k_msleep(IPMB_POLLING_TIME_MS);
	}
}

ipmb_error ipmb_send_request(ipmi_msg *req, uint8_t index)
{
	int ret;
	ret = k_mutex_lock(&mutex_send_req, K_MSEC(1000));
	if (ret) {
		printf("[%s] Failed to lock the mutex\n", __func__);
		return IPMB_ERROR_MUTEX_LOCK;
	}

	struct ipmi_msg_cfg req_cfg;
	/* Builds the message according to the IPMB specification */
	/* Copies data from the msg struct passed by caller */
	memcpy(&req_cfg.buffer.data, &req->data, req->data_len);
	req_cfg.buffer.netfn = req->netfn;
	req_cfg.buffer.cmd = req->cmd;
	req_cfg.buffer.data_len = req->data_len;
	req_cfg.buffer.InF_source = req->InF_source;
	req_cfg.buffer.InF_target = req->InF_target;
	req_cfg.buffer.completion_code = req->completion_code;
	/* Write necessary fields */
	req_cfg.buffer.dest_addr = IPMB_config_table[index].channel_target_address;
	req_cfg.buffer.dest_LUN = 0;
	req_cfg.buffer.src_addr = IPMB_config_table[index].self_address << 1;
	req_cfg.buffer.seq = get_free_seq(index);
	req_cfg.buffer.seq_source = req->seq_source;
	req_cfg.buffer.src_LUN = 0;
	req_cfg.retries = 0;

	if (DEBUG_IPMB) {
		uint8_t i;
		printf("Send req message, index(%d) cc(0x%x) data[%d](", index,
		       req_cfg.buffer.completion_code, req_cfg.buffer.data_len);
		for (i = 0; i < req_cfg.buffer.data_len; i++) {
			printf("0x%x ", req_cfg.buffer.data[i]);
		}
		printf(")\n");
	}

	/* Blocks here until is able put message in tx queue */
	if (k_msgq_put(&ipmb_txqueue[index], &req_cfg, K_FOREVER) != osOK) {
		k_mutex_unlock(&mutex_send_req);
		return IPMB_ERROR_FAILURE;
	}
	k_mutex_unlock(&mutex_send_req);
	return IPMB_ERROR_SUCCESS;
}

ipmb_error ipmb_send_response(ipmi_msg *resp, uint8_t index)
{
	int ret;
	ret = k_mutex_lock(&mutex_send_res, K_MSEC(1000));
	if (ret) {
		printf("[%s] Failed to lock the mutex\n", __func__);
		return IPMB_ERROR_MUTEX_LOCK;
	}

	struct ipmi_msg_cfg resp_cfg;

	/* Builds the message according to the IPMB specification */
	/* Copies data from the response msg struct passed by caller */
	memcpy(&resp_cfg.buffer.data, &resp->data, resp->data_len);
	resp_cfg.buffer.netfn = resp->netfn;
	resp_cfg.buffer.cmd = resp->cmd;
	resp_cfg.buffer.data_len = resp->data_len;
	resp_cfg.buffer.InF_source = resp->InF_source;
	resp_cfg.buffer.InF_target = resp->InF_target;
	resp_cfg.buffer.completion_code = resp->completion_code;
	resp_cfg.buffer.dest_addr = IPMB_config_table[index].channel_target_address;
	resp_cfg.buffer.netfn = resp->netfn + 1;
	resp_cfg.buffer.dest_LUN = resp->src_LUN;
	resp_cfg.buffer.src_addr = IPMB_config_table[index].self_address << 1;
	resp_cfg.buffer.seq = resp->seq;
	resp_cfg.buffer.seq_source = resp_cfg.buffer.seq;
	resp_cfg.buffer.InF_source = resp->InF_source;
	resp_cfg.buffer.src_LUN = resp->dest_LUN;
	resp_cfg.buffer.cmd = resp->cmd;
	resp_cfg.retries = 0;

	if (DEBUG_IPMB) {
		uint8_t i;
		printf("Send resp message, index(%d) cc(0x%x) data[%d](", index,
		       resp_cfg.buffer.completion_code, resp_cfg.buffer.data_len);
		for (i = 0; i < resp_cfg.buffer.data_len; i++) {
			printf("0x%x ", resp_cfg.buffer.data[i]);
		}
		printf(")\n");
	}

	/* Blocks here until is able put message in tx queue */
	if (k_msgq_put(&ipmb_txqueue[index], &resp_cfg, K_FOREVER) != osOK) {
		k_mutex_unlock(&mutex_send_res);
		return IPMB_ERROR_FAILURE;
	}
	k_mutex_unlock(&mutex_send_res);
	return IPMB_ERROR_SUCCESS;
}

ipmb_error ipmb_read(ipmi_msg *msg, uint8_t index)
{
	// Set mutex timeout 10ms more than messageQueue timeout, prevent mutex
	// timeout before messageQueue
	k_mutex_lock(&mutex_read, K_MSEC(IPMB_SEQ_TIMEOUT_MS + 10));
	// Reset a Message Queue to initialize empty state
	k_msgq_purge(&ipmb_rxqueue[index]);

	if (ipmb_send_request(msg, index) != IPMB_ERROR_SUCCESS) {
		printf("[%s] Failed to send IPMB request message", __func__);
		k_mutex_unlock(&mutex_read);
		return IPMB_ERROR_FAILURE;
	}

	if (k_msgq_get(&ipmb_rxqueue[index], (ipmi_msg *)msg, K_MSEC(IPMB_SEQ_TIMEOUT_MS))) {
		k_mutex_unlock(&mutex_read);
		return IPMB_ERROR_GET_MESSAGE_QUEUE;
	}

	k_mutex_unlock(&mutex_read);
	return IPMB_ERROR_SUCCESS;
}

// Send message to IPMI message queue
ipmb_error ipmb_notify_client(ipmi_msg_cfg *msg_cfg)
{
	/* Sends only the ipmi msg, not the control struct */
	if (!IS_RESPONSE(msg_cfg->buffer)) {
		while (k_msgq_put(&ipmi_msgq, msg_cfg, K_NO_WAIT) != 0) {
			/* message queue is full: purge old data & try again */
			k_msgq_purge(&ipmi_msgq);
			printf("Retry to send message to IPMI message queue\n");
		}
	}
	return IPMB_ERROR_SUCCESS;
}

ipmb_error ipmb_encode(uint8_t *buffer, ipmi_msg *msg)
{
	uint8_t i = 0;

	/* Use this variable to address the buffer dynamically */

	buffer[i++] = msg->dest_addr;
	buffer[i++] =
		(((msg->netfn << 2) & IPMB_NETFN_MASK) | (msg->dest_LUN & IPMB_DEST_LUN_MASK));
	buffer[i++] = calculate_checksum(&buffer[0], IPMI_HEADER_CHECKSUM_POSITION);
	buffer[i++] = msg->src_addr;
	buffer[i++] = (((msg->seq << 2) & IPMB_SEQ_MASK) | (msg->src_LUN & IPMB_SRC_LUN_MASK));
	buffer[i++] = msg->cmd;
	if (IS_RESPONSE((*msg))) {
		buffer[i++] = msg->completion_code;
	}
	memcpy(&buffer[i], &msg->data[0], msg->data_len);
	i += msg->data_len;

	buffer[i] = calculate_checksum(&buffer[0], i);
	return IPMB_ERROR_SUCCESS;
}

ipmb_error ipmb_decode(ipmi_msg *msg, uint8_t *buffer, uint8_t len)
{
	/* Use this variable to address the buffer dynamically */
	uint8_t i = 0;

	msg->dest_addr = buffer[i++];
	msg->netfn = buffer[i] >> 2;
	msg->dest_LUN = (buffer[i++] & IPMB_DEST_LUN_MASK);
	msg->hdr_chksum = buffer[i++];
	msg->src_addr = buffer[i++];
	msg->seq = buffer[i] >> 2;
	msg->src_LUN = (buffer[i++] & IPMB_SRC_LUN_MASK);
	msg->cmd = buffer[i++];
	/* Checks if the message is a response and if so, fills the completion code
   * field */
	if (IS_RESPONSE((*msg))) {
		msg->completion_code = buffer[i++];
	}
	msg->data_len = (len > i + 1) ? len - i - 1 : 0;
	memcpy(&msg->data[0], &buffer[i], msg->data_len);
	msg->msg_chksum = buffer[len - 1];

	return IPMB_ERROR_SUCCESS;
}

void IPMB_SeqTimeout_handler(void *arug0, void *arug1, void *arug2)
{
	ipmi_msg_cfg *pnode;
	int ret;
	uint8_t index;
	uint32_t current_time;

	while (1) {
		k_msleep(IPMB_SEQ_TIMEOUT_MS);
		for (index = 0; index < MAX_IPMB_IDX; index++) {
			if (!IPMB_config_table[index].enable_status) {
				continue;
			}

			ret = k_mutex_lock(&mutex_id[index], K_MSEC(1000));

			if (ret) {
				printf("[%s] Failed to lock the mutex, ret(%d)\n", __func__, ret);
			} else {
				pnode = P_start[index];
				while (pnode->next != P_start[index]) {
					current_time = osKernelGetSysTimerCount();
					// The queue stay more than 1000 ms need to be killed
					if (current_time + tick_fix -
							    ((pnode->next)->buffer.timestamp) >
						    (IPMB_SEQ_TIMEOUT_MS / 1000) * sys_tick_freq ||
					    (current_time + tick_fix -
					     ((pnode->next)->buffer.timestamp)) &
						    0x80000000) {
						ipmi_msg_cfg *temp;
						temp = pnode->next;
						pnode->next = temp->next;
						unregister_seq(index, temp->buffer.seq_target);
						SAFE_FREE(temp);
						seq_current_count[index]--;
					}

					pnode = pnode->next;
				}

				k_mutex_unlock(&mutex_id[index]);
			}
		}
	}
}

static void register_target_device(void)
{
#ifdef DEV_IPMB_0
	dev_ipmb[0] = device_get_binding("IPMB_0");
	if (i2c_slave_driver_register(dev_ipmb[0]))
		printf("IPMB0: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_1
	dev_ipmb[1] = device_get_binding("IPMB_1");
	if (i2c_slave_driver_register(dev_ipmb[1]))
		printf("IPMB: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_2
	dev_ipmb[2] = device_get_binding("IPMB_2");
	if (i2c_slave_driver_register(dev_ipmb[2]))
		printf("IPMB2: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_3
	dev_ipmb[3] = device_get_binding("IPMB_3");
	if (i2c_slave_driver_register(dev_ipmb[3]))
		printf("IPMB3: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_4
	dev_ipmb[4] = device_get_binding("IPMB_4");
	if (i2c_slave_driver_register(dev_ipmb[4]))
		printf("IPMB4: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_5
	dev_ipmb[5] = device_get_binding("IPMB_5");
	if (i2c_slave_driver_register(dev_ipmb[5]))
		printf("IPMB5: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_6
	dev_ipmb[6] = device_get_binding("IPMB_6");
	if (i2c_slave_driver_register(dev_ipmb[6]))
		printf("IPMB6: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_7
	dev_ipmb[7] = device_get_binding("IPMB_7");
	if (i2c_slave_driver_register(dev_ipmb[7]))
		printf("IPMB7: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_8
	dev_ipmb[8] = device_get_binding("IPMB_8");
	if (i2c_slave_driver_register(dev_ipmb[8]))
		printf("IPMB8: Target Device driver not found.");
#endif
#ifdef DEV_IPMB_9
	dev_ipmb[9] = device_get_binding("IPMB_9");
	if (i2c_slave_driver_register(dev_ipmb[9]))
		printf("IPMB9: Target Device driver not found.");
#endif
}

void create_ipmb_threads(uint8_t index)
{
	osThreadAttr_t IPMB_TxTask_attr;
	osThreadAttr_t IPMB_RxTask_attr;

	memset(&IPMB_TxTask_attr, 0, sizeof(IPMB_TxTask_attr));
	memset(&IPMB_RxTask_attr, 0, sizeof(IPMB_RxTask_attr));
	memset(&seq_table[index], 0, sizeof(bool) * SEQ_NUM);

	P_start[index] = (void *)malloc(sizeof(struct ipmi_msg_cfg));
	if (P_start[index] == NULL) {
		printf("[%s], Memory allocation failed!\n", __func__);
		return;
	}

	P_temp = P_start[index];
	P_temp->next = P_temp;

	int i = 0, retry = 3;
	for (i = 0; i <= retry; ++i) {
		if (k_mutex_init(&mutex_id[index]) == 0) {
			break;
		}
	}
	if (i > retry) {
		printf("Failed to create threads,Tx(%s) Rx(%s) retry time(%d)\n",
		       IPMB_config_table[index].tx_thread_name,
		       IPMB_config_table[index].rx_thread_name, retry);
		return;
	}

	k_msgq_init(&ipmb_txqueue[index], ipmb_txqueue_buffer[index], sizeof(struct ipmi_msg_cfg),
		    IPMB_TXQUEUE_LEN);
	k_msgq_init(&ipmb_rxqueue[index], ipmb_rxqueue_buffer[index], sizeof(struct ipmi_msg),
		    IPMB_RXQUEUE_LEN);

	IPMB_TX_ID[index] =
		k_thread_create(&IPMB_TX[index], ipmb_tx_stacks[index], IPMB_TX_STACK_SIZE,
				IPMB_TXTask, (void *)&IPMB_config_table[index], NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMB_TX[index], IPMB_config_table[index].tx_thread_name);
	IPMB_RX_ID[index] =
		k_thread_create(&IPMB_RX[index], ipmb_rx_stacks[index], IPMB_RX_STACK_SIZE,
				IPMB_RXTask, (void *)&IPMB_config_table[index], NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMB_RX[index], IPMB_config_table[index].rx_thread_name);

	if (DEBUG_IPMB) {
		printf("Initial IPMB TX/RX threads, bus(0x%x), addr(0x%x)\n",
		       IPMB_config_table[index].bus,
		       IPMB_config_table[index].channel_target_address);
	}

	return;
}

__weak bool pal_load_ipmb_config(void)
{
	return true;
}

void ipmb_init(void)
{
	uint8_t index;
	register_target_device();

	IPMB_config_table = malloc(MAX_IPMB_IDX * sizeof(IPMB_config));
	if (IPMB_config_table == NULL) {
		printf("[%s] Failed to allocate memory\n", __func__);
		return;
	}

	bool ret = pal_load_ipmb_config();
	if (!ret) {
		printf("[%s] Failed to load IPMB configuration\n", __func__);
		return;
	}

	channel_index_mapping();

	memset(&current_seq, 0, sizeof(uint8_t) * MAX_IPMB_IDX);

	// Get the RTOS kernel system timer
	sys_tick_freq = osKernelGetSysTimerFreq();
	tick_fix = sys_tick_freq / 1000;

	// Initial mutex
	if (k_mutex_init(&mutex_send_req)) {
		printf("Failed to initialize IPMB send request mutex\n");
	}
	if (k_mutex_init(&mutex_send_res)) {
		printf("Failed to initialize IPMB send response mutex\n");
	}
	if (k_mutex_init(&mutex_read)) {
		printf("Failed to initialize IPMB read mutex\n");
	}

	// Create IPMB threads for each index
	for (index = 0; index < MAX_IPMB_IDX; index++) {
		if (IPMB_config_table[index].enable_status) {
			create_ipmb_threads(index);
		}
	}

	// Create IPMB sequence number timeout thread
	k_thread_create(&IPMB_SeqTimeout, IPMB_SeqTimeout_stack,
			K_THREAD_STACK_SIZEOF(IPMB_SeqTimeout_stack), IPMB_SeqTimeout_handler, NULL,
			NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMB_SeqTimeout, "IPMB_SeqTimeout");
}
#endif
