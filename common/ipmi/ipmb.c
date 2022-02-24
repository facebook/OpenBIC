#include <zephyr.h>
#include <kernel.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "cmsis_os2.h"
#include "hal_i2c.h"
#include "pal.h"
#include "timer.h"
#include "ipmi.h"
#include "kcs.h"

static struct k_mutex mutex_id[MAX_IPMB_IDX]; // mutex for sequence link list insert/find
static struct k_mutex mutex_send_req, mutex_send_res, mutex_read;
static const struct device *dev_ipmb[MAX_I2C_BUS_NUM];

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
ipmi_msg_cfg *P_start[MAX_IPMB_IDX], *P_temp; // pointer for sequence queue(link list)
static unsigned int seq_current_count[MAX_IPMB_IDX] = { 0 };
static uint32_t tick_fix;
static uint32_t sys_tick_freq;

static uint8_t
	current_seq[MAX_IPMB_IDX]; // Sequence in BIC for sending sequence to other IPMB devices
static bool seq_table[MAX_IPMB_IDX][SEQ_NUM]; // Sequence table in BIC for register record

ipmb_error ipmb_assert_chksum(uint8_t *buffer, uint8_t buffer_len);
ipmb_error ipmb_encode(uint8_t *buffer, ipmi_msg *msg);
ipmb_error ipmb_decode(ipmi_msg *msg, uint8_t *buffer, uint8_t len);
ipmb_error ipmb_notify_client(ipmi_msg_cfg *msg_cfg);

void Queue_timeout_handler(uint32_t arug0, uint32_t arug1);

uint8_t IPMB_inf_index_map[Reserve_IFs]; // map IPMB source/target interface to bus

// map IPMB interface to index
void map_inf_index(void)
{
	uint8_t inf, index_num;

	memset(IPMB_inf_index_map, Reserve_IFs, sizeof(IPMB_inf_index_map));
	for (index_num = 0; IPMB_config_table[index_num].index != IPMB_RESERVE_IDX; index_num++) {
		for (inf = 0; inf < Reserve_IFs; inf++) {
			if (IPMB_config_table[index_num].Inf_source == inf) {
				IPMB_inf_index_map[inf] = index_num;
				break;
			}
		}
	}
}

uint8_t calculate_chksum(uint8_t *buffer, uint8_t range)
{
	uint8_t chksum = 0;
	uint8_t i;

	chksum -=
		buffer[0]
		<< 1; // Use 7bit address for I2C transection, but 8bit address for checksum calculate

	for (i = 1; i < range; i++) {
		chksum -= buffer[i];
	}

	return chksum;
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
			printk("IPMB[%x] free seq not found!!\n", index);
			break;
		}
	} while (1);

	return current_seq[index];
}

// Record IPMB request for checking response sequence and finding source sequence for bridge command
void insert_node(ipmi_msg_cfg *pnode, ipmi_msg *msg, uint8_t index)
{
	ipmi_msg_cfg *ptr_start = pnode;
	int ret;

	ret = k_mutex_lock(&mutex_id[index], K_MSEC(1000));
	if (ret) {
		printk("insert_node: mutex get fail status:%x\n", ret);
		return;
	} else {
		// No more room, remove the first node.
		if (seq_current_count[index] == MAX_DATA_QUENE) {
			ipmi_msg_cfg *temp;
			temp = pnode->next;
			if (temp == NULL) {
				printk("insert_node: pnode list is not circular");
				k_mutex_unlock(&mutex_id[index]);
				return;
			}

			pnode->next = temp->next;
			free(temp);
			seq_current_count[index]--;
		}

		while (pnode && pnode->next != ptr_start) {
			pnode = pnode->next;
		}

		if (pnode == NULL) {
			printk("insert_node: pnode list is not circular");
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

// find if any IPMB request record match receiving response
bool find_node(ipmi_msg_cfg *pnode, ipmi_msg *msg, int seq_index, uint8_t index)
{
	ipmi_msg_cfg *ptr_start = pnode;
	int ret;

	ret = k_mutex_lock(&mutex_id[index], K_MSEC(1000));
	if (ret) {
		printk("find_node: mutex get fail status:%x\n", ret);
		return false;
	} else {
		if (seq_index == 0) { // receive response and find sent request
			while ((pnode != NULL && pnode->next != ptr_start) &&
			       (((pnode->next)->buffer.netfn != (msg->netfn - 1)) ||
				((pnode->next)->buffer.cmd != msg->cmd) ||
				((pnode->next)->buffer.seq_target != msg->seq_target))) {
				pnode = pnode->next;
			}
		} else {
			while ((pnode != NULL && pnode->next != ptr_start) &&
			       (((pnode->next)->buffer.netfn != msg->netfn) ||
				((pnode->next)->buffer.cmd != msg->cmd) ||
				((pnode->next)->buffer.seq_source != msg->seq_source))) {
				pnode = pnode->next;
			}
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

		/*get the target<->Bridge IC IPMB seq number*/
		if (seq_index == 0) {
			// find source sequence for responding
			msg->seq_source = temp->buffer.seq_source;
			unregister_seq(index, temp->buffer.seq_target);
		} else {
			msg->seq_target = temp->buffer.seq_target;
		}

		msg->InF_source = temp->buffer.InF_source;
		msg->InF_target = temp->buffer.InF_target;
		/*temp points to the node which has to be removed*/
		pnode->next = temp->next;
		/*We removed the node which is next to the pointer (which is also temp) */

		/* Beacuse we deleted the node, we no longer require the memory used for it .
       free() will deallocate the memory.
     */
		free(temp);
		seq_current_count[index]--;

		k_mutex_unlock(&mutex_id[index]);
		return true;
	}
}

void IPMB_TXTask(void *pvParameters, void *arvg0, void *arvg1)
{
	struct ipmi_msg_cfg *current_msg_tx;
	IPMB_config ipmb_cfg;
	I2C_MSG *msg;
	uint8_t ipmb_buffer_tx[IPMI_MSG_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH], status = 0,
									       retry = 5;
	memcpy(&ipmb_cfg, (IPMB_config *)pvParameters, sizeof(IPMB_config));

	while (1) {
		current_msg_tx = (struct ipmi_msg_cfg *)malloc(sizeof(struct ipmi_msg_cfg));
		if (current_msg_tx == NULL) { // allocate memory fail, retry later
			k_msleep(10);
			continue;
		}

		k_msgq_get(&ipmb_txqueue[ipmb_cfg.index], (ipmi_msg_cfg *)current_msg_tx,
			   K_FOREVER); // Wait for OS queue send interrupt

		if (IS_RESPONSE(current_msg_tx->buffer)) {
			/* We're sending a response */

			/**********************************/
			/*       Error checking           */
			/**********************************/

			/* See if we've already tried sending this message 3 times */
			if (current_msg_tx->retries > IPMB_MAX_RETRIES) {
				/* Free the message buffer */
				printf("IPMB IF %x write reach MAX retry\n",
				       current_msg_tx->buffer.InF_source);
				if (current_msg_tx != NULL) {
					free(current_msg_tx);
				}
				continue;
			}
			/**********************************/
			/*     Try sending the message    */
			/**********************************/
			/* Fix IPMB target address */
			current_msg_tx->buffer.dest_addr = ipmb_cfg.target_addr;
			/* Encode the message buffer to the IPMB format */
			ipmb_encode(&ipmb_buffer_tx[0], &current_msg_tx->buffer);
			uint8_t resp_tx_size =
				current_msg_tx->buffer.data_len + IPMB_RESP_HEADER_LENGTH;

			if (ipmb_cfg.Inf == I2C_IF) {
				msg = malloc(sizeof(I2C_MSG));
				if (msg == NULL) { // allocate fail, retry allocate
					k_msleep(10);
					msg = malloc(sizeof(I2C_MSG));
					if (msg == NULL) {
						printk("IPMB_TXTask: Fail to malloc for i2c resp msg\n");
						if (current_msg_tx != NULL) {
							free(current_msg_tx);
						}
						continue;
					}
				}
				msg->bus = ipmb_cfg.bus;
				msg->slave_addr = ipmb_cfg.target_addr;
				msg->tx_len = resp_tx_size;
				memcpy(&msg->data[0], &ipmb_buffer_tx[1], resp_tx_size);
				status = i2c_master_write(msg, retry);

				if (msg != NULL) {
					free(msg);
				}
			} else { // TODO: else if (ipmb_cfg.Inf == I3C_IF)
				printf("IPMB %d using not support interface: %x\n", ipmb_cfg.index,
				       ipmb_cfg.Inf);
				if (current_msg_tx != NULL) {
					free(current_msg_tx);
				}
				continue;
			}

			if (status) {
				/* Message couldn't be transmitted right now, increase retry counter and try again later */
				current_msg_tx->retries++;
				k_msgq_put(&ipmb_txqueue[ipmb_cfg.index], current_msg_tx,
					   K_NO_WAIT);
				k_msleep(IPMB_RETRY_DELAY_ms);
			} else {
				/* Success case*/
				/* Free the message buffer */
				if (DEBUG_IPMI) {
					printf("ipmb_txqueue[%x] resp netfn: %x, cmd: %x, CC: %x, target_addr: %x\n",
					       ipmb_cfg.index, current_msg_tx->buffer.netfn,
					       current_msg_tx->buffer.cmd,
					       current_msg_tx->buffer.completion_code,
					       ipmb_cfg.target_addr);
					for (int i = 0; i < resp_tx_size + 1; i++) {
						printf(" %x", ipmb_buffer_tx[i]);
					}
					printf("\n");
				}
			}

		} else {
			/***************************************/
			/* Sending new outgoing request        */
			/***************************************/
			ipmb_encode(&ipmb_buffer_tx[0], &current_msg_tx->buffer);

			uint8_t req_tx_size =
				current_msg_tx->buffer.data_len + IPMB_REQ_HEADER_LENGTH;
			if (DEBUG_IPMI) {
				uint8_t i;
				printf("Tx send req: ");
				for (i = 0; i < req_tx_size + 1; i++) {
					printf(" %x", ipmb_buffer_tx[i]);
				}
				printf("\n");
			}

			if (ipmb_cfg.Inf == I2C_IF) {
				msg = malloc(sizeof(I2C_MSG));
				if (msg == NULL) { // allocate fail, retry allocate
					k_msleep(10);
					msg = malloc(sizeof(I2C_MSG));
					if (msg == NULL) {
						printk("IPMB_TXTask: Fail to malloc for i2c req msg\n");
						if (current_msg_tx != NULL) {
							free(current_msg_tx);
						}
						continue;
					}
				}
				msg->bus = ipmb_cfg.bus;
				msg->slave_addr = ipmb_cfg.target_addr;
				msg->tx_len = req_tx_size;
				memcpy(&msg->data[0], &ipmb_buffer_tx[1], req_tx_size);

				status = i2c_master_write(msg, retry);
				if (msg != NULL) {
					free(msg);
				}
			} else { // TODO: else if (ipmb_cfg.Inf == I3C_IF)
				printf("IPMB %d using not support interface: %x\n", ipmb_cfg.index,
				       ipmb_cfg.Inf);
				if (current_msg_tx != NULL) {
					free(current_msg_tx);
				}
				continue;
			}

			if (status) {
				current_msg_tx->retries += 1;

				if (current_msg_tx->retries > IPMB_MAX_RETRIES) {
					/* Return fail status to request source */
					if (current_msg_tx->buffer.InF_source == Reserve_IFs) {
						printf("IPMB_TXTask: Bridging msg from reserve IFs\n");
					} else if (current_msg_tx->buffer.InF_source == Self_IFs) {
						printf("IPMB_TXTask: BIC sending command fail\n"); // Rain - Should record or notice command fail
#ifdef CONFIG_IPMI_KCS_ASPEED
					} else if (current_msg_tx->buffer.InF_source ==
						   HOST_KCS_IFs) {
						uint8_t *kcs_buff;
						kcs_buff = malloc(KCS_buff_size * sizeof(uint8_t));
						if (kcs_buff ==
						    NULL) { // allocate fail, retry allocate
							k_msleep(10);
							kcs_buff = malloc(KCS_buff_size *
									  sizeof(uint8_t));
							if (kcs_buff == NULL) {
								printk("IPMB_TXTask: Fail to malloc for kcs_buff\n");
								free(current_msg_tx);
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
						if (kcs_buff != NULL) {
							free(kcs_buff);
						}
#endif
					} else {
						ipmb_error status;
						current_msg_tx->buffer.data_len = 0;
						current_msg_tx->buffer.netfn = NETFN_OEM_1S_REQ;
						current_msg_tx->buffer.cmd = CMD_OEM_1S_MSG_OUT;
						current_msg_tx->buffer.completion_code =
							CC_NODE_BUSY;
						status = ipmb_send_response(
							&current_msg_tx->buffer,
							IPMB_inf_index_map[current_msg_tx->buffer
										   .InF_source]);

						if (status != ipmb_error_success) {
							printf("IPMB_TXTask: Send IPMB req fail status: %x",
							       status);
						}
					}

					/* Free the message buffer */
					printf("IPMB send fail after retry %d times, source: %x, cmd: 0x%x 0x%x\n",
					       current_msg_tx->retries,
					       current_msg_tx->buffer.InF_source,
					       current_msg_tx->buffer.netfn,
					       current_msg_tx->buffer.cmd);
				} else {
					k_msgq_put(&ipmb_txqueue[ipmb_cfg.index], current_msg_tx,
						   K_NO_WAIT);
					k_msleep(IPMB_RETRY_DELAY_ms);
				}

			} else {
				/* Request was successfully sent, keep a copy here for future comparison and clean the last used buffer */
				// void insert_node(ipmi_msg_cfg *pnode, ipmi_msg *msg)
				current_msg_tx->buffer.seq_target = current_msg_tx->buffer.seq;
				insert_node(P_start[ipmb_cfg.index], &current_msg_tx->buffer,
					    ipmb_cfg.index);
				if (DEBUG_IPMI) {
					printf("Insert node[%x] seq_s: %x, seq_t: %x\n",
					       ipmb_cfg.Inf_source,
					       current_msg_tx->buffer.seq_source,
					       current_msg_tx->buffer.seq_target);
				}
			}
		}
		if (current_msg_tx != NULL) {
			free(current_msg_tx);
		}

		k_msleep(10);
	}
}

void IPMB_RXTask(void *pvParameters, void *arvg0, void *arvg1)
{
	struct ipmi_msg_cfg *current_msg_rx;
	struct IPMB_config ipmb_cfg;
	struct ipmb_msg *msg = NULL;
	uint8_t *ipmb_buffer_rx;
	uint8_t rx_len;
	static uint16_t i = 0;
	int ret;

	memcpy(&ipmb_cfg, (IPMB_config *)pvParameters, sizeof(IPMB_config));

	if (DEBUG_IPMI) {
		printf("Rx poll bus %d, index %d\n", ipmb_cfg.bus, ipmb_cfg.index);
	}

	while (1) {
		k_msleep(IPMB_MQUEUE_POLL_DELAY_ms);
		current_msg_rx = (struct ipmi_msg_cfg *)malloc(sizeof(struct ipmi_msg_cfg));
		if (current_msg_rx == NULL) {
			k_msleep(10); // allocate fail, retry later
			continue;
		}
		ipmb_buffer_rx =
			malloc((IPMI_MSG_MAX_LENGTH + IPMB_RESP_HEADER_LENGTH) * sizeof(uint8_t));
		if (ipmb_buffer_rx == NULL) {
			free(current_msg_rx);
			k_msleep(10); // allocate fail, retry later
			continue;
		}

		rx_len = 0;
		if (ipmb_cfg.Inf == I2C_IF) {
			ret = ipmb_slave_read(dev_ipmb[ipmb_cfg.bus], &msg, &rx_len);
			if (!ret) {
				memcpy(ipmb_buffer_rx, (uint8_t *)msg, rx_len);
				ipmb_buffer_rx[0] = ipmb_buffer_rx[0] >> 1;
			} else {
				free(current_msg_rx);
				free(ipmb_buffer_rx);
				continue;
			}
		} else if (ipmb_cfg.Inf == I3C_IF) {
			;
		} else {
			printf("IPMB_RXTask: Invalid IPMB interface \n");
		}

		if (rx_len > 0) {
			if (DEBUG_IPMI) {
				printf("bus %d recv[%d]", ipmb_cfg.bus, rx_len);
				for (i = 0; i < rx_len; i++) {
					printf(" %x", ipmb_buffer_rx[i + 1]);
				}
				printf("\n");
			}

			/* Perform a checksum test on the message, if it doesn't pass, just ignore it.
  		 * Following the IPMB specs, we have no way to know if we're the one who should
  		 * receive it. In MicroTCA crates with star topology for IPMB, we are assured we
  		 * are the recipients, however, malformed messages may be safely ignored as the
  		 * MCMC should take care of retrying.
  		 */
			if (ipmb_assert_chksum(ipmb_buffer_rx, rx_len) != ipmb_error_success) {
				printf("Recv invalid chksum from index %d\n", ipmb_cfg.index);
				if (current_msg_rx != NULL) {
					free(current_msg_rx);
				}
				if (ipmb_buffer_rx != NULL) {
					free(ipmb_buffer_rx);
				}
				continue;
			}

			/* Clear our local buffer before writing new data into it */
			ipmb_error IPMB_ERR_STATUS;
			IPMB_ERR_STATUS =
				ipmb_decode(&(current_msg_rx->buffer), ipmb_buffer_rx, rx_len);

			if (ipmb_buffer_rx != NULL) {
				free(ipmb_buffer_rx);
			}

			if (IPMB_ERR_STATUS != ipmb_error_success) {
				printf("IPMB_ERR_STATUS: %x\n", IPMB_ERR_STATUS);
			}

			if (DEBUG_IPMI) {
				printf("buff[%d]", current_msg_rx->buffer.data_len);
				for (i = 0; i < current_msg_rx->buffer.data_len; i++) {
					printf(" %x", current_msg_rx->buffer.data[i]);
				}
				printf("\n");
			}

			if (IS_RESPONSE(current_msg_rx->buffer)) {
				/* The message is a response, check if it's request from BIC and respond in time */
				current_msg_rx->buffer.seq_target = current_msg_rx->buffer.seq;
				if (find_node(P_start[ipmb_cfg.index], &(current_msg_rx->buffer), 0,
					      ipmb_cfg.index)) {
					if (DEBUG_IPMI) {
						printf("find node IFs:%x, IFt:%x\n",
						       current_msg_rx->buffer.InF_source,
						       current_msg_rx->buffer.InF_target);
						printf("find buff[%d] seq %x:",
						       current_msg_rx->buffer.data_len,
						       current_msg_rx->buffer.seq_target);
						for (i = 0; i < current_msg_rx->buffer.data_len;
						     i++) {
							printf(" %x",
							       current_msg_rx->buffer.data[i]);
						}
						printf("\n");
					}

					if (current_msg_rx->buffer.InF_source ==
					    SELF_IPMB_IF) { // Send from other thread
						struct ipmi_msg current_msg;
						current_msg =
							(struct ipmi_msg)current_msg_rx->buffer;
						k_msgq_put(
							&ipmb_rxqueue[IPMB_inf_index_map
									      [current_msg_rx->buffer
										       .InF_target]],
							&current_msg, K_NO_WAIT);
#ifdef CONFIG_IPMI_KCS_ASPEED
					} else if (current_msg_rx->buffer.InF_source ==
						   HOST_KCS_IFs) {
						uint8_t *kcs_buff;
						kcs_buff = malloc(KCS_buff_size * sizeof(uint8_t));
						if (kcs_buff ==
						    NULL) { // allocate fail, retry allocate
							k_msleep(10);
							kcs_buff = malloc(KCS_buff_size *
									  sizeof(uint8_t));
							if (kcs_buff == NULL) {
								printk("IPMB_RXTask: Fail to malloc for kcs_buff\n");
								free(current_msg_rx);
								continue;
							}
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
						if (kcs_buff != NULL) {
							free(kcs_buff);
						}
#endif
					} else if (current_msg_rx->buffer.InF_source ==
						   ME_IPMB_IFs) {
						ipmb_error status;
						ipmi_msg *bridge_msg =
							(ipmi_msg *)malloc(sizeof(ipmi_msg));
						memset(bridge_msg, 0, sizeof(ipmi_msg));

						bridge_msg->netfn =
							current_msg_rx->buffer.netfn -
							1; // fix response netfn and would be shift later in ipmb_send_response
						bridge_msg->cmd = current_msg_rx->buffer.cmd;
						bridge_msg->completion_code =
							current_msg_rx->buffer.completion_code;
						bridge_msg->seq = current_msg_rx->buffer.seq_source;
						bridge_msg->data_len =
							current_msg_rx->buffer.data_len;
						memcpy(&bridge_msg->data[0],
						       &current_msg_rx->buffer.data[0],
						       current_msg_rx->buffer.data_len);

						if (DEBUG_IPMI) {
							printf("ME bridge[%d] seq_s %x, seq_t %x, Inf_source %x, Inf_source_index %x :\n",
							       bridge_msg->data_len,
							       current_msg_rx->buffer.seq_source,
							       current_msg_rx->buffer.seq_target,
							       current_msg_rx->buffer.InF_source,
							       IPMB_inf_index_map
								       [current_msg_rx->buffer
										.InF_source]);
							for (i = 0; i < bridge_msg->data_len; i++) {
								printf(" %x", bridge_msg->data[i]);
							}
							printf("\n");
						}

						status = ipmb_send_response(
							bridge_msg,
							IPMB_inf_index_map[current_msg_rx->buffer
										   .InF_source]);
						if (status != ipmb_error_success) {
							printf("IPMB_RXTask: Send IPMB resp fail status: %x",
							       status);
						}

						if (bridge_msg != NULL) {
							free(bridge_msg);
						}
					} else { // Bridge response to other fru
						ipmb_error status;
						ipmi_msg *bridge_msg =
							(ipmi_msg *)malloc(sizeof(ipmi_msg));
						memset(bridge_msg, 0, sizeof(ipmi_msg));

						bridge_msg->data[0] =
							WW_IANA_ID &
							0xFF; // Move target response to bridge response data
						bridge_msg->data[1] = (WW_IANA_ID >> 8) & 0xFF;
						bridge_msg->data[2] = (WW_IANA_ID >> 16) & 0xFF;
						bridge_msg->data[3] =
							IPMB_config_table[ipmb_cfg.index]
								.Inf_source; // return response source as request target
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

						if (DEBUG_IPMI) {
							printf("bridge[%d] seq_s %x, seq_t %x, Inf_source %x, Inf_source_index %x :\n",
							       bridge_msg->data_len,
							       current_msg_rx->buffer.seq_source,
							       current_msg_rx->buffer.seq_target,
							       current_msg_rx->buffer.InF_source,
							       IPMB_inf_index_map
								       [current_msg_rx->buffer
										.InF_source]);
							for (i = 0; i < bridge_msg->data_len; i++) {
								printf(" %x", bridge_msg->data[i]);
							}
							printf("\n");
						}

						status = ipmb_send_response(
							bridge_msg,
							IPMB_inf_index_map[current_msg_rx->buffer
										   .InF_source]);
						if (status != ipmb_error_success) {
							printf("IPMB_RXTask: Send IPMB resp fail status: %x",
							       status);
						}

						if (bridge_msg != NULL) {
							free(bridge_msg);
						}
					}
				}

			} else {
				// Exception: ME sends standard IPMI commands to BMC but not BIC.
				//            So BIC should bridge ME request to BMC directly, instead of
				//            executing IPMI handler.
				if (IPMB_config_table[ipmb_cfg.index].Inf_source == ME_IPMB_IFs &&
				    (!pal_ME_is_to_ipmi_handler(current_msg_rx->buffer.netfn,
								current_msg_rx->buffer.cmd))) {
					ipmb_error status;
					ipmi_msg *bridge_msg = (ipmi_msg *)malloc(sizeof(ipmi_msg));
					memset(bridge_msg, 0, sizeof(ipmi_msg));

					bridge_msg->data_len = current_msg_rx->buffer.data_len;
					bridge_msg->seq_source = current_msg_rx->buffer.seq;
					bridge_msg->InF_target = BMC_IPMB_IFs;
					bridge_msg->InF_source = ME_IPMB_IFs;
					bridge_msg->netfn = current_msg_rx->buffer.netfn;
					bridge_msg->cmd = current_msg_rx->buffer.cmd;
					if (bridge_msg->data_len != 0) {
						memcpy(&bridge_msg->data[0],
						       &current_msg_rx->buffer.data[0],
						       current_msg_rx->buffer.data_len);
					}

					status = ipmb_send_request(
						bridge_msg, IPMB_inf_index_map[BMC_IPMB_IFs]);
					if (status != ipmb_error_success) {
						printf("OEM_MSG_OUT send IPMB req fail status: %x",
						       status);
						bridge_msg->completion_code = CC_TIMEOUT;
						status = ipmb_send_response(
							bridge_msg,
							IPMB_inf_index_map[bridge_msg->InF_source]);
						if (status != ipmb_error_success) {
							printf("IPMI_handler send IPMB resp fail status: %x",
							       status);
						}
					}
					if (bridge_msg != NULL) {
						free(bridge_msg);
					}
				} else {
					/* The received message is a request */
					/* Record sequence number for later response */
					current_msg_rx->buffer.seq_source =
						current_msg_rx->buffer.seq;
					/* Record source interface for later bridge response */
					current_msg_rx->buffer.InF_source =
						IPMB_config_table[ipmb_cfg.index].Inf_source;
					/* Notify the client about the new request */
					if (DEBUG_IPMI) {
						printf("recv req: data: %x, InfS: %x, seq_s: %x\n",
						       current_msg_rx->buffer.netfn,
						       current_msg_rx->buffer.InF_source,
						       current_msg_rx->buffer.seq_source);
					}
					ipmb_notify_client(current_msg_rx);
				}
			}
		}
		free(current_msg_rx);
	}
}

ipmb_error ipmb_send_request(ipmi_msg *req, uint8_t index)
{
	int ret;
	ret = k_mutex_lock(&mutex_send_req, K_MSEC(1000));
	if (ret) {
		return ipmb_error_mutex_lock;
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
	req_cfg.buffer.dest_addr = IPMB_config_table[index].target_addr;
	req_cfg.buffer.dest_LUN = 0;
	req_cfg.buffer.src_addr = IPMB_config_table[index].slave_addr << 1;
	req_cfg.buffer.seq = get_free_seq(index);
	req_cfg.buffer.seq_source = req->seq_source;
	req_cfg.buffer.src_LUN = 0;
	req_cfg.retries = 0;
	/* Blocks here until is able put message in tx queue */
	if (k_msgq_put(&ipmb_txqueue[index], &req_cfg, K_FOREVER) != osOK) {
		k_mutex_unlock(&mutex_send_req);
		return ipmb_error_failure;
	}
	k_mutex_unlock(&mutex_send_req);
	return ipmi_error_success;
}

ipmb_error ipmb_send_response(ipmi_msg *resp, uint8_t index)
{
	int ret;
	ret = k_mutex_lock(&mutex_send_res, K_MSEC(1000));
	if (ret) {
		return ipmb_error_mutex_lock;
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
	resp_cfg.buffer.dest_addr = IPMB_config_table[index].target_addr;
	resp_cfg.buffer.netfn = resp->netfn + 1;
	resp_cfg.buffer.dest_LUN = resp->src_LUN;
	resp_cfg.buffer.src_addr = IPMB_config_table[index].slave_addr << 1;
	resp_cfg.buffer.seq = resp->seq;
	resp_cfg.buffer.seq_source = resp_cfg.buffer.seq;
	resp_cfg.buffer.InF_source = resp->InF_source;
	resp_cfg.buffer.src_LUN = resp->dest_LUN;
	resp_cfg.buffer.cmd = resp->cmd;
	resp_cfg.retries = 0;

	if (DEBUG_IPMI) {
		uint8_t i;
		printf("send resp[%d] index %x cc %x :", resp_cfg.buffer.data_len, index,
		       resp_cfg.buffer.completion_code);
		for (i = 0; i < resp_cfg.buffer.data_len; i++) {
			printf(" %x", resp_cfg.buffer.data[i]);
		}
		printf("\n");
	}

	/* Blocks here until is able put message in tx queue */
	if (k_msgq_put(&ipmb_txqueue[index], &resp_cfg, K_FOREVER) != osOK) {
		k_mutex_unlock(&mutex_send_res);
		return ipmb_error_failure;
	}
	k_mutex_unlock(&mutex_send_res);
	return ipmi_error_success;
}

ipmb_error ipmb_read(ipmi_msg *msg, uint8_t index)
{
	ipmb_error status;
	// Set mutex timeout 10ms more than messageQueue timeout, prevent mutex timeout before messageQueue
	k_mutex_lock(&mutex_read, K_MSEC(IPMB_SEQ_TIMEOUT_ms + 10));
	// Reset a Message Queue to initial empty state
	k_msgq_purge(&ipmb_rxqueue[index]);

	status = ipmb_send_request(msg, index);
	if (status != ipmb_error_success) {
		printf("ipmb send request fail status: %x", status);
		k_mutex_unlock(&mutex_read);
		return ipmb_error_failure;
	}

	if (k_msgq_get(&ipmb_rxqueue[index], (ipmi_msg *)msg, K_MSEC(IPMB_SEQ_TIMEOUT_ms))) {
		k_mutex_unlock(&mutex_read);
		return ipmb_error_get_messageQueue;
	}

	k_mutex_unlock(&mutex_read);
	return ipmi_error_success;
}

// run IPMI handler and notify command process status
ipmb_error ipmb_notify_client(ipmi_msg_cfg *msg_cfg)
{
	/* Sends only the ipmi msg, not the control struct */
	if (!IS_RESPONSE(msg_cfg->buffer)) {
		while (k_msgq_put(&ipmi_msgq, msg_cfg, K_NO_WAIT) != 0) {
			k_msgq_purge(&ipmi_msgq);
			printf("Retrying put ipmi msgq\n");
		}
	}
	return ipmb_error_success;
}

ipmb_error ipmb_assert_chksum(uint8_t *buffer, uint8_t buffer_len)
{
	uint8_t header_chksum = buffer[2];
	uint8_t msg_chksum = buffer[buffer_len - 1];
	uint8_t calc_header_chksum = calculate_chksum(buffer, IPMI_HEADER_CHECKSUM_POSITION);
	uint8_t calc_msg_chksum = calculate_chksum(buffer, buffer_len - 1);

	if (header_chksum == calc_header_chksum) {
		if (msg_chksum == calc_msg_chksum) {
			return ipmb_error_success;
		}

		return ipmb_error_hdr_chksum;
	}
	return ipmb_error_msg_chksum;
}

ipmb_error ipmb_encode(uint8_t *buffer, ipmi_msg *msg)
{
	uint8_t i = 0;

	/* Use this variable to address the buffer dynamically */

	buffer[i++] = msg->dest_addr;
	buffer[i++] =
		(((msg->netfn << 2) & IPMB_NETFN_MASK) | (msg->dest_LUN & IPMB_DEST_LUN_MASK));
	buffer[i++] = calculate_chksum(&buffer[0], IPMI_HEADER_CHECKSUM_POSITION);
	buffer[i++] = msg->src_addr;
	buffer[i++] = (((msg->seq << 2) & IPMB_SEQ_MASK) | (msg->src_LUN & IPMB_SRC_LUN_MASK));
	buffer[i++] = msg->cmd;
	if (IS_RESPONSE((*msg))) {
		buffer[i++] = msg->completion_code;
	}
	memcpy(&buffer[i], &msg->data[0], msg->data_len);
	i += msg->data_len;

	buffer[i] = calculate_chksum(&buffer[0], i);
	return ipmb_error_success;
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
	/* Checks if the message is a response and if so, fills the completion code field */
	if (IS_RESPONSE((*msg))) {
		msg->completion_code = buffer[i++];
	}
	msg->data_len = (len > i + 1) ? len - i - 1 : 0;
	memcpy(&msg->data[0], &buffer[i], msg->data_len);
	msg->msg_chksum = buffer[len - 1];

	return ipmb_error_success;
}

void IPMB_SeqTimeout_handler(void *arug0, void *arug1, void *arug2)
{
	ipmi_msg_cfg *pnode;
	int ret;
	uint8_t index;
	uint32_t current_time;

	while (1) {
		// k_msleep(IPMB_SEQ_TIMEOUT_ms);
		k_msleep(IPMB_SEQ_TIMEOUT_ms);
		for (index = 0; index < MAX_IPMB_IDX; index++) {
			if (!IPMB_config_table[index].EnStatus) {
				continue;
			}

			ret = k_mutex_lock(&mutex_id[index], K_MSEC(1000));

			if (ret) {
				printk("IPMB_SeqTimeout_handler: mutex get fail status:%x\n", ret);
			} else {
				pnode = P_start[index];
				while (pnode->next != P_start[index]) {
					current_time = osKernelGetSysTimerCount();
					// The queue stay more than 1000 ms need to be killed
					if (current_time + tick_fix -
							    ((pnode->next)->buffer.timestamp) >
						    IPMB_timeout_S * sys_tick_freq ||
					    (current_time + tick_fix -
					     ((pnode->next)->buffer.timestamp)) &
						    0x80000000) {
						ipmi_msg_cfg *temp;
						temp = pnode->next;
						pnode->next = temp->next;
						unregister_seq(index, temp->buffer.seq_target);
						if (temp != NULL) {
							free(temp);
						}
						seq_current_count[index]--;
					}

					pnode = pnode->next;
				}

				k_mutex_unlock(&mutex_id[index]);
			}
		}
	}
}

static void init_ipmb_slave_dev(void)
{
#ifdef DEV_IPMB_0
	dev_ipmb[0] = device_get_binding("IPMB_0");
	if (i2c_slave_driver_register(dev_ipmb[0]))
		printk("IPMB0: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_1
	dev_ipmb[1] = device_get_binding("IPMB_1");
	if (i2c_slave_driver_register(dev_ipmb[1]))
		printk("IPMB: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_2
	dev_ipmb[2] = device_get_binding("IPMB_2");
	if (i2c_slave_driver_register(dev_ipmb[2]))
		printk("IPMB2: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_3
	dev_ipmb[3] = device_get_binding("IPMB_3");
	if (i2c_slave_driver_register(dev_ipmb[3]))
		printk("IPMB3: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_4
	dev_ipmb[4] = device_get_binding("IPMB_4");
	if (i2c_slave_driver_register(dev_ipmb[4]))
		printk("IPMB4: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_5
	dev_ipmb[5] = device_get_binding("IPMB_5");
	if (i2c_slave_driver_register(dev_ipmb[5]))
		printk("IPMB5: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_6
	dev_ipmb[6] = device_get_binding("IPMB_6");
	if (i2c_slave_driver_register(dev_ipmb[6]))
		printk("IPMB6: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_7
	dev_ipmb[7] = device_get_binding("IPMB_7");
	if (i2c_slave_driver_register(dev_ipmb[7]))
		printk("IPMB7: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_8
	dev_ipmb[8] = device_get_binding("IPMB_8");
	if (i2c_slave_driver_register(dev_ipmb[8]))
		printk("IPMB8: Slave Device driver not found.");
#endif
#ifdef DEV_IPMB_9
	dev_ipmb[9] = device_get_binding("IPMB_9");
	if (i2c_slave_driver_register(dev_ipmb[9]))
		printk("IPMB9: Slave Device driver not found.");
#endif
}

void ipmb_util_init(uint8_t index)
{
	osThreadAttr_t IPMB_TxTask_attr;
	osThreadAttr_t IPMB_RxTask_attr;

	memset(&IPMB_TxTask_attr, 0, sizeof(IPMB_TxTask_attr));
	memset(&IPMB_RxTask_attr, 0, sizeof(IPMB_RxTask_attr));
	memset(&seq_table[index], 0, sizeof(bool) * SEQ_NUM);

	P_start[index] = (void *)malloc(sizeof(struct ipmi_msg_cfg));
	P_temp = P_start[index];
	P_temp->next = P_temp;

	if (k_mutex_init(&mutex_id[index])) {
		printf("IPMB mutex %d init fail\n", index);
	}

	k_msgq_init(&ipmb_txqueue[index], ipmb_txqueue_buffer[index], sizeof(struct ipmi_msg_cfg),
		    IPMB_TXQUEUE_LEN);
	k_msgq_init(&ipmb_rxqueue[index], ipmb_rxqueue_buffer[index], sizeof(struct ipmi_msg),
		    IPMB_RXQUEUE_LEN);

	IPMB_TX_ID[index] =
		k_thread_create(&IPMB_TX[index], ipmb_tx_stacks[index], IPMB_TX_STACK_SIZE,
				IPMB_TXTask, (void *)&IPMB_config_table[index], NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMB_TX[index], IPMB_config_table[index].Tx_attr_name);
	IPMB_RX_ID[index] =
		k_thread_create(&IPMB_RX[index], ipmb_rx_stacks[index], IPMB_RX_STACK_SIZE,
				IPMB_RXTask, (void *)&IPMB_config_table[index], NULL, NULL,
				CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMB_RX[index], IPMB_config_table[index].Rx_attr_name);

	if (DEBUG_IPMI) {
		printf("Init IPMB bus %d, addr %x\n", IPMB_config_table[index].bus,
		       IPMB_config_table[index].slave_addr);
	}

	return;
}

void ipmb_init(void)
{
	uint8_t index;
	init_ipmb_slave_dev();

	IPMB_config_table = malloc(MAX_IPMB_IDX * sizeof(IPMB_config));
	if (IPMB_config_table != NULL) {
		pal_load_IPMB_config();
	} else {
		printf("IPMB_config_table alloc fail\n");
		return;
	}

	map_inf_index();

	memset(&current_seq, 0, sizeof(uint8_t) * MAX_IPMB_IDX);

	sys_tick_freq = osKernelGetSysTimerFreq();
	tick_fix = sys_tick_freq / 1000;

	if (k_mutex_init(&mutex_send_req)) {
		printf("IPMB send req mutex init fail\n");
	}
	if (k_mutex_init(&mutex_send_res)) {
		printf("IPMB send seq mutex init fail\n");
	}
	if (k_mutex_init(&mutex_read)) {
		printf("IPMB read mutex init fail\n");
	}

	for (index = 0; index < MAX_IPMB_IDX; index++) {
		if (IPMB_config_table[index].EnStatus) {
			ipmb_util_init(index);
		}
	}

	k_thread_create(&IPMB_SeqTimeout, IPMB_SeqTimeout_stack,
			K_THREAD_STACK_SIZEOF(IPMB_SeqTimeout_stack), IPMB_SeqTimeout_handler, NULL,
			NULL, NULL, CONFIG_MAIN_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&IPMB_SeqTimeout, "IPMB_SeqTimeout");
}
