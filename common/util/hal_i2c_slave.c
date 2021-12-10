/*
  NAME: I2C SLAVE DEVICE
  FILE: hal_i2c_slave.c
  DESCRIPTION: There is 1 callback function "i2c_slave_cb" for I2C slave ISR handle and user APIs for user access.
  AUTHOR: MouchenHung
  DATE/VERSION: 2021.12.09 - v1.4.2
  Note: 
    (1) Shall not modify code in this file!!!

    (2) "hal_i2c_slave.h" must be included!

    (3) User APIs follow check-rule before doing task 
          [api]                               [.is_init] [.is_register]
        * i2c_slave_control                   X          X
        * i2c_slave_read                      O          X
        * i2c_slave_status_get                X          X
        * i2c_slave_status_print              X          X
        * i2c_slave_cfg_get                   O          X
                                              (O: must equal 1, X: no need to check)

    (4) I2C slave function/api usage recommend
        [ACTIVATE]
          Use "i2c_slave_control()" to register/modify/unregister slave bus
        [READ]
          Use "i2c_slave_read()" to read slave queue message

    (5) Slave queue method: Zephyr api, unregister the bus while full msgq, register back while msgq get space.
*/

#include <zephyr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <drivers/i2c.h>
#include <sys/printk.h>
#include "hal_i2c_slave.h"

/* LOG SET */
#include <logging/log.h>
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
LOG_MODULE_REGISTER(mc_i2c_slave);

/* I2C slave device arr */
static struct i2c_slave_device i2c_slave_device_global[MAX_SLAVE_NUM] = { 0 };

/* I2C slave config modify lock */
struct k_mutex i2c_slave_mutex[MAX_SLAVE_NUM];

/* static function declare */
static int do_i2c_slave_cfg(uint8_t bus_num, struct _i2c_slave_config *cfg);
static int do_i2c_slave_register(uint8_t bus_num);
static int do_i2c_slave_unregister(uint8_t bus_num);

static int i2c_slave_write_requested(struct i2c_slave_config *config)
{
  struct i2c_slave_data *data;

  if (!config){
    LOG_ERR("Get empty config!");
    return 1;
  }

  data = CONTAINER_OF(config, struct i2c_slave_data, config);

  data->current_msg.msg_length = 0;
  memset(data->current_msg.msg, 0x0, MAX_I2C_SLAVE_BUFF);
  data->buffer_idx = 0;

  return 0;
}

static int i2c_slave_write_received(struct i2c_slave_config *config, uint8_t val)
{
  struct i2c_slave_data *data;
  
  if (!config){
    LOG_ERR("Get empty config!");
    return 1;
  }

  data = CONTAINER_OF(config, struct i2c_slave_data, config);

  if (data->buffer_idx >= MAX_I2C_SLAVE_BUFF){
    LOG_ERR("Buffer_idx over limit!");
    return 1;
  }
  data->current_msg.msg[data->buffer_idx++] = val;

  return 0;
}

static int i2c_slave_stop(struct i2c_slave_config *config)
{
  struct i2c_slave_data *data;
  
  if (!config){
    LOG_ERR("Get empty config!");
    return 1;
  }

  data = CONTAINER_OF(config, struct i2c_slave_data, config);

  if (data->buffer_idx){
    data->current_msg.msg_length = data->buffer_idx;

#if DEBUG_MODE
    printk("recv: ");
    for (int i=0; i<data->current_msg.msg_length; i++){
      printk("[%d]", data->current_msg.msg[i]);
    }
    printk("\n");
#endif

    /* try to put new node to message queue */
    uint8_t ret = k_msgq_put(&data->z_msgq_id, &data->current_msg, K_NO_WAIT);
    if (ret){
      LOG_ERR("Can't put new node to message queue on bus[%d], cause of %d", data->i2c_bus, ret);
      return 1;
    }

    /* if slave queue is full, unregister the bus slave to prevent next message handle */
    if (!k_msgq_num_free_get(&data->z_msgq_id)){
      LOG_DBG("Slave queue is full, unregister bus[%d]", data->i2c_bus);
      do_i2c_slave_unregister(data->i2c_bus);
    }
  }

  return 0;
}

static const struct i2c_slave_callbacks i2c_slave_cb = {
  .write_requested = i2c_slave_write_requested,
  .read_requested = NULL,
  .write_received = i2c_slave_write_received,
  .read_processed = NULL,
  .stop = i2c_slave_stop,
};

/*
  - Name: i2c_slave_status_get (ESSENTIAL for every user API)
  - Description: Get current status of i2c slave.
  - Input:
      * bus_num: Bus number with zero base
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_error_status")
*/
uint8_t i2c_slave_status_get(uint8_t bus_num)
{
  uint8_t ret = I2C_SLAVE_HAS_NO_ERR;

  if (bus_num >= MAX_SLAVE_NUM){
    ret|=I2C_SLAVE_BUS_INVALID;
    goto out;
  }

  char controllername[10];
  if (snprintf(controllername, sizeof(controllername), "%s%d", I2C_DEVICE_PREFIX, bus_num) < 0){
    LOG_ERR("I2C controller name parsing error!");
    ret = I2C_SLAVE_CONTROLLER_ERR;
    goto out;
  }

  const struct device *tmp_device = device_get_binding(controllername);
  if (!tmp_device){
    ret|=I2C_SLAVE_CONTROLLER_ERR;
    goto out;
  }

  if (!i2c_slave_device_global[bus_num].is_init){
    ret|=I2C_SLAVE_NOT_INIT;
  }

  if (!i2c_slave_device_global[bus_num].is_register){
    ret|=I2C_SLAVE_NOT_REGISTER;
  }

out:
  return ret;
}

/*
  - Name: i2c_slave_cfg_get (OPTIONAL)
  - Description: Get current cfg of i2c slave.
  - Input:
      * bus_num: Bus number with zero base
      * cfg: cfg structure(controller name is not support!)
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
uint8_t i2c_slave_cfg_get(uint8_t bus_num, struct _i2c_slave_config *cfg){
  uint8_t status;

  if (!cfg)
    return I2C_SLAVE_API_INPUT_ERR;

  /* check input */
  status = i2c_slave_status_get(bus_num);
  if ( status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR | I2C_SLAVE_NOT_INIT)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  struct i2c_slave_data *data = &i2c_slave_device_global[bus_num].data;

  cfg->address = data->config.address;
  cfg->i2c_msg_count = data->z_msgq_id.max_msgs;

  return I2C_SLAVE_API_NO_ERR;
}

/*
  - Name: i2c_slave_status_print (OPTIONAL|DEBUGUSE)
  - Description: Get current status of i2c slave queue.
  - Input:
      * bus_num: Bus number with zero base
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
uint8_t i2c_slave_status_print(uint8_t bus_num)
{
  uint8_t status;

  /* check input */
  status = i2c_slave_status_get(bus_num);
  if ( status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  struct i2c_slave_data *data = &i2c_slave_device_global[bus_num].data;
  struct i2c_slave_device *slave_info = &i2c_slave_device_global[bus_num];
  printk("=============================\n");
  printk("Slave bus[%d] monitor\n", bus_num);
  printk("* init:        %d\n", slave_info->is_init);
  printk("* register:    %d\n", slave_info->is_register);
  printk("* address:     0x%x\n", data->config.address);
  printk("* status:      %d/%d\n", k_msgq_num_used_get(&data->z_msgq_id), data->z_msgq_id.max_msgs);
  printk("=============================\n");

  return I2C_SLAVE_API_NO_ERR;
}

/*
  - Name: i2c_slave_read
  - Description: Try to get message from i2c slave message queue.
  - Input:
      * bus_num: Bus number with zero base
      * *buff: Message that readed back from queue
      * buff_len: Length of buffer
      * *msg_len: Read-back message's length
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
uint8_t i2c_slave_read(uint8_t bus_num, uint8_t *buff, uint16_t buff_len, uint16_t *msg_len)
{
  uint8_t status;

  if (!buff || !msg_len)
    return I2C_SLAVE_API_INPUT_ERR;

  /* check input, support while bus slave is unregistered */
  status = i2c_slave_status_get(bus_num);
  if (status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR | I2C_SLAVE_NOT_INIT)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  struct i2c_slave_data *data = &i2c_slave_device_global[bus_num].data;
  struct i2c_msg_package local_buf;

  /* wait if there's no any message in message queue */
  uint8_t ret = k_msgq_get(&data->z_msgq_id, &local_buf, K_FOREVER);
  if (ret){
    LOG_ERR("Can't get new node from message queue on bus[%d], cause of %d", data->i2c_bus, ret);
    return I2C_SLAVE_API_MSGQ_ERR;
  }

  if (buff_len < local_buf.msg_length){
    memcpy(buff, &(local_buf.msg), buff_len);
    *msg_len = buff_len;
  }
  else{
    memcpy(buff, &(local_buf.msg), local_buf.msg_length);
    *msg_len = local_buf.msg_length;
  }
  
  /* if bus slave has been unregister cause of queue full previously, then register it on */
  if (k_msgq_num_used_get(&data->z_msgq_id) == (data->z_msgq_id.max_msgs - 1)){
    LOG_DBG("Slave queue has available space, register bus[%d]", data->i2c_bus);

    if (do_i2c_slave_register(bus_num)){
      LOG_ERR("Slave queue register bus[%d] failed!", data->i2c_bus);
      return I2C_SLAVE_API_BUS_GET_FAIL;
    }
  }
  
  return I2C_SLAVE_API_NO_ERR;
}

/*
  - Name: i2c_slave_control
  - Description: Register controller for user api.
  - Input:
      * bus_num: Bus number with zero base
      * *cfg: Config settings structure
      * mode: check "i2c_slave_api_control_mode"
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
int i2c_slave_control(uint8_t bus_num, struct _i2c_slave_config *cfg, enum i2c_slave_api_control_mode mode)
{
  int status;

  /* Check input and slave status */
  uint8_t slave_status = i2c_slave_status_get(bus_num);
  if (slave_status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, slave_status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  switch (mode)
  {
    /* Case1: do config then register (if already config before, then modify config set) */
    case I2C_CONTROL_REGISTER:
      if (!cfg){
        return I2C_SLAVE_API_INPUT_ERR;
      }

      status = do_i2c_slave_cfg(bus_num ,cfg);
      if (status){
        LOG_ERR("Bus[%d] config failed with errorcode %d!", bus_num, status);
        return status;
      }

      status = do_i2c_slave_register(bus_num);
      if (status){
        LOG_ERR("Bus[%d] register failed with errorcode %d!", bus_num, status);
        return status;
      }

      break;

    /* Case2: do unregister only, config not affected */
    case I2C_CONTROL_UNREGISTER:
      status = do_i2c_slave_unregister(bus_num);
      if (status){
        LOG_ERR("Bus[%d] unregister failed with errorcode %d!", bus_num, status);
        return status;
      }

      break;
  
    default:
      return I2C_SLAVE_API_INPUT_ERR;
  }

  return I2C_SLAVE_API_NO_ERR;
}

/*
  - Name: do_i2c_slave_cfg
  - Description: To initialize I2C slave config, or modify config after initialized.
  - Input:
      * bus_num: Bus number with zero base
      * *bus_name: Bus controler name with string(it's used to get binding from certain zephyr device tree driver)
      * slave_address: Given a slave adress for BIC itself
      * _max_msg_count: Maximum count of messages in message queue
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
static int do_i2c_slave_cfg(uint8_t bus_num, struct _i2c_slave_config *cfg){
  if (!cfg)
    return I2C_SLAVE_API_INPUT_ERR;

  int status;
  uint8_t slave_status = I2C_SLAVE_HAS_NO_ERR;
  int ret = I2C_SLAVE_API_NO_ERR;

  /* check input, support while bus slave is unregistered */
  slave_status = i2c_slave_status_get(bus_num);
  if (slave_status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, slave_status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  /* need unregister first */
  if ( !(slave_status & I2C_SLAVE_NOT_REGISTER) ){
    status = do_i2c_slave_unregister(bus_num);
    if (status){
      LOG_ERR("Slave bus[%d] mutex lock failed!", bus_num);
      return I2C_SLAVE_API_BUS_GET_FAIL;
    }
  }

  /* Mutex init here */
  if (slave_status & I2C_SLAVE_NOT_INIT){
    if (k_mutex_init(&i2c_slave_mutex[bus_num])){
      LOG_ERR("Slave bus[%d] mutex init - failed!", bus_num);
      return I2C_SLAVE_API_LOCK_ERR;
    }
    LOG_DBG("Slave bus[%d] mutex init - success!", bus_num);
  }

  if (k_mutex_lock(&i2c_slave_mutex[bus_num], K_MSEC(1000))) {
    LOG_ERR("Slave bus[%d] mutex lock failed!", bus_num);
    return I2C_SLAVE_API_LOCK_ERR;
  }

  uint8_t slave_address = cfg->address;
  uint16_t _max_msg_count = cfg->i2c_msg_count;

  struct i2c_slave_data *data = &i2c_slave_device_global[bus_num].data;
  char *i2C_slave_queue_buffer;

  /* do init, Only one time init for each bus slave */
  if ( slave_status & I2C_SLAVE_NOT_INIT ){
    LOG_DBG("Bus[%d] is going to init!", bus_num);
    
    data->i2c_bus = bus_num;

    char controllername[10];
    if (snprintf(controllername, sizeof(controllername), "%s%d", I2C_DEVICE_PREFIX, bus_num) < 0){
      LOG_ERR("I2C controller name parsing error!");
      ret = I2C_SLAVE_API_MEMORY_ERR;
      goto unlock;
    }

    data->i2c_controller = device_get_binding(controllername);
    if (!data->i2c_controller){
      LOG_ERR("I2C controller not found!");
      ret = -EINVAL;
      goto unlock;
    }
    
    data->config.callbacks = &i2c_slave_cb;
  }
  /* do modify, modify config set after init */
  else{
    LOG_DBG("Bus[%d] is going to modified!", bus_num);

    k_msgq_purge(&data->z_msgq_id);
    
    if (data->z_msgq_id.buffer_start != NULL){
      free(data->z_msgq_id.buffer_start);
      data->z_msgq_id.buffer_start = NULL;
    }
  }

  data->max_msg_count = _max_msg_count;
  data->config.address = slave_address >> 1; // to 7-bit slave address
  
  i2C_slave_queue_buffer = malloc(data->max_msg_count * sizeof(struct i2c_msg_package));
  if (!i2C_slave_queue_buffer){
    LOG_ERR("I2C slave bus[%d] msg queue memory allocate failed!", data->i2c_bus);
    ret = I2C_SLAVE_API_MEMORY_ERR;
    goto unlock;
  }

  k_msgq_init(&data->z_msgq_id, i2C_slave_queue_buffer, sizeof(struct i2c_msg_package), data->max_msg_count);

  i2c_slave_device_global[bus_num].is_init = 1;

  LOG_DBG("I2C slave bus[%d] message queue create success with count %d!", data->i2c_bus, data->max_msg_count);

unlock:
  if (k_mutex_unlock(&i2c_slave_mutex[bus_num])) {
    LOG_ERR("Mutex unlock failed!");
  }

  return ret;
}

/*
  - Name: do_i2c_slave_register
  - Description: Set config to register for enable i2c slave.
  - Input:
      * bus_num: Bus number with zero base
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
static int do_i2c_slave_register(uint8_t bus_num){
  int ret = 0;

  /* Check input and slave status */
  uint8_t slave_status = i2c_slave_status_get(bus_num);
  if (slave_status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR | I2C_SLAVE_NOT_INIT)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, slave_status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  /* Check register status */
  if ( !(slave_status & I2C_SLAVE_NOT_REGISTER) ){
    LOG_ERR("Bus[%d] has already been registered, please unregister first!", bus_num);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  struct i2c_slave_data *data = &i2c_slave_device_global[bus_num].data;

  /* check whether msgq is full */
  if (!k_msgq_num_free_get(&data->z_msgq_id)){
    LOG_ERR("Bus[%d] msgq is already full, can't register now, please read out message first!", bus_num);
    return I2C_SLAVE_API_MSGQ_ERR;
  }

  ret = i2c_slave_register(data->i2c_controller, &data->config);
  if (ret)
    return ret;
  
  i2c_slave_device_global[bus_num].is_register = 1;

  return I2C_SLAVE_API_NO_ERR;
}

/*
  - Name: do_i2c_slave_unregister
  - Description: Set config to register for disable i2c slave.
  - Input:
      * bus_num: Bus number with zero base
      * mutex_flag: skip check if 1, otherwise 0
  - Return: 
      * 0, if no error
      * others, get error(check "i2c_slave_api_error_status")
*/
static int do_i2c_slave_unregister(uint8_t bus_num){
  int ret = 0;

  /* Check input and slave status */
  uint8_t slave_status = i2c_slave_status_get(bus_num);
  if (slave_status & (I2C_SLAVE_BUS_INVALID | I2C_SLAVE_CONTROLLER_ERR | I2C_SLAVE_NOT_INIT)){
    LOG_ERR("Bus[%d] check status failed with error status 0x%x!", bus_num, slave_status);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  /* Check register status */
  if ( slave_status & I2C_SLAVE_NOT_REGISTER ){
    LOG_ERR("Bus[%d] has already been unregistered, please register first!", bus_num);
    return I2C_SLAVE_API_BUS_GET_FAIL;
  }

  struct i2c_slave_data *data = &i2c_slave_device_global[bus_num].data;

  ret = i2c_slave_unregister(data->i2c_controller, &data->config);
  if (ret)
    return ret;
  
  i2c_slave_device_global[bus_num].is_register = 0;

  return I2C_SLAVE_API_NO_ERR;
}