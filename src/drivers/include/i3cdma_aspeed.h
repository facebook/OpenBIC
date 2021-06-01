hal_status_t aspeed_i3cdma_request_channel(i3cdma_t *obj, uint32_t *channel);
hal_status_t aspeed_i3cdma_prepare(i3cdma_t *obj, i3cdma_desc_t *desc);
hal_status_t aspeed_i3cdma_go(i3cdma_t *obj, i3cdma_desc_t *desc);
hal_status_t aspeed_i3cdma_wait_done(i3cdma_t *obj, i3cdma_desc_t *desc);
hal_status_t aspeed_i3cdma_init(i3cdma_t *obj);