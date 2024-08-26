enum {
	IT_LEAK_E_0,
	IT_LEAK_E_1,
	IT_LEAK_E_2,
	IT_LEAK_E_3,
};

void it_leak_action_0();
void it_leak_action_1();
void it_leak_action_2();
void it_leak_action_3();

void emergency_button_action();
void fault_leak_action();
void deassert_all_rpu_ready_pin();
void set_all_rpu_ready_pin_normal(void);
void aalc_leak_behavior(uint8_t sensor_num);