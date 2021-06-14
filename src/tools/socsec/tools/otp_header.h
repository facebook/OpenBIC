#define OTP_REG_RESERVED	-1
#define OTP_REG_VALUE		-2
#define OTP_REG_VALID_BIT	-3

struct otpstrap_info {
	signed char bit_offset;
	signed char length;
	signed char value;
	char *information;
};

struct otpconf_info {
	signed char dw_offset;
	signed char bit_offset;
	signed char length;
	signed char value;
	char *information;
};

