#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <endian.h>
#include <ncurses.h>
#include <hidapi/hidapi.h>
#include "cglm/cglm.h"
#include "../Fusion/Fusion/Fusion.h"

#define AIR_VID 0x3318
#define AIR_PID 0x0424

// this is a guess
// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)


static int rows, cols;
static versor rotation = GLM_QUAT_IDENTITY_INIT;
static vec3 ang_vel = {}, accel_vec = {};
static FusionEuler euler;
static FusionVector earth;

#define SAMPLE_RATE (1000) // replace this with actual sample rate

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
} air_sample;

static int
parse_report(const unsigned char* buffer, int size, air_sample* out_sample)
{
	if (size != 64) {
		printf("Invalid packet size");
		return -1;
	}
	// clock in nanoseconds
	buffer += 4;
	out_sample->tick = ((uint64_t)*(buffer++)) | (((uint64_t)*(buffer++))  << 8) | (((uint64_t)*(buffer++)) << 16) | (((uint64_t)*(buffer++)) << 24) 
						| (((uint64_t)*(buffer++)) << 32) | (((uint64_t)*(buffer++))  << 40) | (((uint64_t)*(buffer++))  << 48) | (((uint64_t)*(buffer++))  << 56);
	
	// gyroscope measurements
	buffer += 6;
	if( *(buffer+2) & 0x80 ) {
		out_sample->ang_vel[0] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->ang_vel[0] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	
	if( *(buffer+2) & 0x80 ) {
		out_sample->ang_vel[1] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->ang_vel[1] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if( *(buffer+2) & 0x80 ) {
		out_sample->ang_vel[2] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->ang_vel[2] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	// accelerometer data
	buffer += 6;
	if( *(buffer+2) & 0x80 ) {
		out_sample->accel[0] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->accel[0] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}
	
	if( *(buffer+2) & 0x80 ) {
		out_sample->accel[1] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->accel[1] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	if( *(buffer+2) & 0x80 ) {
		out_sample->accel[2] = (0xff << 24) | *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	} else {
		out_sample->accel[2] = *(buffer++) | (*(buffer++) << 8) | (*(buffer++) << 16);
	}

	return 0;
}

static void
process_ang_vel(const int32_t in_ang_vel[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_ang_vel[0]) * -1.0f * GYRO_SCALAR;
	out_vec[1] = (float)(in_ang_vel[2]) * GYRO_SCALAR;
	out_vec[2] = (float)(in_ang_vel[1]) * GYRO_SCALAR;
}

static void
process_accel(const int32_t in_accel[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_accel[0]) * ACCEL_SCALAR;
	out_vec[1] = (float)(in_accel[2]) * ACCEL_SCALAR;
	out_vec[2] = (float)(in_accel[1]) * ACCEL_SCALAR;
}

static void
update_rotation(float dt, vec3 in_ang_vel)
{
	float ang_vel_length = glm_vec3_norm(in_ang_vel);

	if (ang_vel_length > 0.0001f) {
		vec3 rot_axis = { in_ang_vel[0] / ang_vel_length, in_ang_vel[1] / ang_vel_length, in_ang_vel[2] / ang_vel_length };
		float rot_angle = ang_vel_length * dt;

		versor delta_rotation;
		glm_quatv(delta_rotation, rot_angle, rot_axis);
		glm_quat_mul(rotation, delta_rotation, rotation);
	}

	glm_quat_normalize(rotation);
}

static void
print_report()
{
	const int HEIGHT = 3;
	const int WIDTH = 20;
	int x = (cols - WIDTH) / 2;
	int y = (rows - HEIGHT) / 2;

	 // Convert versor to rotation matrix
	mat4 rotation_mat;
	glm_mat4_identity(rotation_mat);
	glm_quat_mat4(rotation, rotation_mat);

	vec3 euler_vec;
	glm_euler_angles(rotation_mat, euler_vec);

	// glm_vec3_scale(euler_vec,(180.f / 3.14159f), euler_vec);


	mvprintw(y++, x, "Rate: %.3f %.3f %.3f", ang_vel[1], ang_vel[0], ang_vel[2]);
	// mvprintw(y++, x, "Angle: %.3f %.3f %.3f", euler_vec[2] * 180.f / 3.14159f, euler_vec[0] * 180.f / 3.14159f, euler_vec[1] * 180.f / 3.14159f);
	mvprintw(y++, x, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f",euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
	mvprintw(y++, x, "Gyro Scalar: %.9f", GYRO_SCALAR);
	mvprintw(y++, x, "Accel: %.3f %.3f %.3f", accel_vec[0], accel_vec[1], accel_vec[2]);
	mvprintw(y++, x, "Earth Accel: X %0.1f, Y %0.1f, Z %0.1f", earth.axis.x, earth.axis.y, earth.axis.z);
	mvprintw(y++, x, "Accel Scalar: %.9f", ACCEL_SCALAR);
}

static void
print_bytes(const uint8_t* buf, size_t len)
{
	const int WIDTH = 16;
	int y = (rows - len / (WIDTH + 1)) / 2;
	int x = (cols - WIDTH * 2 - WIDTH + 1) / 2;

	for (int i = 0; i < len; i++) {
		if (i % WIDTH == 0) y++;
		mvprintw(y, x + 2 * (i % WIDTH) + (i % WIDTH), "%02x", buf[i]);
	}
}

static void
print_line(const char* s)
{
	int width = (width = strlen(s)) > cols ? cols : width;
	mvprintw(rows / 2, (cols - width) / 2, "%s", s);
}

static hid_device*
open_device()
{
	struct hid_device_info* devs = hid_enumerate(AIR_VID, AIR_PID);
	struct hid_device_info* cur_dev = devs;
	hid_device* device = NULL;

	while (devs) {
		if (cur_dev->interface_number == 3) {
			device = hid_open_path(cur_dev->path);
			break;
		}

		cur_dev = cur_dev->next;
	}

	hid_free_enumeration(devs);
	return device;
}

int
main(void)
{
	 // Define calibration (replace with actual calibration data if available)
    const FusionMatrix gyroscopeMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector gyroscopeSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector gyroscopeOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix accelerometerMisalignment = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector accelerometerSensitivity = {1.0f, 1.0f, 1.0f};
    const FusionVector accelerometerOffset = {0.0f, 0.0f, 0.0f};
    const FusionMatrix softIronMatrix = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    const FusionVector hardIronOffset = {0.0f, 0.0f, 0.0f};

    // Initialise algorithms
    FusionOffset offset;
    FusionAhrs ahrs;

    FusionOffsetInitialise(&offset, SAMPLE_RATE);
    FusionAhrsInitialise(&ahrs);

    // Set AHRS algorithm settings
    const FusionAhrsSettings settings = {
            .gain = 0.5f,
            .accelerationRejection = 10.0f,
            .magneticRejection = 20.0f,
            .rejectionTimeout = 5 * SAMPLE_RATE, /* 5 seconds */
    };
    FusionAhrsSetSettings(&ahrs, &settings);


	// open device
	hid_device* device = open_device();
	if (!device) {
		printf("Unable to open device\n");
		return 1;
	}

	// open the floodgates
	uint8_t magic_payload[] = { 0xaa, 0xc5, 0xd1, 0x21, 0x42, 0x04, 0x00, 0x19, 0x01 };
	int res = hid_write(device, magic_payload, sizeof(magic_payload));
	if (res < 0) {
		printf("Unable to write to device\n");
		return 1;
	}

	glm_quat_copy(GLM_QUAT_IDENTITY, rotation);

	initscr();
	cbreak();
	noecho();
	curs_set(0);

	unsigned char buffer[64] = {};
	uint64_t last_sample_tick = 0;
	air_sample sample = {};

	do {
		getmaxyx(stdscr, rows, cols);
		erase();

		res = hid_read(device, (void*)&buffer, sizeof(buffer));
		if (res < 0) {
			printf("Unable to get feature report\n");
			break;
		}

		if (buffer[0] != 0x01 || buffer[1] != 0x02){
			print_bytes((void*)&buffer, res);
			refresh();
			//getch();
			continue;
		}

		parse_report(buffer, sizeof(buffer), &sample);
		process_ang_vel(sample.ang_vel, ang_vel);
		process_accel(sample.accel, accel_vec);

		 // Acquire latest sensor data
        const uint64_t timestamp = sample.tick; // replace this with actual gyroscope timestamp
        FusionVector gyroscope = {ang_vel[0], ang_vel[1], ang_vel[2]}; // replace this with actual gyroscope data in degrees/s
        FusionVector accelerometer = {accel_vec[0], accel_vec[1], accel_vec[2]}; // replace this with actual accelerometer data in g
       
        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		
		// Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static uint64_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / (float) 1e9;
        previousTimestamp = timestamp;

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

        // Print algorithm outputs
        euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
        earth = FusionAhrsGetEarthAcceleration(&ahrs);

		update_rotation(deltaTime,ang_vel);
		
		print_report();
		refresh();
	} while (res);
	getch();
	endwin();

	hid_close(device);
	res = hid_exit();

	return 0;
}
