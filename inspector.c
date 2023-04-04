#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <endian.h>
#include <math.h>
#include <time.h>
#include <ncurses.h>
#include <hidapi/hidapi.h>
#include "cglm/cglm.h"
#include "../Fusion/Fusion/Fusion.h"
#include <signal.h>

static volatile int keepRunning = 1;

void intHandler(int dummy) {
    keepRunning = 0;
}

#define AIR_VID 0x3318
#define AIR_PID 0x0424

// this is a guess
// ticks are in nanoseconds, 1000 Hz packets
#define TICK_LEN (1.0f / 1E9f)

// based on 24bit signed int w/ FSR = +/-2000 dps, datasheet option
#define GYRO_SCALAR (1.0f / 8388608.0f * 2000.0f)

// based on 24bit signed int w/ FSR = +/-16 g, datasheet option
#define ACCEL_SCALAR (1.0f / 8388608.0f * 16.0f)

#define MAG_SCALAR (1.0f / 32768.0f) * 1600.0f
#define MAG_OFFSET -1600.0f


static int rows, cols;
static versor rotation = GLM_QUAT_IDENTITY_INIT;
static vec3 ang_vel = {}, accel_vec = {}, mag_vec = {};
static vec3 mag_max_vec = {}, mag_min_vec = {}, mag_w_offset_vec = {};
static FusionQuaternion quaternion;
static FusionEuler euler;
static FusionVector earth;
static bool stats_init = true;
static FILE *fpt;
static struct timespec start_clock, now_clock;


#define SAMPLE_RATE (1000) // replace this with actual sample rate

typedef struct {
	uint64_t tick;
	int32_t ang_vel[3];
	int32_t accel[3];
	uint16_t mag[3];
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

	// mag data
	buffer += 6;
	
	out_sample->mag[0] = *(buffer++) | (*(buffer++) << 8);
	out_sample->mag[1] = *(buffer++) | (*(buffer++) << 8);
	out_sample->mag[2] = *(buffer++) | (*(buffer++) << 8);
	
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
process_mag(const uint16_t in_mag[3], vec3 out_vec)
{
	// these scale and bias corrections are all rough guesses
	out_vec[0] = (float)(in_mag[0]) * MAG_SCALAR + MAG_OFFSET;
	out_vec[1] = (float)(in_mag[2]) * MAG_SCALAR + MAG_OFFSET;
	out_vec[2] = (float)(in_mag[1]) * MAG_SCALAR + MAG_OFFSET;
}

static void
process_mag_stats()
{
	if(stats_init)
	{
		glm_vec3_copy(mag_vec, mag_max_vec);
		glm_vec3_copy(mag_vec, mag_min_vec);
		stats_init = false;		

	}else {
		int n = 0;
		mag_max_vec[n] = fmax(mag_max_vec[n],mag_vec[n]);
		mag_min_vec[n] = fmin(mag_min_vec[n],mag_vec[n]);
		n=1;
		mag_max_vec[n] = fmax(mag_max_vec[n],mag_vec[n]);
		mag_min_vec[n] = fmin(mag_min_vec[n],mag_vec[n]);
		n=2;
		mag_max_vec[n] = fmax(mag_max_vec[n],mag_vec[n]);
		mag_min_vec[n] = fmin(mag_min_vec[n],mag_vec[n]);
	}

	int n = 0;
	mag_w_offset_vec[n] = (mag_vec[n] - ((mag_max_vec[n] + mag_min_vec[n]) / 2.0f));
	n = 1;
	mag_w_offset_vec[n] = (mag_vec[n] - ((mag_max_vec[n] + mag_min_vec[n]) / 2.0f));
	n = 2;
	mag_w_offset_vec[n] = (mag_vec[n] - ((mag_max_vec[n] + mag_min_vec[n]) / 2.0f));
	
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

	float mag_magnitude = sqrt( mag_vec[0]*mag_vec[0] + mag_vec[1]*mag_vec[1] + mag_vec[2] * mag_vec[2]);
	float heading = atan2(mag_vec[0], mag_vec[1]) * 180.0f / M_PI;

	float offset_magnitude = sqrt( mag_w_offset_vec[0]*mag_w_offset_vec[0] + mag_w_offset_vec[1]*mag_w_offset_vec[1] + mag_w_offset_vec[2] * mag_w_offset_vec[2]);
	float offset_heading = atan2(mag_w_offset_vec[0], mag_w_offset_vec[1]) * 180.0f / M_PI;
	float heading1 = atan2(mag_w_offset_vec[1], mag_w_offset_vec[0]) * 180.0f / M_PI;
	float heading2 = atan2(mag_w_offset_vec[0], mag_w_offset_vec[2]) * 180.0f / M_PI;
	float heading3 = atan2(mag_w_offset_vec[2], mag_w_offset_vec[0]) * 180.0f / M_PI;
	float heading4 = atan2(mag_w_offset_vec[1], mag_w_offset_vec[2]) * 180.0f / M_PI;
	float heading5 = atan2(mag_w_offset_vec[2], mag_w_offset_vec[1]) * 180.0f / M_PI;

	clock_gettime(CLOCK_REALTIME, &now_clock);
	time_t seconds_elapsed = now_clock.tv_sec - start_clock.tv_sec;    // in seconds

	mvprintw(y++, x, "Time Elapsed (s): %jd", seconds_elapsed);
	mvprintw(y++, x, "Rate: %.3f %.3f %.3f", ang_vel[1], ang_vel[0], ang_vel[2]);
	// mvprintw(y++, x, "Angle: %.3f %.3f %.3f", euler_vec[2] * 180.f / 3.14159f, euler_vec[0] * 180.f / 3.14159f, euler_vec[1] * 180.f / 3.14159f);
	mvprintw(y++, x, "Roll %0.1f, Pitch %0.1f, Yaw %0.1f",euler.angle.roll, euler.angle.pitch, euler.angle.yaw);
	mvprintw(y++, x, "Gyro Scalar: %.9f", GYRO_SCALAR);
	mvprintw(y++, x, "Accel: %.3f %.3f %.3f", accel_vec[0], accel_vec[1], accel_vec[2]);
	mvprintw(y++, x, "Earth Accel: X %0.1f, Y %0.1f, Z %0.1f", earth.axis.x, earth.axis.y, earth.axis.z);
	mvprintw(y++, x, "Accel Scalar: %.9f", ACCEL_SCALAR);
	mvprintw(y++, x, "Mag: %.6f %.6f %.6f %.6f, %.6f", mag_vec[0], mag_vec[1], mag_vec[2], mag_magnitude, heading);
	mvprintw(y++, x, "Mag Max: %.6f %.6f %.6f", mag_max_vec[0], mag_max_vec[1], mag_max_vec[2]);
	mvprintw(y++, x, "Mag Min: %.6f %.6f %.6f", mag_min_vec[0], mag_min_vec[1], mag_min_vec[2]);
	mvprintw(y++, x, "Mag Offset: %.6f %.6f %.6f", ((mag_max_vec[0] + mag_min_vec[0]) / 2.0f), ((mag_max_vec[1] + mag_min_vec[1]) / 2.0f), ((mag_max_vec[2] + mag_min_vec[2]) / 2.0f));
	mvprintw(y++, x, "Mag w/ Offset: %.6f %.6f %.6f %.6f %.6f", mag_w_offset_vec[0], mag_w_offset_vec[1], mag_w_offset_vec[2], offset_magnitude, offset_heading);
	mvprintw(y++, x, "Offset headings: %.6f %.6f %.6f %.6f %.6f %.6f", offset_heading, heading1, heading2, heading3, heading4, heading5);
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

	signal(SIGINT, intHandler);

	// open log file
	char filename[40];
	struct tm *timenow;

	time_t now = time(NULL);
	timenow = gmtime(&now);

	strftime(filename, sizeof(filename), "./logs/data_%Y-%m-%d_%H-%M-%S.csv", timenow);

	fpt = fopen(filename, "w+");

	if(fpt == NULL) {
		// get out code
		printf("File open failed. Exiting...");
		exit(1);
	}	
	

	fprintf(fpt,"ts_nanoseconds, gyro1_dps, gyro2_dps, gyro3_dps, accel1_g, accel2_g, accel3_g, mag1_uT, mag2_uT, mag3_uT, euler1_deg, euler2_deg, euler3_deg, quat1, quat2, quat3, quat4, accel_err_deg, accel_ignored, accel_rej_timer,mag_err_deg,mag_ignored,mag_rej_timer, initialising, accel_rej_warn, accel_rej_timeout, mag_rej_warn, mag_rej_timeout,\n");
	
	
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
	
	FusionAhrsInternalStates internal_states = FusionAhrsGetInternalStates(&ahrs);
	FusionAhrsFlags ahrs_flags = FusionAhrsGetFlags(&ahrs);

	fprintf(fpt," %f, %d, %f, %f, %d, %f,",internal_states.accelerationError, internal_states.accelerometerIgnored, internal_states.accelerationRejectionTimer,
																											internal_states.magneticError, internal_states.magnetometerIgnored, internal_states.magneticRejectionTimer);
	fprintf(fpt," %d, %d, %d, %d, %d\n", ahrs_flags.initialising, ahrs_flags.accelerationRejectionWarning, ahrs_flags.accelerationRejectionTimeout, 
																										 ahrs_flags.magneticRejectionWarning, ahrs_flags.magneticRejectionTimeout);




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

	uint16_t read_count = 0;

	clock_gettime(CLOCK_REALTIME, &start_clock);

	
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
		process_mag(sample.mag, mag_vec);	
		process_mag_stats();
		

			
		 // Acquire latest sensor data
        const uint64_t timestamp = sample.tick;
        FusionVector gyroscope = {ang_vel[0], ang_vel[1], ang_vel[2]};
        FusionVector accelerometer = {accel_vec[0], accel_vec[1], accel_vec[2]}; 
       
        // Apply calibration
        gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
        accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
		
		// Update gyroscope offset correction algorithm
        gyroscope = FusionOffsetUpdate(&offset, gyroscope);

        // Calculate delta time (in seconds) to account for gyroscope sample clock error
        static uint64_t previousTimestamp;
        const float deltaTime = (float) (timestamp - previousTimestamp) / (float) 1e9;
        previousTimestamp = timestamp;

		if(deltaTime > 0.01){
			FusionAhrsReset(&ahrs);
			continue;
		}

        // Update gyroscope AHRS algorithm
        FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, deltaTime);

        // Print algorithm outputs
		quaternion = FusionAhrsGetQuaternion(&ahrs);
        euler = FusionQuaternionToEuler(quaternion);
        earth = FusionAhrsGetEarthAcceleration(&ahrs);

		// FusionAhrsInternalStates 
		internal_states = FusionAhrsGetInternalStates(&ahrs);
		// FusionAhrsFlags 
		ahrs_flags = FusionAhrsGetFlags(&ahrs);

		update_rotation(deltaTime,ang_vel);
		
		fprintf(fpt,"%lu, %f, %f, %f, %f, %f, %f, %f, %f, %f,",timestamp, ang_vel[0],ang_vel[1],ang_vel[2],accel_vec[0], accel_vec[1], accel_vec[2], mag_vec[0], mag_vec[1], mag_vec[2]);
		fprintf(fpt," %f, %f, %f, %f, %f, %f, %f,", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, quaternion.element.w, quaternion.element.x,quaternion.element.y,quaternion.element.z);
		fprintf(fpt," %f, %d, %f, %f, %d, %f,",internal_states.accelerationError, internal_states.accelerometerIgnored, internal_states.accelerationRejectionTimer,
																											internal_states.magneticError, internal_states.magnetometerIgnored, internal_states.magneticRejectionTimer);
		fprintf(fpt," %d, %d, %d, %d, %d\n", ahrs_flags.initialising, ahrs_flags.accelerationRejectionWarning, ahrs_flags.accelerationRejectionTimeout, 
																										 ahrs_flags.magneticRejectionWarning, ahrs_flags.magneticRejectionTimeout);
		
		read_count++;

		if(read_count % 500 == 0)
		{
			print_report();
			refresh();					
		}

		// if(read_count % 30000 == 0)
		// {
		// 	stats_init = true;
		// }

		
	} while (res && keepRunning);
	getch();
	endwin();

	hid_close(device);
	res = hid_exit();

	fclose(fpt);

	printf("Logfile: %s\n", filename);

	printf("Exiting...\n");
	return 0;
}
