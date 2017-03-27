/**
 * @file facelockedloop/main.c
 * @brief Application that allows face tracking within a given video stream.
 * 
 * @author Raquel Medina <raquel.medina.rodriguez@gmail.com>
 *
 */
#include <sys/types.h>
#include <time.h>
#include <signal.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>

#include "pipeline.h"
#include "capture.h"
#include "detect.h"
#include "track.h"
#include "time_utils.h"
#include "debug.h"

extern const char *fll_version_name;
#define FLL_MAX_SERVO_COUNT SERVOLIB_MAX_SERVO_COUNT
#define FLL_SERVO_COUNT 2
#define FLL ((struct stage *)NULL)

static struct pipeline fllpipe;
static volatile sigset_t set;


static int with_output;
static const struct option options[] = {
	{
#define help_opt	0
		.name = "help",
		.has_arg = 0,
		.flag = NULL,
	},
	{
#define xmlfile_opt	1
		.name = "xmlfile",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define output_opt	2
		.name = "output",
		.has_arg = 2,
		.flag = &with_output,
		.val = 1,
	},
	{
#define camera_opt      3
		.name = "video",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define algrthm_opt 4
		.name = "algorithm",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define trackdev_opt 5
		.name = "servodev",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define trackpan_opt   6
		.name = "panchannel",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define tracktilt_opt   7
		.name = "tiltchannel",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define nloops_opt 8
		.name = "loops",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define dmins_opt 9
		.name = "min_s",
		.has_arg = 1,
		.flag = NULL,
	},
	{
#define dmaxs_opt 10
		.name = "max_s",
		.has_arg = 1,
		.flag = NULL,
	},
};

static void usage(void)
{
	fprintf(stderr, "usage: fll  <options>, with:                   \n");
	fprintf(stderr, "            --xmlfile=<filepath/filename>   "
		":specifies detector config file (default: ~/cascade.xml)\n");
	fprintf(stderr, "            --output[=<file-tmpl>]          "
		":dump output data (default: discard)                    \n");
	fprintf(stderr, "            --video[=<camera-index>] 	     "
		":specifies which camera to use (default: any camera)    \n");
	fprintf(stderr, "            --algorithm[=<haar>|<lsvm>]     "
		":select which detection algorithm to use (default: haar)\n");
	fprintf(stderr, "            --servodevnode=<dev-node-index> "
		":specifies the servos device control node (default: 0)  \n");
	fprintf(stderr, "            --panchannel[=<channel-index>]  "
		":specifies which channel the pan servo is connected to "
		"(default: 2)\n");
	fprintf(stderr, "            --tiltchannel[=<channel-index>] "
		":specifies which channel the tilt servo is connected to "
		"(default: 5)\n");
	fprintf(stderr, "            --nloops=<n>                    "
		":specifies number of pipeline iterations to run, if 0 run "
		"forever (default: 0)\n");
	fprintf(stderr, "            --min_s=<n>]                    "
		":specifies min size for the detector (default: 80)     \n");
	fprintf(stderr, "            --max_s=<n>]                    "
		":specifies max size for the detector (default: 180)    \n");
	fprintf(stderr, "            --help                          "
		"this help\n");
}


/*helper function*/
static void *signal_catch(void *arg)
{
	sigset_t *monitorset = (sigset_t*)arg;
	int sig;
	for (;;) {
		sigwait(monitorset, &sig);
		
		printf("caught signal %d. Terminate!\n", sig);
		if (sig == SIGINT) {
			pipeline_terminate(&fllpipe, -EINTR);
			break;
		}
	}
	return NULL;
}

	
static void setup_term_signals(void)
{
	pthread_attr_t attr;
	pthread_t id;
       

	sigemptyset((sigset_t*)&set);
	sigaddset((sigset_t*)&set, SIGTERM); /* 0x4000 */
	sigaddset((sigset_t*)&set, SIGHUP);  /* 0x0001 */
	sigaddset((sigset_t*)&set, SIGINT);  /* 0x0002 */
	sigaddset((sigset_t*)&set, SIGQUIT); /* 0x0003 */
	
	/*all threads created after this point will share same mask.*/
	pthread_sigmask(SIG_BLOCK, (sigset_t*)&set, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	pthread_create(&id, &attr, signal_catch, (sigset_t*)&set);
	pthread_attr_destroy(&attr);
}

int main(int argc, char *const argv[])
{
	char *outfile, *xmlfile;
	int video = 0;
	struct timespec start_time, stop_time, duration;
	int pos[FLL_MAX_SERVO_COUNT] =
		{ [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	int speed[FLL_MAX_SERVO_COUNT] =
		{ [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	int accel[FLL_MAX_SERVO_COUNT] =
		{ [0 ... FLL_MAX_SERVO_COUNT -1] = -1};
	struct tracker_params servo_params;
	struct imager_params camera_params;
	struct detector_params algorithm_params;
	struct imager camera;
	struct detector algorithm;
	struct tracker servo;
	enum object_detector_t dtype = CDT_HAAR;
	int lindex, c, i, l, ret, panchannel, tiltchannel, servodevnode, loops;
	int dmins, dmaxs;
	char ch;
	servodevnode = 0;
	outfile = NULL;
	xmlfile = "haarcascade_frontalface_default.xml";
	panchannel = 1;
	tiltchannel = 5;
	loops = 0;
	dmins = 100;
	dmaxs = 180;
	
	/* get local configurations */
	for (;;) {
		lindex = -1;
		c = getopt_long_only(argc, argv, "", options, &lindex);
		if (c == EOF)
			break;
		switch (lindex) {
		case help_opt:
			usage();
			exit(0);
		case xmlfile_opt:
			xmlfile = optarg;
			break;
		case output_opt:
			outfile = optarg;
			break;
		case camera_opt:
			video = atoi(optarg);
			break;
		case algrthm_opt:
			if (optarg && strncmp(optarg, "lsvm",4) == 0)
				dtype = CDT_LSVM;
			break;
		case trackdev_opt:
			servodevnode = atoi(optarg);
			break;
		case trackpan_opt:
			panchannel = atoi(optarg);
			break;
		case tracktilt_opt:
			tiltchannel = atoi(optarg);
			break;
		case nloops_opt:
			loops = atoi(optarg);
			break;
		case dmins_opt:
			dmins = atoi(optarg);
			break;
		case dmaxs_opt:
			dmaxs = atoi(optarg);
			break;
		default:
			usage();
			exit(1);
		}
	}
	if (xmlfile != NULL)
		printf("cascade filter:%s.\n", xmlfile);
	if (outfile != NULL)
		printf("output data:%s.\n", outfile);

	setup_term_signals();

	pipeline_init(&fllpipe);


	/* first stage */
	camera_params.name = malloc(10);
	ret = asprintf(&camera_params.name, "FLL cam%d", video);
	if (ret < 0)
		goto terminate;
	

	camera_params.vididx = video;
	camera_params.frame = NULL;
	camera_params.videocam = NULL;
	ret = capture_initialize(&camera, &camera_params, &fllpipe);
	if (ret) {
		printf("capture init ret:%d.\n", ret);
		goto terminate;
	}
	/* second stage */
	algorithm_params.odt = dtype;
	algorithm_params.cascade_xml = xmlfile;
	algorithm_params.srcframe = NULL;
	algorithm_params.dstframe = NULL;
	algorithm_params.algorithm = NULL;
	algorithm_params.scratchbuf = NULL;
	algorithm_params.min_size = dmins;
	algorithm_params.max_size = dmaxs;
	ret = detect_initialize(&algorithm, &algorithm_params, &fllpipe);
	if (ret) {
		printf("detection init ret:%d.\n", ret);
		goto terminate;
	}
	/* third stage */
	servo_params.pan_tgt = 0;
	servo_params.tilt_tgt = 0;
	servo_params.dev = servodevnode;
	servo_params.pan_params.channel = panchannel;
	servo_params.tilt_params.channel = tiltchannel;

	ret = track_initialize(&servo , &servo_params, &fllpipe);
	if (ret) {
		printf("tracking init ret:%d.\n", ret);
		goto terminate;
	}

	ret =pipeline_getcount(&fllpipe);
	if (ret != PIPELINE_MAX_STAGE) {
		printf("missing stages for fll, only %d present.\n", ret);
		goto terminate;
	}

	
	for (i=0; i < FLL_MAX_SERVO_COUNT; i++)
		debug(FLL, "servo channel %d, pos:%d, speedLim:%d, accelLim:%d.\n",
		       i, pos[i], speed[i], accel[i]);

	clock_gettime(CLOCK_MONOTONIC, &start_time);
	for (l=0; ret >= 0; l++)  {
		debug(FLL, "loop:%d.\n", l);
		ret = pipeline_run(&fllpipe);
		if (ret) {
			printf("Cannot run FLL, ret:%d.\n", ret);
			break;
		}
#if 0		
		ret = pipeline_pause(&fllpipe);
		if (ret){
			printf("Cannot pause FLL, ret:%d.\n", ret);
			break;
		} 
#endif
        	if (fllpipe.status == STAGE_ABRT)
			break;

		if (loops && (l >= loops))
			break;
	};
terminate:
	printf("camara %d: %s.\n", camera_params.vididx, camera_params.name);
	pipeline_teardown(&fllpipe);
	clock_gettime(CLOCK_MONOTONIC, &stop_time);
	timespec_substract(&duration, &stop_time, &start_time);
	printf("duration->  %lds %ldns .\n", duration.tv_sec , duration.tv_nsec );
        free(camera_params.name);	
	printf("press a key to continue\n");
	ch = getchar();
	if (ch)
		goto exitfll;
exitfll:
	return 0;
}
