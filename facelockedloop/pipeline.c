#if defined(HAVE_OPENCV2)

#include "highgui/highgui_c.h"
#include "imgproc/imgproc_c.h"
#include "objdetect/objdetect.hpp"

int stage_up(struct stage *stg, const struct stage_params *p,
	     struct stage_ops *o, struct pipeline *pipe)
{
	pthread_attr_t attr;

	stg->params = p;
	stg->ops = o;
	stg->pipeline = pipe;
	timespec_zero(&stg->duration);
	timespec_zero(&stg->stats.lastrun);
	stg->stats.ofinterest = 0;
	stg->stats.persecond = 0;

	if (stg->params.nth_stage < PIPELINE_MAX_STAGE)
		stg->next = &(pipe->stages[stg->params.nth_stage]);
	else
		stg->next = NULL; /* end of pipeline */

	sem_init(&stg->nowait);
	sem_init(&stg->done);
	mutex_init(&stg->lock);
	pthread_cond_init(&stg->sync, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&stg->worker, &attr, stage_worker, stg);
	ptrhead_attr_destroy(&attr);
	
	return 0;
}

static void *stage_worker(void *arg)
{
	struct timespec start, stop;
	struct stage *step = arg;
	int ret;
	
	for (;;)
	{
		//wait_for_previous_step
		ret = sem_wait(&step->nowait);
		if (ret)
			printf("step %d wait error %d./n",
			       step->params.nth_stage, ret);
		clock_gettime(CLOCK_MONOTONIC, &start);
		ret = step->ops->run(step);
		clock_gettime(CLOCK_MONOTONIC, &stop);
		timespec_sub(&step->duration, &stop, &start);
	}
	if (ret < 0)
		printf("step %d failed./n", step->params.nth_stage);

	printf("step %d performance: nofinterest=%d./n",
	       step->stats.ofinterest);

	//post to next step
	sem_post(&step->done);
	return NULL;
}

void stage_go(struct stage *stg)
{
	sem_post(&stg->nowait);
}

void stage_wait(struct stage *stg)
{
	ret = sem_wait(&stg->done);
	if (ret)
		printf("%s: step %d wait error %d./n",
		       __func__, step->params.nth_stage, ret);

}

int stage_passit(struct stage *stg, void *it)
{
	struct stage *next = stg->next;

	if (next) {
		mutex_lock(&next->lock);
		next->params.data_in = it;
		pthread_cond_signal(next->sync);
		mutex_unlock(&next->lock);
	}

}

int stage_processit(struct stage *stg, void **it)
{
	mutex_lock(&stg->lock);
	while (stg->params->data_in == NULL)
		pthread_cond_wait(&stg->sync, &stg->lock);

	mutex_unlock(&stg_lock);
	it = stg->params->data_in;
}
	
void stage_down(struct stage *stg)
{
	pthread_cancel(stg->worker);
	pthread_join(stg->worker, NULL);
	pthread_cond_destroy(&stg->sync);
	mutex_destroy(&stg->lock);
	sem_destroy(&stg->nowait);
	sem_destroy(&stg->done);
}

void stage_printstats(struct stage *stg)
{
	return 0;
}

void pipeline_init(struct pipeline *pipe)
{
	pipe->count = 0;
	pipe->status = 0;
	memset(pipe->stgs, 0, PIPELINE_MAX_STAGE * (sizeof(struct stage)));
}

int pipeline_register(struct pipeline *pipe, struct stage *stg)
{
	if (!stg)
		return -EINVAL;
	
	pipe->stgs[stg->params->nth_stage] = stg;
	++(pipe->count);
	return 0;	
}

int pipeline_run(int vindex, enum object_detector_t cdt, int scale)
{
	IplImage* srcframe, *dstframe;
	CvSeq* faces;
	int n, ret;
	
	ret = capture_initialize(vindex, cdt);
	if (ret < 0)
		return ret;

	srcframe = 0;
	dstframe = 0;	
	for (n=0; ; n++) {
		printf("%s: step1 get frame.\n", __func__);
		ret = capture_run(srcframe, n);
		if (ret < 0)
			break;

		printf("%s: step2 configure.\n", __func__);
		ret = capture_configure_detector(srcframe, dstframe, cdt);
		if (ret < 0)
			break;
		printf("%s: step3 run detector on frame.\n", __func__);
		faces = capture_run_detector(dstframe, scratchbuf, cdt);
		if (!faces)
			continue;
		printf("%s: step4 draw bboxes on frame.\n", __func__);
		ret = capture_overlay_bboxes(faces, dstframe, scale);
	}

	capture_tear_down(dstframe, cdt);
	return ret;
}


#else

int capture_process(enum capture_detector_t cdt, int scale)
{
	return -ENODEV;
}

int capture_get_facecount(void)
{
	return 0;
}

int capture_read_locations(struct store_box *faceset)
{
	faceset = 0;
	return -ENODEV;
}
#endif /*HAVE_OPENCV2*/


int pipeline_getcount(struct pipeline *pipe)
{
	return pipe->count;
}
