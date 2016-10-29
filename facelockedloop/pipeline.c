#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>

#include "pipeline.h"
#include "time_utils.h"

static void *stage_worker(void *arg);

void stage_up(struct stage *stg, struct stage_params *p,
	     struct stage_ops *o, struct pipeline *pipe)
{
	pthread_attr_t attr;

	stg->params = *p;
	stg->ops = o;
	stg->pipeline = pipe;
	timespec_zero(&stg->duration);
	timespec_zero(&stg->stats.lastrun);
	stg->stats.ofinterest = 0;
	stg->stats.persecond = 0;

	if (stg->params.nth_stage < PIPELINE_MAX_STAGE)
		stg->next = &(pipe->stgs[stg->params.nth_stage]);
	else
		stg->next = NULL; /* end of pipeline */

	sem_init(&stg->nowait, 0, 0);
	sem_init(&stg->done, 0, 0);
	pthread_mutex_init(&stg->lock, NULL);
	pthread_cond_init(&stg->sync, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&stg->worker, &attr, stage_worker, stg);
	pthread_attr_destroy(&attr);
	o->up(stg, p, o, pipe);
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
		timespec_substract(&step->duration, &stop, &start);
	}
	if (ret < 0)
		printf("step %d failed./n", step->params.nth_stage);

	printf("step %d performance: nofinterest=%ld./n",
	       step->params.nth_stage,
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
	int ret = sem_wait(&stg->done);
	if (ret)
		printf("%s: step %d wait error %d./n",
		       __func__, stg->params.nth_stage, ret);

}

int stage_passit(struct stage *stg, void *it)
{
	struct stage *next = stg->next;

	if (next) {
		pthread_mutex_lock(&next->lock);
		next->params.data_in = it;
		pthread_cond_signal(&next->sync);
		pthread_mutex_unlock(&next->lock);
	}
	return 0;
}

int stage_processit(struct stage *stg, void **it)
{
	pthread_mutex_lock(&stg->lock);
	while (stg->params.data_in == NULL)
		pthread_cond_wait(&stg->sync, &stg->lock);

	pthread_mutex_unlock(&stg->lock);
	*it = stg->params.data_in;
	return 0;
}
	
void stage_down(struct stage *stg)
{
	pthread_cancel(stg->worker);
	pthread_join(stg->worker, NULL);
	pthread_cond_destroy(&stg->sync);
	pthread_mutex_destroy(&stg->lock);
	sem_destroy(&stg->nowait);
	sem_destroy(&stg->done);
}

void stage_printstats(struct stage *stg)
{
	return;
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
	
	pipe->stgs[stg->params.nth_stage] = *stg;
	++(pipe->count);
	return 0;	
}

int pipeline_deregister(struct pipeline *pipe, struct stage *stg)
{
	if (!stg)
		return -EINVAL;
	
	memset(&pipe->stgs[stg->params.nth_stage], 0, (sizeof(struct stage)));
	--(pipe->count);
	return 0;	
}

int pipeline_run(struct pipeline *pipe)
{
	int l, n, ret;
	struct stage *s;
	
	ret = 0;
	for (l=0; ; l++) {
		for (n=CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {
			s = &pipe->stgs[CAPTURE_STAGE];
			printf("%s: run stage %d.\n", __func__,
			       s->params.nth_stage);
			ret = s->ops->run(s);
			if (ret < 0)
				break;
		};
		
		if (ret < 0)
			break;
	}

	return ret;
}

int pipeline_getcount(struct pipeline *pipe)
{
	return pipe->count;
}

void pipeline_teardown(struct pipeline *pipe)
{
	struct stage *s;
	int n;
	
	for (n=CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {
		s = &pipe->stgs[CAPTURE_STAGE];
		if (s->self) {
			printf("%s: run stage %d.\n", __func__,
			       s->params.nth_stage);
			s->ops->down(s);
		};
	};
}
