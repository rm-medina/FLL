#include <stdio.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>
#include <string.h>

#include "pipeline.h"
#include "time_utils.h"
#include "debug.h"

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
	timespec_zero(&stg->stats.overall);
	stg->stats.ofinterest = 0;
	stg->stats.persecond = 0;
	stg->next = NULL;
	sem_init(&stg->nowait, 0, 0);
	sem_init(&stg->done, 0, 0);
	pthread_mutex_init(&stg->lock, NULL);
	pthread_cond_init(&stg->sync, NULL);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	pthread_create(&stg->worker, &attr, stage_worker, stg);
	pthread_attr_destroy(&attr);
}

static void *stage_worker(void *arg)
{
	struct timespec start, stop;
	struct stage *step = arg;
	int ret;
	
	for (;;)
	{
		/*wait for 'go' signal*/
		ret = sem_wait(&step->nowait);
		if (ret)
			debug(step, "step %d wait error %d.\n",
			       step->params.nth_stage, ret);
		clock_gettime(CLOCK_MONOTONIC, &start);
		if (step->ops->input)
			ret = step->ops->input(step, NULL);
		if (ret)
			debug(step, "step %d input error %d.\n",
			       step->params.nth_stage, ret);
		ret = step->ops->run(step);
		if (ret)
			debug(step, "step %d run error %d.\n",
			       step->params.nth_stage, ret);
		clock_gettime(CLOCK_MONOTONIC, &stop);
		timespec_substract(&step->duration, &stop, &start);
		sem_post(&step->done);
	}
	if (ret < 0)
		debug(step, "step %d failed.\n", step->params.nth_stage);

	debug(step, "step %d performance: nofinterest=%ld.\n",
	       step->params.nth_stage,
	       step->stats.ofinterest);

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
		debug(stg, "%s: step %d wait error %d.\n",
		       __func__, stg->params.nth_stage, ret);

}

int stage_output(struct stage *stg, void *it)
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

int stage_input(struct stage *stg, void **it)
{
	pthread_mutex_lock(&stg->lock);
	while (stg->params.data_in == NULL)
		pthread_cond_wait(&stg->sync, &stg->lock);

	*it = stg->params.data_in;
	pthread_mutex_unlock(&stg->lock);
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
	memset(pipe->stgs, 0, PIPELINE_MAX_STAGE);
}

int pipeline_register(struct pipeline *pipe, struct stage *stg)
{
	if (!stg)
		return -EINVAL;

	pipe->stgs[stg->params.nth_stage] = stg;

	if (stg->params.nth_stage > 0)
		if (stg->params.nth_stage <= PIPELINE_MAX_STAGE)
			pipe->stgs[stg->params.nth_stage-1]->next = stg;

	++(pipe->count);
	return 0;	
}

int pipeline_deregister(struct pipeline *pipe, struct stage *stg)
{
	if (!stg)
		return -EINVAL;
	
	pipe->stgs[stg->params.nth_stage] = NULL;
	--(pipe->count);
	return 0;	
}

int pipeline_run(struct pipeline *pipe)
{
	int n, ret;
	struct stage *s;
	struct timespec delta, now, before;
	ret = 0;
	timespec_zero(&delta);
	timespec_zero(&now);
	timespec_zero(&before);
	
	for (n=CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {
		s = pipe->stgs[n];
		debug(s, "%s: run stage %d in:%p %p.\n", __func__,
		       s->params.nth_stage, s->params.data_in,
		       s->params.data_out);

		clock_gettime(CLOCK_MONOTONIC, &before);
		s->stats.lastrun.tv_sec = before.tv_sec;
		s->stats.lastrun.tv_nsec = before.tv_nsec;

		s->ops->go(s);
		ret = sem_wait(&s->done);
		if (ret) {
			debug(s, "step %d done error %d.\n",
			       s->params.nth_stage, ret);
			break;
		}
		if (s->ops->output && s->next) 
			s->ops->output(s, s->params.data_out);

		clock_gettime(CLOCK_MONOTONIC, &now);
		timespec_substract(&delta, &now, &before);
		timespec_add(&s->stats.overall, &delta);
		
		debug(s, "stage %d: delta->  %lds %ldns .\n", s->params.nth_stage,
		       delta.tv_sec , delta.tv_nsec);
		debug(s, "stage %d: count: %ld. \n", s->params.nth_stage,
		       s->stats.ofinterest);

		debug(s, "stage %d: overall: %lds %ldns. \n", s->params.nth_stage,
		       s->stats.overall.tv_sec,
		       s->stats.overall.tv_nsec);

		if (!(s->params.data_out))
		    break;

		s->stats.persecond =
			(s->stats.overall.tv_sec != 0) ?
			((s->stats.ofinterest) /
			 (s->stats.overall.tv_sec)) :
			s->stats.ofinterest;

		debug(s, "stage %d: frequency (count/s) : %ld .\n",
		       s->params.nth_stage,
		       s->stats.persecond);
		
		if (pipe->status) {
			printf("exiting pipeline...\n");
			ret = pipe->status;
			break;

		}
	};
	return ret;
}

void pipeline_terminate(struct pipeline *pipe, int reason)
{
	printf("aborting (reason:%d)/n", reason);
	pipe->status = STAGE_ABRT;
}

int pipeline_pause(struct pipeline *pipe)
{
	int n, ret;
	struct stage *s;
	
	ret = 0;
	for (n=CAPTURE_STAGE; n < PIPELINE_MAX_STAGE; n++) {
		s = pipe->stgs[n];
		printf("%s: pause stage %d.\n", __func__,
		       s->params.nth_stage);
		s->ops->wait(s);
	};
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
		s = pipe->stgs[n];
		if (s && s->self) {
			printf("%s: run stage %d.\n", __func__,
			       s->params.nth_stage);
			s->ops->down(s);
		};
	};
}
