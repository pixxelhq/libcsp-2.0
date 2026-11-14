

/*
Inspired by c-pthread-queue by Matthew Dickinson
http://code.google.com/p/c-pthread-queue/
*/

#include "pthread_queue.h"

#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <csp/csp.h>

static inline int get_deadline(struct timespec * ts, uint32_t timeout_ms) {
	int ret = clock_gettime(CLOCK_MONOTONIC, ts);

	if (ret < 0) {
		return ret;
	}

	uint32_t sec = timeout_ms / 1000;
	uint32_t nsec = (timeout_ms - 1000 * sec) * 1000000;

	ts->tv_sec += sec;

	if (ts->tv_nsec + nsec >= 1000000000) {
		ts->tv_sec++;
	}

	ts->tv_nsec = (ts->tv_nsec + nsec) % 1000000000;

	return ret;
}

static inline int init_cond_clock_monotonic(pthread_cond_t * cond) {

	int ret;
	pthread_condattr_t attr;

	pthread_condattr_init(&attr);
	ret = pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);

	if (ret == 0) {
		ret = pthread_cond_init(cond, &attr);
	}

	pthread_condattr_destroy(&attr);
	return ret;
}

/* A queue is shared by threads of very different real-time priority: the
 * buffer pool and connection queues serve both a 2 ms receive task and 1 s
 * senders. With a default mutex, a low-priority thread preempted inside the
 * few-microsecond critical section blocks a high-priority one for as long as
 * medium-priority threads keep running (measured: 1.7 ms on a 2 ms rate group,
 * PolarFire PREEMPT_RT). Priority inheritance bounds that wait to the critical
 * section. Falls back to a default mutex where the protocol is unsupported. */
static inline int init_mutex_prio_inherit(pthread_mutex_t * mutex) {

	int ret;
	pthread_mutexattr_t attr;

	ret = pthread_mutexattr_init(&attr);
	if (ret != 0) {
		return pthread_mutex_init(mutex, NULL);
	}

	ret = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
	if (ret == 0) {
		ret = pthread_mutex_init(mutex, &attr);
	}
	if (ret != 0) {
		ret = pthread_mutex_init(mutex, NULL);
	}

	pthread_mutexattr_destroy(&attr);
	return ret;
}

pthread_queue_t * pthread_queue_create(int length, size_t item_size) {

	int ret;
	pthread_queue_t * q;

	q = malloc(sizeof(pthread_queue_t));
	if (q == NULL) {
		goto out;
	}

	q->buffer = malloc(length * item_size);
	if (q->buffer == NULL) {
		goto free_q;
	}

	q->size = length;
	q->item_size = item_size;
	q->items = 0;
	q->in = 0;
	q->out = 0;

	ret = init_mutex_prio_inherit(&(q->mutex));
	if (ret != 0) {
		goto free_q_buffer;
	}

	ret = init_cond_clock_monotonic(&(q->cond_full));
	if (ret != 0) {
		goto destroy_mutex;
	}

	ret = init_cond_clock_monotonic(&(q->cond_empty));
	if (ret != 0) {
		goto destroy_cond;
	}

	return q;

destroy_cond:
	(void)pthread_cond_destroy(&(q->cond_full));

destroy_mutex:
	(void)pthread_mutex_destroy(&(q->mutex));

free_q_buffer:
	free(q->buffer);
	q->buffer = NULL;

free_q:
	free(q);
	q = NULL;

out:
	return q;
}

void pthread_queue_delete(pthread_queue_t * q) {

	if (q == NULL)
		return;

	free(q->buffer);
	free(q);

	return;
}

static inline int wait_slot_available(pthread_queue_t * queue, struct timespec * ts) {

	int ret;

	while (queue->items == queue->size) {

		if (ts != NULL) {
			ret = pthread_cond_timedwait(&(queue->cond_full), &(queue->mutex), ts);
		} else {
			ret = pthread_cond_wait(&(queue->cond_full), &(queue->mutex));
		}

		if (ret != 0 && ret != EINTR) {
			return PTHREAD_QUEUE_FULL;  // Timeout
		}
	}

	return PTHREAD_QUEUE_OK;
}

int pthread_queue_enqueue(pthread_queue_t * queue, const void * value, uint32_t timeout) {

	int ret;
	struct timespec ts;
	struct timespec * pts = NULL;

	/* Calculate timeout */
	if (timeout != CSP_MAX_TIMEOUT) {
		if (get_deadline(&ts, timeout) != 0) {
			return PTHREAD_QUEUE_ERROR;
		}
		pts = &ts;
	} else {
		pts = NULL;
	}

	/* Get queue lock */
	pthread_mutex_lock(&(queue->mutex));

	ret = wait_slot_available(queue, pts);
	if (ret == PTHREAD_QUEUE_OK) {
		/* Copy object from input buffer */
		memcpy((char *)queue->buffer + (queue->in * queue->item_size), value, queue->item_size);
		queue->items++;
		queue->in = (queue->in + 1) % queue->size;
	}

	pthread_mutex_unlock(&(queue->mutex));

	if (ret == PTHREAD_QUEUE_OK) {
		/* Notify blocked threads */
		pthread_cond_broadcast(&(queue->cond_empty));
	}

	return ret;
}

static inline int wait_item_available(pthread_queue_t * queue, struct timespec * ts) {

	int ret;

	while (queue->items == 0) {

		if (ts != NULL) {
			ret = pthread_cond_timedwait(&(queue->cond_empty), &(queue->mutex), ts);
		} else {
			ret = pthread_cond_wait(&(queue->cond_empty), &(queue->mutex));
		}

		if (ret != 0 && ret != EINTR) {
			return PTHREAD_QUEUE_EMPTY;  // Timeout
		}
	}

	return PTHREAD_QUEUE_OK;
}

int pthread_queue_dequeue(pthread_queue_t * queue, void * buf, uint32_t timeout) {

	int ret;
	struct timespec ts;
	struct timespec * pts;

	if(!queue){
		csp_print("csp not initialized\n");
		return PTHREAD_QUEUE_ERROR;
	}

	/* Calculate timeout */
	if (timeout != CSP_MAX_TIMEOUT) {
		if (get_deadline(&ts, timeout) != 0) {
			return PTHREAD_QUEUE_ERROR;
		}
		pts = &ts;
	} else {
		pts = NULL;
	}

	/* Get queue lock */
	pthread_mutex_lock(&(queue->mutex));

	ret = wait_item_available(queue, pts);
	if (ret == PTHREAD_QUEUE_OK) {
		/* Coby object to output buffer */
		memcpy(buf, (char *)queue->buffer + (queue->out * queue->item_size), queue->item_size);
		queue->items--;
		queue->out = (queue->out + 1) % queue->size;
	}

	pthread_mutex_unlock(&(queue->mutex));

	if (ret == PTHREAD_QUEUE_OK) {
		/* Notify blocked threads */
		pthread_cond_broadcast(&(queue->cond_full));
	}

	return ret;
}

int pthread_queue_items(pthread_queue_t * queue) {

	pthread_mutex_lock(&(queue->mutex));
	int items = queue->items;
	pthread_mutex_unlock(&(queue->mutex));

	return items;
}

int pthread_queue_free(pthread_queue_t * queue) {

	pthread_mutex_lock(&(queue->mutex));
	int free = queue->size - queue->items;
	pthread_mutex_unlock(&(queue->mutex));

	return free;
}

void pthread_queue_empty(pthread_queue_t * queue) {

	pthread_mutex_lock(&(queue->mutex));
	queue->items = 0;
	queue->in = 0;
	queue->out = 0;
	pthread_mutex_unlock(&(queue->mutex));
}
