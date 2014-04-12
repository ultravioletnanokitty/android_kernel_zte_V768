/* kernel/power/earlysuspend.c
 *
 * Copyright (C) 2005-2008 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/earlysuspend.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/syscalls.h> /* sys_sync */
#include <linux/wakelock.h>
#include <linux/workqueue.h>

#include "power.h"

enum {
	DEBUG_USER_STATE = 1U << 0,
	DEBUG_SUSPEND = 1U << 2,
};
//heweimao@wind-mobi.com begain
//merge 534042
#define DEBUG_BK	1
#if	DEBUG_BK
static int debug_mask = DEBUG_USER_STATE;//|DEBUG_SUSPEND;
#else
static int debug_mask = DEBUG_USER_STATE;//DEBUG_SUSPEND;
#endif
//heweimao@wind-mobi.com end

module_param_named(debug_mask, debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

static DEFINE_MUTEX(early_suspend_lock);
static LIST_HEAD(early_suspend_handlers);
static void early_suspend(struct work_struct *work);
static void late_resume(struct work_struct *work);
static DECLARE_WORK(early_suspend_work, early_suspend);
static DECLARE_WORK(late_resume_work, late_resume);
static DEFINE_SPINLOCK(state_lock);
enum {
	SUSPEND_REQUESTED = 0x1,
	SUSPENDED = 0x2,
	SUSPEND_REQUESTED_AND_SUSPENDED = SUSPEND_REQUESTED | SUSPENDED,
};
static int state;

void register_early_suspend(struct early_suspend *handler)
{
	struct list_head *pos;

	mutex_lock(&early_suspend_lock);
	list_for_each(pos, &early_suspend_handlers) {
		struct early_suspend *e;
		e = list_entry(pos, struct early_suspend, link);
		if (e->level > handler->level)
			break;
	}
	list_add_tail(&handler->link, pos);
	if ((state & SUSPENDED) && handler->suspend)
		handler->suspend(handler);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(register_early_suspend);

void unregister_early_suspend(struct early_suspend *handler)
{
	mutex_lock(&early_suspend_lock);
	list_del(&handler->link);
	mutex_unlock(&early_suspend_lock);
}
EXPORT_SYMBOL(unregister_early_suspend);

#if DEBUG_BK
struct cpu_workqueue_struct {

	spinlock_t lock;

	struct list_head worklist;
	wait_queue_head_t more_work;
	struct work_struct *current_work;

	struct workqueue_struct *wq;
	struct task_struct *thread;
} ____cacheline_aligned;

/*
 * The externally visible workqueue abstraction is an array of
 * per-CPU workqueues:
 */
struct workqueue_struct {
	struct cpu_workqueue_struct *cpu_wq;
	struct list_head list;
	const char *name;
	int singlethread;
	int freezeable;		/* Freeze threads during suspend */
	int rt;
#ifdef CONFIG_LOCKDEP
	struct lockdep_map lockdep_map;
#endif
};

unsigned int g_check_sd_cnt = 0,g_check_sd_cnt1=0,g_check_sd_cnt2=0,g_check_sd_cnt3=0;
EXPORT_SYMBOL(g_check_sd_cnt);
EXPORT_SYMBOL(g_check_sd_cnt1);
EXPORT_SYMBOL(g_check_sd_cnt2);
EXPORT_SYMBOL(g_check_sd_cnt3);
/*
unsigned int g_check_suspend_prepare = 0,g_check_suspend_prepare1 = 0,g_check_suspend_prepare2 = 0;
EXPORT_SYMBOL(g_check_suspend_prepare);
EXPORT_SYMBOL(g_check_suspend_prepare1);
EXPORT_SYMBOL(g_check_suspend_prepare2);
*/
#endif

static void early_suspend(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;

	pr_info("early_suspend ++++++++++\n");
	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED)
		state |= SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
		if (debug_mask & DEBUG_SUSPEND)
			pr_info("early_suspend: abort, state %d\n", state);
		mutex_unlock(&early_suspend_lock);
		goto abort;
	}

	if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: call handlers\n");
	list_for_each_entry(pos, &early_suspend_handlers, link) {
		pr_info("early_suspend: call handlers=%pf\n",pos->suspend);
		if (pos->suspend != NULL)
			pos->suspend(pos);
	}
	mutex_unlock(&early_suspend_lock);

	//if (debug_mask & DEBUG_SUSPEND)
		pr_info("early_suspend: sync\n");

 
	sys_sync();
#if DEBUG_BK	
	printk("xy mmc_blk_issue_rq=%d,%d,0x%x,%d\n",g_check_sd_cnt,g_check_sd_cnt1,g_check_sd_cnt2,g_check_sd_cnt3);
#endif
	
abort:
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPEND_REQUESTED_AND_SUSPENDED)
		wake_unlock(&main_wake_lock);
	spin_unlock_irqrestore(&state_lock, irqflags);
	pr_info("early_suspend ---------\n");
}

static void late_resume(struct work_struct *work)
{
	struct early_suspend *pos;
	unsigned long irqflags;
	int abort = 0;
	pr_info("late_resume ++++++++++\n");

	mutex_lock(&early_suspend_lock);
	spin_lock_irqsave(&state_lock, irqflags);
	if (state == SUSPENDED)
		state &= ~SUSPENDED;
	else
		abort = 1;
	spin_unlock_irqrestore(&state_lock, irqflags);

	if (abort) {
//heweimao@wind-mobi.com begain
//merge 534042
//		if (debug_mask & DEBUG_SUSPEND)
			pr_info("late_resume: abort, state %d\n", state);
//heweimao@wind-mobi.com end
		goto abort;
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: call handlers\n");
	list_for_each_entry_reverse(pos, &early_suspend_handlers, link)
	{
		pr_info("late_resume: call handlers=%pf\n",pos->resume);
		if (pos->resume != NULL)
			pos->resume(pos);
	}
	if (debug_mask & DEBUG_SUSPEND)
		pr_info("late_resume: done\n");
abort:
	mutex_unlock(&early_suspend_lock);
	pr_info("late_resume ---------\n");
}


void request_suspend_state(suspend_state_t new_state)
{
	unsigned long irqflags;
	int old_sleep;

	int ret=0x55aa;
	spin_lock_irqsave(&state_lock, irqflags);
	old_sleep = state & SUSPEND_REQUESTED;
	if (debug_mask & DEBUG_USER_STATE) {
		struct timespec ts;
		struct rtc_time tm;
		getnstimeofday(&ts);
		rtc_time_to_tm(ts.tv_sec, &tm);
		pr_info("request_suspend_state: %s (%d->%d) at %lld "
//heweimao@wind-mobi.com begain
//merge 534042
//			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n",
			"(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC),state=%d\n",
//heweimao@wind-mobi.com end
			new_state != PM_SUSPEND_ON ? "sleep" : "wakeup",
			requested_suspend_state, new_state,
			ktime_to_ns(ktime_get()),
			tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
//heweimao@wind-mobi.com begain
//merge 534042
//			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
			tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec,state);
//heweimao@wind-mobi.com end
	}
#if DEBUG_BK
	{
#if 0	
		struct cpu_workqueue_struct *cwq = suspend_work_queue->cpu_wq;
		struct work_struct *work;
		
		list_for_each_entry(work, &cwq->worklist, entry){
			work_func_t f = work->func;

			pr_info("Current in the queue, %pf\r\n", f);
		}
		pr_info("g_check_suspend_prepare = %d,%d,%d\n", g_check_suspend_prepare,g_check_suspend_prepare1,g_check_suspend_prepare2);
#endif		
		pr_info("g_check_sd_cnt=%d,%d,0x%x,%d\r\n",g_check_sd_cnt,g_check_sd_cnt1,g_check_sd_cnt2,g_check_sd_cnt3);
	}
#endif   
//heweimao@wind-mobi.com begain
//merge 534042
	if (!old_sleep && new_state != PM_SUSPEND_ON) {
		 //feixiaoping@wind-mobi.com begin
		 //Ensure state value is right whenever what happened.
	    if(state!=0)
    	{
       		state=0;
    	}
	    //feixiaoping@wind-mobi.com end 
		state |= SUSPEND_REQUESTED;
//		queue_work(suspend_work_queue, &early_suspend_work);
		ret=queue_work(suspend_work_queue, &early_suspend_work);
		pr_info("queue work --- early_suspend_work=%d\r\n",ret);
	} else if (old_sleep && new_state == PM_SUSPEND_ON) {
	    //feixiaoping@wind-mobi.com begin
	    //Ensure state value is right whenever what happened.
	    if(state!=3)
    	{
       		state=3; 
    	}
	    //feixiaoping@wind-mobi.com end 
		state &= ~SUSPEND_REQUESTED;
		wake_lock(&main_wake_lock);
//		queue_work(suspend_work_queue, &late_resume_work);
		ret=queue_work(suspend_work_queue, &late_resume_work);
		pr_info("queue work --- late_resume_work=%d\r\n",ret);
	}
//heweimao@wind-mobi.com end
	requested_suspend_state = new_state;
	spin_unlock_irqrestore(&state_lock, irqflags);
}

suspend_state_t get_suspend_state(void)
{
	return requested_suspend_state;
}
