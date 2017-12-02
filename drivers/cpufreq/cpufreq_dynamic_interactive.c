/*
 * drivers/cpufreq/cpufreq_dynamic_interactive.c
 *
 * Copyright (C) 2010 Google, Inc.
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
 * Author: Mike Chan (mike@android.com)
 *
 * Ported over to work with Newer Kernels Like 3.10/18
 * By Eliminater (eliminater74@gmail.com)  09/28/2016
 *
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/cpufreq.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/time.h>
#include <linux/timer.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <asm/cputime.h>

#define CREATE_TRACE_POINTS
#include <trace/events/cpufreq_dynamic_interactive.h>

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>

static atomic_t active_count = ATOMIC_INIT(0);

struct cpufreq_dynamic_interactive_cpuinfo {
	struct timer_list cpu_timer;
	int timer_idlecancel;
	u64 time_in_idle;
	u64 idle_exit_time;
	u64 timer_run_time;
	int idling;
	u64 target_set_time;
	u64 target_set_time_in_idle;
	struct cpufreq_policy *policy;
	struct cpufreq_frequency_table *freq_table;
	unsigned int target_freq;
	unsigned int floor_freq;
	u64 floor_validate_time;
	u64 hispeed_validate_time;
	int governor_enabled;
	unsigned int *load_history;
	unsigned int history_load_index;
	unsigned int total_avg_load;
	unsigned int total_load_history;
	unsigned int low_power_rate_history;
	unsigned int cpu_tune_value;
};

static DEFINE_PER_CPU(struct cpufreq_dynamic_interactive_cpuinfo, cpuinfo);

/* realtime thread handles frequency scaling */
static struct task_struct *speedchange_task;
static cpumask_t speedchange_cpumask;
static spinlock_t speedchange_cpumask_lock;

static struct workqueue_struct *tune_wq;
static struct work_struct tune_work;
static cpumask_t tune_cpumask;
static spinlock_t tune_cpumask_lock;

static unsigned int sampling_periods;
static unsigned int low_power_threshold;
static unsigned int hi_perf_threshold;
static unsigned int low_power_rate;
static enum tune_values {
	LOW_POWER_TUNE,
	DEFAULT_TUNE,
	HIGH_PERF_TUNE = 1,
} cur_tune_value;

#define MIN_GO_HISPEED_LOAD 70
#define DEFAULT_LOW_POWER_RATE 10
#define DEFAULT_SAMPLING_PERIODS 10
#define DEFAULT_HI_PERF_THRESHOLD 80
#define DEFAULT_LOW_POWER_THRESHOLD 35
#define MAX_MIN_SAMPLE_TIME (80 * USEC_PER_MSEC)

/* Hi speed to bump to from lo speed when load burst (default max) */
static u64 hispeed_freq;

/* Go to hi speed when CPU load at or above this value. */
#define DEFAULT_GO_HISPEED_LOAD 85
static unsigned long go_hispeed_load;
/*
 * The minimum amount of time to spend at a frequency before we can ramp down.
 */
#define DEFAULT_MIN_SAMPLE_TIME (20 * USEC_PER_MSEC)
static unsigned long min_sample_time;

/*
 * The sample rate of the timer used to increase frequency
 */
#define DEFAULT_TIMER_RATE (20 * USEC_PER_MSEC)
static unsigned long timer_rate;

/*
 * Wait this long before raising speed above hispeed, by default a single
 * timer interval.
 */
#define DEFAULT_ABOVE_HISPEED_DELAY DEFAULT_TIMER_RATE
static unsigned long above_hispeed_delay_val;

/*
 * Non-zero means longer-term speed boost active.
 */

static int boost_val;

static int cpufreq_governor_dynamic_interactive(struct cpufreq_policy *policy,
		unsigned int event);

#ifndef CONFIG_CPU_FREQ_DEFAULT_GOV_DYNAMIC_INTERACTIVE
static
#endif
struct cpufreq_governor cpufreq_gov_dynamic_interactive = {
	.name = "dyninteractive",
	.governor = cpufreq_governor_dynamic_interactive,
	.max_transition_latency = 10000000,
	.owner = THIS_MODULE,
};

static void cpufreq_dynamic_interactive_timer(unsigned long data)
{
	unsigned int delta_idle;
	unsigned int delta_time;
	int cpu_load;
	int load_since_change;
	u64 time_in_idle;
	u64 idle_exit_time;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, data);
	u64 now_idle;
	unsigned int new_freq, new_tune_value;
	unsigned int index, i, j;
	unsigned long flags;

	smp_rmb();

	if (!pcpu->governor_enabled)
		goto exit;

	/*
	 * Once pcpu->timer_run_time is updated to >= pcpu->idle_exit_time,
	 * this lets idle exit know the current idle time sample has
	 * been processed, and idle exit can generate a new sample and
	 * re-arm the timer.  This prevents a concurrent idle
	 * exit on that CPU from writing a new set of info at the same time
	 * the timer function runs (the timer function can't use that info
	 * until more time passes).
	 */
	time_in_idle = pcpu->time_in_idle;
	idle_exit_time = pcpu->idle_exit_time;
	now_idle = get_cpu_idle_time_us(data, &pcpu->timer_run_time);
	smp_wmb();

	/* If we raced with cancelling a timer, skip. */
	if (!idle_exit_time)
		goto exit;

	delta_idle = (unsigned int) cputime64_sub(now_idle, time_in_idle);
	delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
						  idle_exit_time);

	/*
	 * If timer ran less than 1ms after short-term sample started, retry.
	 */
	if (delta_time < 1000)
		goto rearm;

	if (delta_idle > delta_time)
		cpu_load = 0;
	else
		cpu_load = 100 * (delta_time - delta_idle) / delta_time;

	delta_idle = (unsigned int) cputime64_sub(now_idle,
						pcpu->target_set_time_in_idle);
	delta_time = (unsigned int) cputime64_sub(pcpu->timer_run_time,
						  pcpu->target_set_time);

	if ((delta_time == 0) || (delta_idle > delta_time))
		load_since_change = 0;
	else
		load_since_change =
			100 * (delta_time - delta_idle) / delta_time;

	/*
	 * Choose greater of short-term load (since last idle timer
	 * started or timer function re-armed itself) or long-term load
	 * (since last frequency change).
	 */
	if (load_since_change > cpu_load)
		cpu_load = load_since_change;
	pcpu->load_history[pcpu->history_load_index] = cpu_load;

	pcpu->total_load_history = 0;
	pcpu->low_power_rate_history = 0;

	/* compute average load across in & out sampling periods */
	for (i = 0, j = pcpu->history_load_index;
					i < sampling_periods; i++, j--) {
		pcpu->total_load_history += pcpu->load_history[j];
		if (low_power_rate < sampling_periods)
			if (i < low_power_rate)
				pcpu->low_power_rate_history
						  += pcpu->load_history[j];
		if (j == 0)
			j = sampling_periods;
	}

	/* return to first element if we're at the circular buffer's end */
	if (++pcpu->history_load_index == sampling_periods)
		pcpu->history_load_index = 0;
	else if (unlikely(pcpu->history_load_index > sampling_periods)) {
		/*
		 * This not supposed to happen.
		 * If we got here - means something is wrong.
		 */
		pr_err("%s: have gone beyond allocated buffer of history!\n",
					__func__);
		pcpu->history_load_index = 0;
	}

	pcpu->total_avg_load = pcpu->total_load_history / sampling_periods;

	if (pcpu->total_avg_load > hi_perf_threshold)
		new_tune_value = HIGH_PERF_TUNE;
	else if (pcpu->total_avg_load < low_power_threshold)
		new_tune_value = LOW_POWER_TUNE;
	else
		new_tune_value = DEFAULT_TUNE;

	if (new_tune_value != cur_tune_value)
		if ((pcpu->cpu_tune_value != new_tune_value)
			&& ((new_tune_value == HIGH_PERF_TUNE)
				|| (new_tune_value == LOW_POWER_TUNE))) {
			spin_lock_irqsave(&tune_cpumask_lock, flags);
			cpumask_set_cpu(data, &tune_cpumask);
			spin_unlock_irqrestore(&tune_cpumask_lock, flags);
			queue_work(tune_wq, &tune_work);
		}
	pcpu->cpu_tune_value = new_tune_value;

	if (cur_tune_value == LOW_POWER_TUNE) {
		if (low_power_rate < sampling_periods)
			cpu_load = pcpu->low_power_rate_history
						/ low_power_rate;
		else
			cpu_load = pcpu->total_avg_load;
	}

	if (cpu_load >= go_hispeed_load || boost_val) {
		if (pcpu->target_freq <= pcpu->policy->min) {
			new_freq = hispeed_freq;
		} else {
			new_freq = pcpu->policy->max * cpu_load / 100;

			if (new_freq < hispeed_freq)
				new_freq = hispeed_freq;

			if (pcpu->target_freq == hispeed_freq &&
			    new_freq > hispeed_freq &&
			    cputime64_sub(pcpu->timer_run_time,
					  pcpu->hispeed_validate_time)
			    < above_hispeed_delay_val) {
				trace_cpufreq_dynamic_interactive_notyet(data, cpu_load,
								 pcpu->target_freq,
								 new_freq);
				goto rearm;
			}
		}
	} else {
		new_freq = pcpu->policy->max * cpu_load / 100;
	}

	if (new_freq <= hispeed_freq)
		pcpu->hispeed_validate_time = pcpu->timer_run_time;

	if (cpufreq_frequency_table_target(pcpu->policy, pcpu->freq_table,
					   new_freq, CPUFREQ_RELATION_H,
					   &index)) {
		pr_warn_once("timer %d: cpufreq_frequency_table_target error\n",
			     (int) data);
		goto rearm;
	}

	new_freq = pcpu->freq_table[index].frequency;

	/*
	 * Do not scale below floor_freq unless we have been at or above the
	 * floor frequency for the minimum sample time since last validated.
	 */
	if (new_freq < pcpu->floor_freq) {
		if (cputime64_sub(pcpu->timer_run_time,
				  pcpu->floor_validate_time)
		    < min_sample_time) {
			trace_cpufreq_dynamic_interactive_notyet(data, cpu_load,
					 pcpu->target_freq, new_freq);
			goto rearm;
		}
	}

	pcpu->floor_freq = new_freq;
	pcpu->floor_validate_time = pcpu->timer_run_time;

	if (pcpu->target_freq == new_freq) {
		trace_cpufreq_dynamic_interactive_already(data, cpu_load,
						  pcpu->target_freq, new_freq);
		goto rearm_if_notmax;
	}

	trace_cpufreq_dynamic_interactive_target(data, cpu_load, pcpu->target_freq,
					 new_freq);
	pcpu->target_set_time_in_idle = now_idle;
	pcpu->target_set_time = pcpu->timer_run_time;

	pcpu->target_freq = new_freq;
    spin_lock_irqsave(&speedchange_cpumask_lock, flags);
    cpumask_set_cpu(data, &speedchange_cpumask);
    spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);
    wake_up_process(speedchange_task);

rearm_if_notmax:
	/*
	 * Already set max speed and don't see a need to change that,
	 * wait until next idle to re-evaluate, don't need timer.
	 */
	if (pcpu->target_freq == pcpu->policy->max)
		goto exit;

rearm:
	if (!timer_pending(&pcpu->cpu_timer)) {
		/*
		 * If already at min: if that CPU is idle, don't set timer.
		 * Else cancel the timer if that CPU goes idle.  We don't
		 * need to re-evaluate speed until the next idle exit.
		 */
		if (pcpu->target_freq == pcpu->policy->min) {
			smp_rmb();

			if (pcpu->idling)
				goto exit;

			pcpu->timer_idlecancel = 1;
		}

		pcpu->time_in_idle = get_cpu_idle_time_us(
			data, &pcpu->idle_exit_time);
		mod_timer(&pcpu->cpu_timer,
			  jiffies + usecs_to_jiffies(timer_rate));
	}

exit:
	return;
}

static void cpufreq_dynamic_interactive_tune(struct work_struct *work)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu;

	unsigned int max_total_avg_load = 0;
	unsigned int index;

	spin_lock_irqsave(&tune_cpumask_lock, flags);
	tmp_mask = tune_cpumask;
	cpumask_clear(&tune_cpumask);
	spin_unlock_irqrestore(&tune_cpumask_lock, flags);

	for_each_cpu(cpu, &tmp_mask) {
		unsigned int j;

		pcpu = &per_cpu(cpuinfo, cpu);
		smp_rmb();

		if (!pcpu->governor_enabled)
			continue;

		for_each_cpu(j, pcpu->policy->cpus) {
			struct cpufreq_dynamic_interactive_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

			if (pjcpu->total_avg_load > max_total_avg_load)
				max_total_avg_load = pjcpu->total_avg_load;
		}

		if ((max_total_avg_load > hi_perf_threshold)
				&& (cur_tune_value != HIGH_PERF_TUNE)) {
				cur_tune_value = HIGH_PERF_TUNE;
				go_hispeed_load = MIN_GO_HISPEED_LOAD;
				min_sample_time = MAX_MIN_SAMPLE_TIME;
				hispeed_freq = pcpu->policy->max;
		} else if ((max_total_avg_load < low_power_threshold)
				&& (cur_tune_value != LOW_POWER_TUNE)) {
			/* Boost down the performance */
				go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
				min_sample_time = DEFAULT_MIN_SAMPLE_TIME;
				cpufreq_frequency_table_target(pcpu->policy,
					pcpu->freq_table, pcpu->policy->min,
					CPUFREQ_RELATION_H, &index);
				hispeed_freq =
					pcpu->freq_table[index+1].frequency;
				cur_tune_value = LOW_POWER_TUNE;
		}
	}
}

static void cpufreq_dynamic_interactive_idle_start(void)
{
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());
	int pending;

	if (!pcpu->governor_enabled)
		return;

	pcpu->idling = 1;
	smp_wmb();
	pending = timer_pending(&pcpu->cpu_timer);

	if (pcpu->target_freq != pcpu->policy->min) {
#ifdef CONFIG_SMP
		/*
		 * Entering idle while not at lowest speed.  On some
		 * platforms this can hold the other CPU(s) at that speed
		 * even though the CPU is idle. Set a timer to re-evaluate
		 * speed so this idle CPU doesn't hold the other CPUs above
		 * min indefinitely.  This should probably be a quirk of
		 * the CPUFreq driver.
		 */
		if (!pending) {
			pcpu->time_in_idle = get_cpu_idle_time_us(
				smp_processor_id(), &pcpu->idle_exit_time);
			pcpu->timer_idlecancel = 0;
			mod_timer(&pcpu->cpu_timer,
				  jiffies + usecs_to_jiffies(timer_rate));
		}
#endif
	} else {
		/*
		 * If at min speed and entering idle after load has
		 * already been evaluated, and a timer has been set just in
		 * case the CPU suddenly goes busy, cancel that timer.  The
		 * CPU didn't go busy; we'll recheck things upon idle exit.
		 */
		if (pending && pcpu->timer_idlecancel) {
			del_timer(&pcpu->cpu_timer);
			/*
			 * Ensure last timer run time is after current idle
			 * sample start time, so next idle exit will always
			 * start a new idle sampling period.
			 */
			pcpu->idle_exit_time = 0;
			pcpu->timer_idlecancel = 0;
		}
	}

}

static void cpufreq_dynamic_interactive_idle_end(void)
{
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu =
		&per_cpu(cpuinfo, smp_processor_id());

    if (!pcpu->governor_enabled)
        return;
    
	pcpu->idling = 0;
	smp_wmb();

	/*
	 * Arm the timer for 1-2 ticks later if not already, and if the timer
	 * function has already processed the previous load sampling
	 * interval.  (If the timer is not pending but has not processed
	 * the previous interval, it is probably racing with us on another
	 * CPU.  Let it compute load based on the previous sample and then
	 * re-arm the timer for another interval when it's done, rather
	 * than updating the interval start time to be "now", which doesn't
	 * give the timer function enough time to make a decision on this
	 * run.)
	 */
	if (timer_pending(&pcpu->cpu_timer) == 0 &&
	    pcpu->timer_run_time >= pcpu->idle_exit_time &&
	    pcpu->governor_enabled) {
		pcpu->time_in_idle =
			get_cpu_idle_time_us(smp_processor_id(),
					     &pcpu->idle_exit_time);
		pcpu->timer_idlecancel = 0;
		mod_timer(&pcpu->cpu_timer,
			  jiffies + usecs_to_jiffies(timer_rate));
	}

}

static int cpufreq_dynamic_interactive_speedchange_task(void *data)
{
	unsigned int cpu;
	cpumask_t tmp_mask;
	unsigned long flags;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu;

	while (1) {
		set_current_state(TASK_INTERRUPTIBLE);
		spin_lock_irqsave(&speedchange_cpumask_lock, flags);

		if (cpumask_empty(&speedchange_cpumask)) {
			spin_unlock_irqrestore(&speedchange_cpumask_lock,
                       flags);
			schedule();

			if (kthread_should_stop())
				break;

			spin_lock_irqsave(&speedchange_cpumask_lock, flags);
		}

		set_current_state(TASK_RUNNING);
		tmp_mask = speedchange_cpumask;
		cpumask_clear(&speedchange_cpumask);
		spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

		for_each_cpu(cpu, &tmp_mask) {
			unsigned int j;
			unsigned int max_freq = 0;

			pcpu = &per_cpu(cpuinfo, cpu);
			smp_rmb();

			if (!pcpu->governor_enabled)
				continue;

			for_each_cpu(j, pcpu->policy->cpus) {
				struct cpufreq_dynamic_interactive_cpuinfo *pjcpu =
					&per_cpu(cpuinfo, j);

				if (pjcpu->target_freq > max_freq)
					max_freq = pjcpu->target_freq;
			}

			if (max_freq != pcpu->policy->cur)
				__cpufreq_driver_target(pcpu->policy,
							max_freq,
							CPUFREQ_RELATION_H);
			trace_cpufreq_dynamic_interactive_setspeed(cpu,
                             pcpu->target_freq,
						     pcpu->policy->cur);
		}
	}

	return 0;
}

static void cpufreq_dynamic_interactive_boost(void)
{
	int i;
	int anyboost = 0;
	unsigned long flags;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu;

	spin_lock_irqsave(&speedchange_cpumask_lock, flags);

	for_each_online_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);

		if (pcpu->target_freq < hispeed_freq) {
			pcpu->target_freq = hispeed_freq;
			cpumask_set_cpu(i, &speedchange_cpumask);
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time_us(i, &pcpu->target_set_time);
			pcpu->hispeed_validate_time = pcpu->target_set_time;
			anyboost = 1;
		}

		/*
		 * Set floor freq and (re)start timer for when last
		 * validated.
		 */

		pcpu->floor_freq = hispeed_freq;
		pcpu->floor_validate_time = ktime_to_us(ktime_get());
	}

	spin_unlock_irqrestore(&speedchange_cpumask_lock, flags);

	if (anyboost)
		wake_up_process(speedchange_task);
}

static ssize_t show_hispeed_freq(struct kobject *kobj,
				 struct attribute *attr, char *buf)
{
	return sprintf(buf, "%llu\n", hispeed_freq);
}

static ssize_t store_hispeed_freq(struct kobject *kobj,
				  struct attribute *attr, const char *buf,
				  size_t count)
{
	int ret;
	u64 val;

	ret = kstrtoull(buf, 0, &val);
	if (ret < 0)
		return ret;
	hispeed_freq = val;
	return count;
}

static struct global_attr hispeed_freq_attr = __ATTR(hispeed_freq, 0644,
		show_hispeed_freq, store_hispeed_freq);


static ssize_t show_go_hispeed_load(struct kobject *kobj,
				     struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", go_hispeed_load);
}

static ssize_t store_go_hispeed_load(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	go_hispeed_load = val;
	return count;
}

static struct global_attr go_hispeed_load_attr = __ATTR(go_hispeed_load, 0644,
		show_go_hispeed_load, store_go_hispeed_load);

static ssize_t show_min_sample_time(struct kobject *kobj,
				struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", min_sample_time);
}

static ssize_t store_min_sample_time(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	min_sample_time = val;
	return count;
}

static struct global_attr min_sample_time_attr = __ATTR(min_sample_time, 0644,
		show_min_sample_time, store_min_sample_time);

static ssize_t show_above_hispeed_delay(struct kobject *kobj,
					struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", above_hispeed_delay_val);
}

static ssize_t store_above_hispeed_delay(struct kobject *kobj,
					 struct attribute *attr,
					 const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	above_hispeed_delay_val = val;
	return count;
}

define_one_global_rw(above_hispeed_delay);

static ssize_t show_timer_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%lu\n", timer_rate);
}

static ssize_t store_timer_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	timer_rate = val;
	return count;
}

static struct global_attr timer_rate_attr = __ATTR(timer_rate, 0644,
		show_timer_rate, store_timer_rate);

static ssize_t show_boost(struct kobject *kobj, struct attribute *attr,
			  char *buf)
{
	return sprintf(buf, "%d\n", boost_val);
}

static ssize_t store_boost(struct kobject *kobj, struct attribute *attr,
			   const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	boost_val = val;

	if (boost_val) {
		trace_cpufreq_dynamic_interactive_boost("on");
		cpufreq_dynamic_interactive_boost();
	} else {
		trace_cpufreq_dynamic_interactive_unboost("off");
	}

	return count;
}

define_one_global_rw(boost);

static ssize_t store_boostpulse(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;

	trace_cpufreq_dynamic_interactive_boost("pulse");
	cpufreq_dynamic_interactive_boost();
	return count;
}

static struct global_attr boostpulse =
	__ATTR(boostpulse, 0200, NULL, store_boostpulse);

static ssize_t show_sampling_periods(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", sampling_periods);
}

static ssize_t store_sampling_periods(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned int val, t_mask = 0;
	unsigned int *temp;
	unsigned int j, i;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu;

	ret = sscanf(buf, "%u", &val);
	if (ret != 1)
		return ret;

	if (val == sampling_periods)
		return count;

	for_each_present_cpu(j) {
		pcpu = &per_cpu(cpuinfo, j);
		ret = del_timer_sync(&pcpu->cpu_timer);
		if (ret)
			t_mask |= BIT(j);
		pcpu->history_load_index = 0;

		temp = kmalloc((sizeof(unsigned int) * val), GFP_KERNEL);
		if (!temp) {
			pr_err("%s:can't allocate memory for history\n",
					__func__);
			count = -ENOMEM;
			goto out;
		}
		memcpy(temp, pcpu->load_history,
			(min(sampling_periods, val) * sizeof(unsigned int)));
		if (val > sampling_periods)
			for (i = sampling_periods; i < val; i++)
				temp[i] = 50;

		kfree(pcpu->load_history);
		pcpu->load_history = temp;
	}

out:
	if (!(count < 0 && val > sampling_periods))
		sampling_periods = val;

	for_each_online_cpu(j) {
		pcpu = &per_cpu(cpuinfo, j);
		if (t_mask & BIT(j))
			mod_timer(&pcpu->cpu_timer,
				jiffies + usecs_to_jiffies(timer_rate));
	}

	return count;
}

static struct global_attr sampling_periods_attr = __ATTR(sampling_periods,
			0644, show_sampling_periods, store_sampling_periods);

static ssize_t show_hi_perf_threshold(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", hi_perf_threshold);
}

static ssize_t store_hi_perf_threshold(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	hi_perf_threshold = val;
	return count;
}

static struct global_attr hi_perf_threshold_attr = __ATTR(hi_perf_threshold,
			0644, show_hi_perf_threshold, store_hi_perf_threshold);


static ssize_t show_low_power_threshold(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", low_power_threshold);
}

static ssize_t store_low_power_threshold(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	low_power_threshold = val;
	return count;
}

static struct global_attr low_power_threshold_attr = __ATTR(low_power_threshold,
		     0644, show_low_power_threshold, store_low_power_threshold);

static ssize_t show_low_power_rate(struct kobject *kobj,
			struct attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", low_power_rate);
}

static ssize_t store_low_power_rate(struct kobject *kobj,
			struct attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long val;

	ret = kstrtoul(buf, 0, &val);
	if (ret < 0)
		return ret;
	low_power_rate = val;
	return count;
}

static struct global_attr low_power_rate_attr = __ATTR(low_power_rate,
		     0644, show_low_power_rate, store_low_power_rate);


static struct attribute *dynamic_interactive_attributes[] = {
	&hispeed_freq_attr.attr,
	&go_hispeed_load_attr.attr,
	&above_hispeed_delay.attr,
	&min_sample_time_attr.attr,
	&timer_rate_attr.attr,
	&boost.attr,
	&boostpulse.attr,
	&low_power_threshold_attr.attr,
	&hi_perf_threshold_attr.attr,
	&sampling_periods_attr.attr,
	&low_power_rate_attr.attr,
	NULL,
};

static struct attribute_group dynamic_interactive_attr_group = {
	.attrs = dynamic_interactive_attributes,
	.name = "dynamic_interactive",
};

static int cpufreq_dynamic_interactive_idle_notifier(struct notifier_block *nb,
                                             unsigned long val,
                                             void *data)
{
	switch (val) {
        case IDLE_START:
            cpufreq_dynamic_interactive_idle_start();
            break;
        case IDLE_END:
            cpufreq_dynamic_interactive_idle_end();
            break;
	}
    
	return 0;
}

static struct notifier_block cpufreq_dynamic_interactive_idle_nb = {
	.notifier_call = cpufreq_dynamic_interactive_idle_notifier,
};

static int cpufreq_governor_dynamic_interactive(struct cpufreq_policy *policy,
		unsigned int event)
{
	int rc;
	unsigned int j, i;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu;
	struct cpufreq_frequency_table *freq_table;

	switch (event) {
	case CPUFREQ_GOV_START:
		if (!cpu_online(policy->cpu))
			return -EINVAL;

		freq_table =
			cpufreq_frequency_get_table(policy->cpu);

		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->policy = policy;
			pcpu->target_freq = policy->cur;
			pcpu->freq_table = freq_table;
			pcpu->target_set_time_in_idle =
				get_cpu_idle_time_us(j,
					     &pcpu->target_set_time);
			pcpu->floor_freq = pcpu->target_freq;
			pcpu->floor_validate_time =
				pcpu->target_set_time;
			pcpu->hispeed_validate_time =
				pcpu->target_set_time;
			pcpu->governor_enabled = 1;
			pcpu->load_history = kmalloc(
				(sizeof(unsigned int) * sampling_periods),
				 GFP_KERNEL);
			if (!pcpu->load_history)
				return -ENOMEM;
			for (i = 0; i < sampling_periods; i++)
				pcpu->load_history[i] = 0;
			pcpu->history_load_index = 0;
			smp_wmb();
		}

		if (!hispeed_freq)
			hispeed_freq = policy->max;

		/*
		 * Do not register the idle hook and create sysfs
		 * entries if we have already done so.
		 */
		if (atomic_inc_return(&active_count) > 1)
			return 0;

		rc = sysfs_create_group(cpufreq_global_kobject,
				&dynamic_interactive_attr_group);
		if (rc)
			return rc;
            
        idle_notifier_register(&cpufreq_dynamic_interactive_idle_nb);
		break;

	case CPUFREQ_GOV_STOP:
		for_each_cpu(j, policy->cpus) {
			pcpu = &per_cpu(cpuinfo, j);
			pcpu->governor_enabled = 0;
			smp_wmb();
			del_timer_sync(&pcpu->cpu_timer);

			/*
			 * Reset idle exit time since we may cancel the timer
			 * before it can run after the last idle exit time,
			 * to avoid tripping the check in idle exit for a timer
			 * that is trying to run.
			 */
			pcpu->idle_exit_time = 0;
			kfree(pcpu->load_history);
		}

		flush_work(&tune_work);

		if (atomic_dec_return(&active_count) > 0)
			return 0;
            
        idle_notifier_unregister(&cpufreq_dynamic_interactive_idle_nb);
		sysfs_remove_group(cpufreq_global_kobject,
				&dynamic_interactive_attr_group);

		break;

	case CPUFREQ_GOV_LIMITS:
		if (policy->max < policy->cur)
			__cpufreq_driver_target(policy,
					policy->max, CPUFREQ_RELATION_H);
		else if (policy->min > policy->cur)
			__cpufreq_driver_target(policy,
					policy->min, CPUFREQ_RELATION_L);
		break;
	}
	return 0;
}

static int __init cpufreq_dynamic_interactive_init(void)
{
	unsigned int i;
	struct cpufreq_dynamic_interactive_cpuinfo *pcpu;
	struct sched_param param = { .sched_priority = MAX_RT_PRIO-1 };

	go_hispeed_load = DEFAULT_GO_HISPEED_LOAD;
	min_sample_time = DEFAULT_MIN_SAMPLE_TIME;
	above_hispeed_delay_val = DEFAULT_ABOVE_HISPEED_DELAY;
	timer_rate = DEFAULT_TIMER_RATE;

	sampling_periods = DEFAULT_SAMPLING_PERIODS;
	hi_perf_threshold = DEFAULT_HI_PERF_THRESHOLD;
	low_power_threshold = DEFAULT_LOW_POWER_THRESHOLD;
	low_power_rate = DEFAULT_LOW_POWER_RATE;
	cur_tune_value = DEFAULT_TUNE;
	/* Initalize per-cpu timers */
	for_each_possible_cpu(i) {
		pcpu = &per_cpu(cpuinfo, i);
		init_timer(&pcpu->cpu_timer);
		pcpu->cpu_timer.function = cpufreq_dynamic_interactive_timer;
		pcpu->cpu_timer.data = i;
		pcpu->cpu_tune_value = DEFAULT_TUNE;
	}

    spin_lock_init(&tune_cpumask_lock);
    spin_lock_init(&speedchange_cpumask_lock);

    speedchange_task =
        kthread_create(cpufreq_dynamic_interactive_speedchange_task, NULL,
                "cfdynamic_interactive");
    if (IS_ERR(speedchange_task))
        return PTR_ERR(speedchange_task);        

	sched_setscheduler_nocheck(speedchange_task, SCHED_FIFO, &param);
	get_task_struct(speedchange_task);

	tune_wq = alloc_workqueue("knteractive_tune", 0, 1);

	INIT_WORK(&tune_work,
		  cpufreq_dynamic_interactive_tune);
    
    /* NB: wake up so the thread does not look hung to the freezer */
    wake_up_process(speedchange_task);

	return cpufreq_register_governor(&cpufreq_gov_dynamic_interactive);
}

#ifdef CONFIG_CPU_FREQ_DEFAULT_GOV_dynamic_interactive
fs_initcall(cpufreq_dynamic_interactive_init);
#else
module_init(cpufreq_dynamic_interactive_init);
#endif

static void __exit cpufreq_dynamic_interactive_exit(void)
{
	cpufreq_unregister_governor(&cpufreq_gov_dynamic_interactive);
	kthread_stop(speedchange_task);
	put_task_struct(speedchange_task);
	destroy_workqueue(tune_wq);
}

module_exit(cpufreq_dynamic_interactive_exit);

MODULE_AUTHOR("Mike Chan <mike@android.com>");
MODULE_DESCRIPTION("'cpufreq_dynamic_interactive' - A cpufreq governor for "
	"Latency sensitive workloads");
MODULE_LICENSE("GPL");
