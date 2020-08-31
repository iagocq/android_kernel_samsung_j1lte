/*
 * This module exposes the interface to kernel space for specifying
 * QoS dependencies.  It provides infrastructure for registration of:
 *
 * Dependents on a QoS value : register requests
 * Watchers of QoS value : get notified when target QoS value changes
 *
 * This QoS design is best effort based.  Dependents register their QoS needs.
 * Watchers register to keep track of the current QoS needs of the system.
 *
 * There are 3 basic classes of QoS parameter: latency, timeout, throughput
 * each have defined units:
 * latency: usec
 * timeout: usec <-- currently not used.
 * throughput: kbs (kilo byte / sec)
 *
 * There are lists of pm_qos_objects each one wrapping requests, notifiers
 *
 * User mode requests on a QOS parameter register themselves to the
 * subsystem by opening the device node /dev/... and writing there request to
 * the node.  As long as the process holds a file handle open to the node the
 * client continues to be accounted for.  Upon file release the usermode
 * request is removed and a new qos target is computed.  This way when the
 * request that the application has is cleaned up when closes the file
 * pointer or exits the pm_qos_object will get an opportunity to clean up.
 *
 * Mark Gross <mgross@linux.intel.com>
 */

/*#define DEBUG*/

#include <linux/debugfs.h>
#include <linux/pm_qos.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/time.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/init.h>
#include <linux/kernel.h>

#include <linux/uaccess.h>
#include <linux/export.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <trace/events/power.h>

DEFINE_SPINLOCK(pm_qos_lock);

static struct pm_qos_object null_pm_qos;

static BLOCKING_NOTIFIER_HEAD(cpu_dma_lat_notifier);
static struct pm_qos_constraints cpu_dma_constraints = {
	.list = PLIST_HEAD_INIT(cpu_dma_constraints.list),
	.target_value = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE,
	.default_value = PM_QOS_CPU_DMA_LAT_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
	.notifiers = &cpu_dma_lat_notifier,
};
static struct pm_qos_object cpu_dma_pm_qos = {
	.constraints = &cpu_dma_constraints,
	.name = "cpu_dma_latency",
};

static BLOCKING_NOTIFIER_HEAD(network_lat_notifier);
static struct pm_qos_constraints network_lat_constraints = {
	.list = PLIST_HEAD_INIT(network_lat_constraints.list),
	.target_value = PM_QOS_NETWORK_LAT_DEFAULT_VALUE,
	.default_value = PM_QOS_NETWORK_LAT_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
	.notifiers = &network_lat_notifier,
};
static struct pm_qos_object network_lat_pm_qos = {
	.constraints = &network_lat_constraints,
	.name = "network_latency",
};


static BLOCKING_NOTIFIER_HEAD(network_throughput_notifier);
static struct pm_qos_constraints network_tput_constraints = {
	.list = PLIST_HEAD_INIT(network_tput_constraints.list),
	.target_value = PM_QOS_NETWORK_THROUGHPUT_DEFAULT_VALUE,
	.default_value = PM_QOS_NETWORK_THROUGHPUT_DEFAULT_VALUE,
	.type = PM_QOS_MAX,
	.notifiers = &network_throughput_notifier,
};
static struct pm_qos_object network_throughput_pm_qos = {
	.constraints = &network_tput_constraints,
	.name = "network_throughput",
};


static BLOCKING_NOTIFIER_HEAD(cpuidle_block_notifier);
static struct pm_qos_constraints cpuidle_block_constraints = {
	.list = PLIST_HEAD_INIT(cpuidle_block_constraints.list),
	.target_value = PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE,
	.default_value = PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
	.notifiers = &cpuidle_block_notifier,
};
static struct pm_qos_object cpuidle_block_pm_qos = {
	.constraints = &cpuidle_block_constraints,
	.name = "cpuidle_block",
};

static BLOCKING_NOTIFIER_HEAD(cpu_freq_min_notifier);
static struct pm_qos_constraints cpu_freq_min_constraints = {
	.list = PLIST_HEAD_INIT(cpu_freq_min_constraints.list),
	.notifiers = &cpu_freq_min_notifier,
	.default_value = PM_QOS_DEFAULT_VALUE,
	.target_value = PM_QOS_DEFAULT_VALUE,
	.type = PM_QOS_MAX,
};

static struct pm_qos_object cpu_freq_min_pm_qos = {
	.constraints = &cpu_freq_min_constraints,
	.name = "cpu_freq_min",
};

static BLOCKING_NOTIFIER_HEAD(cpu_freq_max_notifier);
static struct pm_qos_constraints cpu_freq_max_constraints = {
	.list = PLIST_HEAD_INIT(cpu_freq_max_constraints.list),
	.notifiers = &cpu_freq_max_notifier,
	.default_value = INT_MAX,
	.target_value = PM_QOS_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
};

static struct pm_qos_object cpu_freq_max_pm_qos = {
	.constraints = &cpu_freq_max_constraints,
	.name = "cpu_freq_max",
};

static BLOCKING_NOTIFIER_HEAD(cpu_num_min_notifier);
static struct pm_qos_constraints cpu_num_min_constraints = {
	.list = PLIST_HEAD_INIT(cpu_num_min_constraints.list),
	.notifiers = &cpu_num_min_notifier,
	.default_value = PM_QOS_DEFAULT_VALUE,
	.target_value = PM_QOS_DEFAULT_VALUE,
	.type = PM_QOS_MAX,
};

static struct pm_qos_object cpu_num_min_pm_qos = {
	.constraints = &cpu_num_min_constraints,
	.name = "cpu_num_min",
};

static BLOCKING_NOTIFIER_HEAD(cpu_num_max_notifier);
static struct pm_qos_constraints cpu_num_max_constraints = {
	.list = PLIST_HEAD_INIT(cpu_num_max_constraints.list),
	.notifiers = &cpu_num_max_notifier,
	.default_value = INT_MAX,
	.target_value = PM_QOS_DEFAULT_VALUE,
	.type = PM_QOS_MIN,
};

static struct pm_qos_object cpu_num_max_pm_qos = {
	.constraints = &cpu_num_max_constraints,
	.name = "cpu_num_max",
};

static BLOCKING_NOTIFIER_HEAD(ddr_devfreq_min_notifier);
static struct pm_qos_constraints ddr_devfreq_min_constraints = {
	.list = PLIST_HEAD_INIT(ddr_devfreq_min_constraints.list),
	.target_value = PM_QOS_DEFAULT_VALUE,
	.default_value = 0,
	.type = PM_QOS_MAX,
	.notifiers = &ddr_devfreq_min_notifier,
};
static struct pm_qos_object ddr_devfreq_min_pm_qos = {
	.constraints = &ddr_devfreq_min_constraints,
	.name = "ddr_devfreq_min",
};

static BLOCKING_NOTIFIER_HEAD(ddr_devfreq_max_notifier);
static struct pm_qos_constraints ddr_devfreq_max_constraints = {
	.list = PLIST_HEAD_INIT(ddr_devfreq_max_constraints.list),
	.target_value = PM_QOS_DEFAULT_VALUE,
	.default_value = INT_MAX,
	.type = PM_QOS_MIN,
	.notifiers = &ddr_devfreq_max_notifier,
};
static struct pm_qos_object ddr_devfreq_max_pm_qos = {
	.constraints = &ddr_devfreq_max_constraints,
	.name = "ddr_devfreq_max",
};

static BLOCKING_NOTIFIER_HEAD(axi_min_notifier);
static struct pm_qos_constraints axi_min_constraints = {
	.list = PLIST_HEAD_INIT(axi_min_constraints.list),
	.notifiers = &axi_min_notifier,
	.default_value = PM_QOS_DEFAULT_VALUE,
	.target_value = PM_QOS_DEFAULT_VALUE,
	.type = PM_QOS_MAX,
};

static struct pm_qos_object axi_min_pm_qos = {
	.constraints = &axi_min_constraints,
	.name = "axi_min",
};

static BLOCKING_NOTIFIER_HEAD(vpu_devfreq_min_notifier);
static struct pm_qos_constraints vpu_devfreq_min_constraints = {
	.list = PLIST_HEAD_INIT(vpu_devfreq_min_constraints.list),
	.target_value = PM_QOS_DEFAULT_VALUE,
	.default_value = PM_QOS_DEFAULT_VALUE,
	.type = PM_QOS_MAX,
	.notifiers = &vpu_devfreq_min_notifier,
};
static struct pm_qos_object vpu_devfreq_min_pm_qos = {
	.constraints = &vpu_devfreq_min_constraints,
	.name = "vpu_devfreq_min",
};

static BLOCKING_NOTIFIER_HEAD(vpu_devfreq_max_notifier);
static struct pm_qos_constraints vpu_devfreq_max_constraints = {
	.list = PLIST_HEAD_INIT(vpu_devfreq_max_constraints.list),
	.target_value = PM_QOS_DEFAULT_VALUE,
	.default_value = INT_MAX,
	.type = PM_QOS_MIN,
	.notifiers = &vpu_devfreq_max_notifier,
};
static struct pm_qos_object vpu_devfreq_max_pm_qos = {
	.constraints = &vpu_devfreq_max_constraints,
	.name = "vpu_devfreq_max",
};

#define DECLARE_GPU_NOTIFIER(CORE, MINMAX, TYPE) \
	static BLOCKING_NOTIFIER_HEAD(gpu_freq_##CORE##_##MINMAX##_notifier); \
	static struct pm_qos_constraints gpu_freq_##CORE##_##MINMAX##_constraints = { \
		.list = PLIST_HEAD_INIT(gpu_freq_##CORE##_##MINMAX##_constraints.list), \
		.notifiers = &gpu_freq_##CORE##_##MINMAX##_notifier, \
		.default_value = 0, \
		.target_value = 0, \
		.type = TYPE, \
	}; \
	static struct pm_qos_object gpu_freq_##CORE##_##MINMAX##_pm_qos = { \
		.constraints = &gpu_freq_##CORE##_##MINMAX##_constraints, \
		.name = "gpu_freq_"#CORE"_"#MINMAX, \
	};

DECLARE_GPU_NOTIFIER(3d, min, PM_QOS_MAX);
DECLARE_GPU_NOTIFIER(2d, min, PM_QOS_MAX);
DECLARE_GPU_NOTIFIER(sh, min, PM_QOS_MAX);
DECLARE_GPU_NOTIFIER(3d, max, PM_QOS_MIN);
DECLARE_GPU_NOTIFIER(2d, max, PM_QOS_MIN);
DECLARE_GPU_NOTIFIER(sh, max, PM_QOS_MIN);

struct pm_qos_object *pm_qos_array[] = {
	&null_pm_qos,
	&cpu_dma_pm_qos,
	&network_lat_pm_qos,
	&network_throughput_pm_qos,
	&cpuidle_block_pm_qos,
	&cpu_freq_min_pm_qos,
	&cpu_freq_max_pm_qos,
	&cpu_num_min_pm_qos,
	&cpu_num_max_pm_qos,
	&ddr_devfreq_min_pm_qos,
	&ddr_devfreq_max_pm_qos,
	&axi_min_pm_qos,
	&vpu_devfreq_min_pm_qos,
	&vpu_devfreq_max_pm_qos,
	&gpu_freq_3d_min_pm_qos,
	&gpu_freq_2d_min_pm_qos,
	&gpu_freq_sh_min_pm_qos,
	&gpu_freq_3d_max_pm_qos,
	&gpu_freq_2d_max_pm_qos,
	&gpu_freq_sh_max_pm_qos,
};

static ssize_t pm_qos_power_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos);
static ssize_t pm_qos_power_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos);
static int pm_qos_power_open(struct inode *inode, struct file *filp);
static int pm_qos_power_release(struct inode *inode, struct file *filp);

static const struct file_operations pm_qos_power_fops = {
	.write = pm_qos_power_write,
	.read = pm_qos_power_read,
	.open = pm_qos_power_open,
	.release = pm_qos_power_release,
	.llseek = noop_llseek,
};

/* unlocked internal variant */
static inline int pm_qos_get_value(struct pm_qos_constraints *c)
{
	if (plist_head_empty(&c->list))
		return c->default_value;

	switch (c->type) {
	case PM_QOS_MIN:
		return plist_first(&c->list)->prio;

	case PM_QOS_MAX:
		return plist_last(&c->list)->prio;

	default:
		/* runtime check for not using enum */
		BUG();
		return PM_QOS_DEFAULT_VALUE;
	}
}

s32 pm_qos_read_value(struct pm_qos_constraints *c)
{
	return c->target_value;
}

static inline void pm_qos_set_value(struct pm_qos_constraints *c, s32 value)
{
	c->target_value = value;
}

/**
 * pm_qos_update_target - manages the constraints list and calls the notifiers
 *  if needed
 * @c: constraints data struct
 * @node: request to add to the list, to update or to remove
 * @action: action to take on the constraints list
 * @value: value of the request to add or update
 *
 * This function returns 1 if the aggregated constraint value has changed, 0
 *  otherwise.
 */
int pm_qos_update_target(struct pm_qos_constraints *c, struct plist_node *node,
			 enum pm_qos_req_action action, int value)
{
	unsigned long flags;
	int prev_value, curr_value, new_value;

	spin_lock_irqsave(&pm_qos_lock, flags);
	prev_value = pm_qos_get_value(c);
	if (value == PM_QOS_DEFAULT_VALUE)
		new_value = c->default_value;
	else
		new_value = value;

	switch (action) {
	case PM_QOS_REMOVE_REQ:
		plist_del(node, &c->list);
		break;
	case PM_QOS_UPDATE_REQ:
		/*
		 * to change the list, we atomically remove, reinit
		 * with new value and add, then see if the extremal
		 * changed
		 */
		plist_del(node, &c->list);
	case PM_QOS_ADD_REQ:
		plist_node_init(node, new_value);
		plist_add(node, &c->list);
		break;
	default:
		/* no action */
		;
	}

	curr_value = pm_qos_get_value(c);
	pm_qos_set_value(c, curr_value);

	spin_unlock_irqrestore(&pm_qos_lock, flags);

	trace_pm_qos_update_target(action, prev_value, curr_value);
	if (prev_value != curr_value) {
		blocking_notifier_call_chain(c->notifiers,
					     (unsigned long)curr_value,
					     NULL);
		return 1;
	} else {
		return 0;
	}
}

/**
 * pm_qos_flags_remove_req - Remove device PM QoS flags request.
 * @pqf: Device PM QoS flags set to remove the request from.
 * @req: Request to remove from the set.
 */
static void pm_qos_flags_remove_req(struct pm_qos_flags *pqf,
				    struct pm_qos_flags_request *req)
{
	s32 val = 0;

	list_del(&req->node);
	list_for_each_entry(req, &pqf->list, node)
		val |= req->flags;

	pqf->effective_flags = val;
}

/**
 * pm_qos_update_flags - Update a set of PM QoS flags.
 * @pqf: Set of flags to update.
 * @req: Request to add to the set, to modify, or to remove from the set.
 * @action: Action to take on the set.
 * @val: Value of the request to add or modify.
 *
 * Update the given set of PM QoS flags and call notifiers if the aggregate
 * value has changed.  Returns 1 if the aggregate constraint value has changed,
 * 0 otherwise.
 */
bool pm_qos_update_flags(struct pm_qos_flags *pqf,
			 struct pm_qos_flags_request *req,
			 enum pm_qos_req_action action, s32 val)
{
	unsigned long irqflags;
	s32 prev_value, curr_value;

	spin_lock_irqsave(&pm_qos_lock, irqflags);

	prev_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	switch (action) {
	case PM_QOS_REMOVE_REQ:
		pm_qos_flags_remove_req(pqf, req);
		break;
	case PM_QOS_UPDATE_REQ:
		pm_qos_flags_remove_req(pqf, req);
	case PM_QOS_ADD_REQ:
		req->flags = val;
		INIT_LIST_HEAD(&req->node);
		list_add_tail(&req->node, &pqf->list);
		pqf->effective_flags |= val;
		break;
	default:
		/* no action */
		;
	}

	curr_value = list_empty(&pqf->list) ? 0 : pqf->effective_flags;

	spin_unlock_irqrestore(&pm_qos_lock, irqflags);

	trace_pm_qos_update_flags(action, prev_value, curr_value);
	return prev_value != curr_value;
}

/**
 * pm_qos_request - returns current system wide qos expectation
 * @pm_qos_class: identification of which qos value is requested
 *
 * This function returns the current target value.
 */
int pm_qos_request(int pm_qos_class)
{
	return pm_qos_read_value(pm_qos_array[pm_qos_class]->constraints);
}
EXPORT_SYMBOL_GPL(pm_qos_request);

int pm_qos_request_active(struct pm_qos_request *req)
{
	return req->pm_qos_class != 0;
}
EXPORT_SYMBOL_GPL(pm_qos_request_active);

static void __pm_qos_update_request(struct pm_qos_request *req,
			   s32 new_value)
{
	trace_pm_qos_update_request(req->pm_qos_class, new_value);

	if (new_value != req->node.prio)
		pm_qos_update_target(
			pm_qos_array[req->pm_qos_class]->constraints,
			&req->node, PM_QOS_UPDATE_REQ, new_value);
}

/**
 * pm_qos_work_fn - the timeout handler of pm_qos_update_request_timeout
 * @work: work struct for the delayed work (timeout)
 *
 * This cancels the timeout request by falling back to the default at timeout.
 */
static void pm_qos_work_fn(struct work_struct *work)
{
	struct pm_qos_request *req = container_of(to_delayed_work(work),
						  struct pm_qos_request,
						  work);

	__pm_qos_update_request(req, PM_QOS_DEFAULT_VALUE);
}

/**
 * pm_qos_add_request - inserts new qos request into the list
 * @req: pointer to a preallocated handle
 * @pm_qos_class: identifies which list of qos request to use
 * @value: defines the qos request
 *
 * This function inserts a new entry in the pm_qos_class list of requested qos
 * performance characteristics.  It recomputes the aggregate QoS expectations
 * for the pm_qos_class of parameters and initializes the pm_qos_request
 * handle.  Caller needs to save this handle for later use in updates and
 * removal.
 */

void pm_qos_add_request(struct pm_qos_request *req,
			int pm_qos_class, s32 value)
{
	if (!req) /*guard against callers passing in null */
		return;

	if (pm_qos_request_active(req)) {
		WARN(1, KERN_ERR "pm_qos_add_request() called for already added request\n");
		return;
	}
	req->pm_qos_class = pm_qos_class;
	INIT_DELAYED_WORK(&req->work, pm_qos_work_fn);
	trace_pm_qos_add_request(pm_qos_class, value);
	pm_qos_update_target(pm_qos_array[pm_qos_class]->constraints,
			     &req->node, PM_QOS_ADD_REQ, value);
}
EXPORT_SYMBOL_GPL(pm_qos_add_request);

/**
 * pm_qos_update_request - modifies an existing qos request
 * @req : handle to list element holding a pm_qos request to use
 * @value: defines the qos request
 *
 * Updates an existing qos request for the pm_qos_class of parameters along
 * with updating the target pm_qos_class value.
 *
 * Attempts are made to make this code callable on hot code paths.
 */
void pm_qos_update_request(struct pm_qos_request *req,
			   s32 new_value)
{
	if (!req) /*guard against callers passing in null */
		return;

	if (!pm_qos_request_active(req)) {
		WARN(1, KERN_ERR "pm_qos_update_request() called for unknown object\n");
		return;
	}

	if (delayed_work_pending(&req->work))
		cancel_delayed_work_sync(&req->work);
	__pm_qos_update_request(req, new_value);
}
EXPORT_SYMBOL_GPL(pm_qos_update_request);

/**
 * pm_qos_update_request_timeout - modifies an existing qos request temporarily.
 * @req : handle to list element holding a pm_qos request to use
 * @new_value: defines the temporal qos request
 * @timeout_us: the effective duration of this qos request in usecs.
 *
 * After timeout_us, this qos request is cancelled automatically.
 */
void pm_qos_update_request_timeout(struct pm_qos_request *req, s32 new_value,
				   unsigned long timeout_us)
{
	if (!req)
		return;
	if (WARN(!pm_qos_request_active(req),
		 "%s called for unknown object.", __func__))
		return;

	if (delayed_work_pending(&req->work))
		cancel_delayed_work_sync(&req->work);

	trace_pm_qos_update_request_timeout(req->pm_qos_class,
					    new_value, timeout_us);
	if (new_value != req->node.prio)
		pm_qos_update_target(
			pm_qos_array[req->pm_qos_class]->constraints,
			&req->node, PM_QOS_UPDATE_REQ, new_value);

	schedule_delayed_work(&req->work, usecs_to_jiffies(timeout_us));
}

/**
 * pm_qos_remove_request - modifies an existing qos request
 * @req: handle to request list element
 *
 * Will remove pm qos request from the list of constraints and
 * recompute the current target value for the pm_qos_class.  Call this
 * on slow code paths.
 */
void pm_qos_remove_request(struct pm_qos_request *req)
{
	if (!req) /*guard against callers passing in null */
		return;
		/* silent return to keep pcm code cleaner */

	if (!pm_qos_request_active(req)) {
		WARN(1, KERN_ERR "pm_qos_remove_request() called for unknown object\n");
		return;
	}

	if (delayed_work_pending(&req->work))
		cancel_delayed_work_sync(&req->work);

	trace_pm_qos_remove_request(req->pm_qos_class, PM_QOS_DEFAULT_VALUE);
	pm_qos_update_target(pm_qos_array[req->pm_qos_class]->constraints,
			     &req->node, PM_QOS_REMOVE_REQ,
			     PM_QOS_DEFAULT_VALUE);
	memset(req, 0, sizeof(*req));
}
EXPORT_SYMBOL_GPL(pm_qos_remove_request);

/**
 * pm_qos_add_notifier - sets notification entry for changes to target value
 * @pm_qos_class: identifies which qos target changes should be notified.
 * @notifier: notifier block managed by caller.
 *
 * will register the notifier into a notification chain that gets called
 * upon changes to the pm_qos_class target value.
 */
int pm_qos_add_notifier(int pm_qos_class, struct notifier_block *notifier)
{
	int retval;

	retval = blocking_notifier_chain_register(
			pm_qos_array[pm_qos_class]->constraints->notifiers,
			notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_qos_add_notifier);

/**
 * pm_qos_remove_notifier - deletes notification entry from chain.
 * @pm_qos_class: identifies which qos target changes are notified.
 * @notifier: notifier block to be removed.
 *
 * will remove the notifier from the notification chain that gets called
 * upon changes to the pm_qos_class target value.
 */
int pm_qos_remove_notifier(int pm_qos_class, struct notifier_block *notifier)
{
	int retval;

	retval = blocking_notifier_chain_unregister(
			pm_qos_array[pm_qos_class]->constraints->notifiers,
			notifier);

	return retval;
}
EXPORT_SYMBOL_GPL(pm_qos_remove_notifier);

/* User space interface to PM QoS classes via misc devices */
static int register_pm_qos_misc(struct pm_qos_object *qos)
{
	qos->pm_qos_power_miscdev.minor = MISC_DYNAMIC_MINOR;
	qos->pm_qos_power_miscdev.name = qos->name;
	qos->pm_qos_power_miscdev.fops = &pm_qos_power_fops;

	return misc_register(&qos->pm_qos_power_miscdev);
}

static int find_pm_qos_object_by_minor(int minor)
{
	int pm_qos_class;

	for (pm_qos_class = PM_QOS_CPU_DMA_LATENCY;
		pm_qos_class < PM_QOS_NUM_CLASSES; pm_qos_class++) {
		if (minor ==
			pm_qos_array[pm_qos_class]->pm_qos_power_miscdev.minor)
			return pm_qos_class;
	}
	return -1;
}

static int pm_qos_power_open(struct inode *inode, struct file *filp)
{
	long pm_qos_class;

	pm_qos_class = find_pm_qos_object_by_minor(iminor(inode));
	if (pm_qos_class >= PM_QOS_CPU_DMA_LATENCY) {
		struct pm_qos_request *req = kzalloc(sizeof(*req), GFP_KERNEL);
		if (!req)
			return -ENOMEM;

		pm_qos_add_request(req, pm_qos_class, PM_QOS_DEFAULT_VALUE);
		filp->private_data = req;

		return 0;
	}
	return -EPERM;
}

static int pm_qos_power_release(struct inode *inode, struct file *filp)
{
	struct pm_qos_request *req;

	req = filp->private_data;
	pm_qos_remove_request(req);
	kfree(req);

	return 0;
}


static ssize_t pm_qos_power_read(struct file *filp, char __user *buf,
		size_t count, loff_t *f_pos)
{
	s32 value;
	unsigned long flags;
	struct pm_qos_request *req = filp->private_data;

	if (!req)
		return -EINVAL;
	if (!pm_qos_request_active(req))
		return -EINVAL;

	spin_lock_irqsave(&pm_qos_lock, flags);
	value = pm_qos_get_value(pm_qos_array[req->pm_qos_class]->constraints);
	spin_unlock_irqrestore(&pm_qos_lock, flags);

	return simple_read_from_buffer(buf, count, f_pos, &value, sizeof(s32));
}

static ssize_t pm_qos_power_write(struct file *filp, const char __user *buf,
		size_t count, loff_t *f_pos)
{
	s32 value;
	struct pm_qos_request *req;

	if (count == sizeof(s32)) {
		if (copy_from_user(&value, buf, sizeof(s32)))
			return -EFAULT;
	} else {
		int ret;

		ret = kstrtos32_from_user(buf, count, 16, &value);
		if (ret)
			return ret;
	}

	req = filp->private_data;
	pm_qos_update_request(req, value);

	return count;
}

static struct dentry *cpuidle_block_dentry;

/**
 * cpuidle_block_show - Print information of devices blocking LPM.
 * @m: seq_file to print the statistics into.
 */

static int cpuidle_block_show(struct seq_file *m, void *unused)
{
	unsigned long flags;
	struct pm_qos_object *o;
	struct list_head *list;
	struct plist_node *node;
	struct pm_qos_request *req;
	s32 target_value = PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE;

	o = pm_qos_array[PM_QOS_CPUIDLE_BLOCK];
	list = &o->constraints->list.node_list;

	rcu_read_lock();
	spin_lock_irqsave(&pm_qos_lock, flags);

	target_value = pm_qos_read_value(o->constraints);
	if (target_value != PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE) {
		seq_puts(m, "Can't enter state equals to or deeper than state ");
		seq_printf(m, "%d\n", target_value);
		seq_puts(m, "*****Blocking devices*****\n");
	} else
		seq_puts(m, "SOC can enter all states\n");

	list_for_each_entry(node, list, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_CPUIDLE_BLOCK_DEFAULT_VALUE)
			seq_printf(m, "state %d:\t%s\n", node->prio, req->name);
	}

	spin_unlock_irqrestore(&pm_qos_lock, flags);
	rcu_read_unlock();

	return 0;
}

static int cpuidle_block_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpuidle_block_show, NULL);
}

const struct file_operations cpuidle_block_stats_fops = {
	.owner = THIS_MODULE,
	.open = cpuidle_block_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init cpuidle_block_debugfs_init(void)
{
	cpuidle_block_dentry = debugfs_create_file("cpuidle_block_devices",
			S_IRUGO, NULL, NULL, &cpuidle_block_stats_fops);
	return 0;
}

postcore_initcall(cpuidle_block_debugfs_init);

static int __init pm_qos_power_init(void)
{
	int ret = 0;
	int i;

	BUILD_BUG_ON(ARRAY_SIZE(pm_qos_array) != PM_QOS_NUM_CLASSES);

	for (i = PM_QOS_CPU_DMA_LATENCY; i < PM_QOS_NUM_CLASSES; i++) {
		ret = register_pm_qos_misc(pm_qos_array[i]);
		if (ret < 0) {
			printk(KERN_ERR "pm_qos_param: %s setup failed\n",
			       pm_qos_array[i]->name);
			return ret;
		}
	}

	return ret;
}

late_initcall(pm_qos_power_init);


/**
 * cpufreq_qos_show - Print information of cpu freq qos min and max.
 * @m: seq_file to print the statistics into.
 */
static int cpufreq_qos_show(struct seq_file *m, void *unused)
{
	unsigned long flags;
	struct pm_qos_object *qos_min, *qos_max;
	struct list_head *list_min, *list_max;
	struct plist_node *node;
	s32 target_min = 0, target_max = 0;
	struct pm_qos_request *req;

	qos_min = pm_qos_array[PM_QOS_CPUFREQ_MIN];
	list_min = &qos_min->constraints->list.node_list;
	qos_max = pm_qos_array[PM_QOS_CPUFREQ_MAX];
	list_max = &qos_max->constraints->list.node_list;

	rcu_read_lock();
	spin_lock_irqsave(&pm_qos_lock, flags);

	target_min = pm_qos_read_value(qos_min->constraints);
	target_max = pm_qos_read_value(qos_max->constraints);

	seq_printf(m, "Target min %d\n", target_min);
	list_for_each_entry(node, list_min, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_DEFAULT_VALUE)
			seq_printf(m, "Req: %d\t Name: %s\n",
				node->prio, req->name);
	}

	seq_printf(m, "Target max %d\n", target_max);
	list_for_each_entry(node, list_max, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_DEFAULT_VALUE)
			seq_printf(m, "Req: %d\t Name: %s\n",
				node->prio, req->name);
	}
	spin_unlock_irqrestore(&pm_qos_lock, flags);
	rcu_read_unlock();

	return 0;
}

static int cpufreq_qos_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpufreq_qos_show, NULL);
}

const struct file_operations cpufreq_qos_fops = {
	.owner = THIS_MODULE,
	.open = cpufreq_qos_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init cpufreq_qos_debugfs_init(void)
{
	debugfs_create_file("cpufreq_qos",
			S_IRUGO, NULL, NULL, &cpufreq_qos_fops);
	return 0;
}
postcore_initcall(cpufreq_qos_debugfs_init);

/**
 * cpu_num_qos_show - Print information of cpu num qos.
 * @m: seq_file to print the statistics info.
 */
static int cpu_num_qos_show(struct seq_file *m, void *unused)
{
	unsigned long flags;
	struct pm_qos_object *qos_max, *qos_min;
	struct list_head *list_max, *list_min;
	struct plist_node *node;
	s32 target_max = 0, target_min = 0;
	struct pm_qos_request *req;

	qos_min = pm_qos_array[PM_QOS_CPU_NUM_MIN];
	list_min = &qos_min->constraints->list.node_list;
	qos_max = pm_qos_array[PM_QOS_CPU_NUM_MAX];
	list_max = &qos_max->constraints->list.node_list;

	rcu_read_lock();
	spin_lock_irqsave(&pm_qos_lock, flags);

	target_min = pm_qos_read_value(qos_min->constraints);
	seq_printf(m, "Target min %d\n", target_min);
	list_for_each_entry(node, list_min, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_DEFAULT_VALUE)
			seq_printf(m, "Req: %d\t Name: %s\n",
					node->prio, req->name);
	}

	target_max = pm_qos_read_value(qos_max->constraints);

	seq_printf(m, "Target max %d\n", target_max);
	list_for_each_entry(node, list_max, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_DEFAULT_VALUE)
			seq_printf(m, "Req: %d\t Name: %s\n",
				node->prio, req->name);
	}
	spin_unlock_irqrestore(&pm_qos_lock, flags);
	rcu_read_unlock();

	return 0;
}

static int cpu_num_qos_open(struct inode *inode, struct file *file)
{
	return single_open(file, cpu_num_qos_show, NULL);
}

const struct file_operations cpu_num_qos_fops = {
	.owner = THIS_MODULE,
	.open = cpu_num_qos_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init cpu_num_qos_debugfs_init(void)
{
	debugfs_create_file("cpu_num_qos",
			S_IRUGO, NULL, NULL, &cpu_num_qos_fops);
	return 0;
}
postcore_initcall(cpu_num_qos_debugfs_init);

/**
 * ddrfreq_qos_show - Print information of ddr freq qos min and max.
 * @m: seq_file to print the statistics into.
 */
static int ddrfreq_qos_show(struct seq_file *m, void *unused)
{
	unsigned long flags;
	struct pm_qos_object *qos_min, *qos_max;
	struct list_head *list_min, *list_max;
	struct plist_node *node;
	s32 target_min = 0, target_max = 0;
	struct pm_qos_request *req;

	qos_min = pm_qos_array[PM_QOS_DDR_DEVFREQ_MIN];
	list_min = &qos_min->constraints->list.node_list;
	qos_max = pm_qos_array[PM_QOS_DDR_DEVFREQ_MAX];
	list_max = &qos_max->constraints->list.node_list;

	rcu_read_lock();
	spin_lock_irqsave(&pm_qos_lock, flags);

	target_min = pm_qos_read_value(qos_min->constraints);
	target_max = pm_qos_read_value(qos_max->constraints);

	seq_printf(m, "Target min %d\n", target_min);
	list_for_each_entry(node, list_min, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_DEFAULT_VALUE)
			seq_printf(m, "Req: %d\t Name: %s\n",
				node->prio, req->name);
	}

	seq_printf(m, "Target max %d\n", target_max);
	list_for_each_entry(node, list_max, node_list) {
		req = container_of(node, struct pm_qos_request, node);
		if (node->prio != PM_QOS_DEFAULT_VALUE)
			seq_printf(m, "Req: %d\t Name: %s\n",
				node->prio, req->name);
	}
	spin_unlock_irqrestore(&pm_qos_lock, flags);
	rcu_read_unlock();

	return 0;
}

static int ddrfreq_qos_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddrfreq_qos_show, NULL);
}

const struct file_operations ddrfreq_qos_fops = {
	.owner = THIS_MODULE,
	.open = ddrfreq_qos_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init ddrfreq_qos_debugfs_init(void)
{
	debugfs_create_file("ddrfreq_qos",
			S_IRUGO, NULL, NULL, &ddrfreq_qos_fops);
	return 0;
}
postcore_initcall(ddrfreq_qos_debugfs_init);

/*
 * gpufreq_qos_show - Print information of gpu freq qos min and max.
 * @m: seq_file to print the statistics into.
 */
#define __GPUFREQ_QOS_SHOW(CORE, PM_QOS_CLASS_MIN, PM_QOS_CLASS_MAX) \
{ \
	unsigned long flags; \
	struct pm_qos_object *qos_min, *qos_max; \
	struct list_head *list_min, *list_max; \
	struct plist_node *node; \
	s32 target_min = 0, target_max = 0; \
	struct pm_qos_request *req; \
\
	qos_min = pm_qos_array[PM_QOS_CLASS_MIN]; \
	list_min = &qos_min->constraints->list.node_list; \
	qos_max = pm_qos_array[PM_QOS_CLASS_MAX]; \
	list_max = &qos_max->constraints->list.node_list; \
\
	rcu_read_lock(); \
	spin_lock_irqsave(&pm_qos_lock, flags); \
\
	target_min = pm_qos_read_value(qos_min->constraints); \
	target_max = pm_qos_read_value(qos_max->constraints); \
\
	seq_printf(m, #CORE " | Target min %d\n", target_min); \
	list_for_each_entry(node, list_min, node_list) { \
		req = container_of(node, struct pm_qos_request, node); \
		if (node->prio != 0) \
			seq_printf(m, "Req: %d\t Name: %s\n", \
				node->prio, req->name); \
	} \
	seq_printf(m, "\n"); \
\
	seq_printf(m, #CORE " | Target max %d\n", target_max); \
	list_for_each_entry(node, list_max, node_list) { \
		req = container_of(node, struct pm_qos_request, node); \
		if (node->prio != 0) \
			seq_printf(m, "Req: %d\t Name: %s\n", \
				node->prio, req->name); \
	} \
	seq_printf(m, "\n"); \
	spin_unlock_irqrestore(&pm_qos_lock, flags); \
	rcu_read_unlock(); \
}

static int gpufreq_qos_show(struct seq_file *m, void *unused)
{
	__GPUFREQ_QOS_SHOW(3D, PM_QOS_GPUFREQ_3D_MIN, PM_QOS_GPUFREQ_3D_MAX);
	__GPUFREQ_QOS_SHOW(2D, PM_QOS_GPUFREQ_2D_MIN, PM_QOS_GPUFREQ_2D_MAX);
	__GPUFREQ_QOS_SHOW(SH, PM_QOS_GPUFREQ_SH_MIN, PM_QOS_GPUFREQ_SH_MAX);
	return 0;
}

static int gpufreq_qos_open(struct inode *inode, struct file *file)
{
	return single_open(file, gpufreq_qos_show, NULL);
}

const struct file_operations gpufreq_qos_fops = {
	.owner = THIS_MODULE,
	.open = gpufreq_qos_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init gpufreq_qos_debugfs_init(void)
{
	debugfs_create_file("gpufreq_qos",
			S_IRUGO, NULL, NULL, &gpufreq_qos_fops);
	return 0;
}
postcore_initcall(gpufreq_qos_debugfs_init);