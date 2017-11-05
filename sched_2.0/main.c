/*
 * main.c
 *
 *  Created on: 2017. ápr. 11.
 *      Author: tibi
 */
/* Release notes:
 *
 *	Not fully perfect things:
 * 		- at module init slot address range should be checked, and should implement correct freeing mechanism
 *
 *
 * TODO:
 *  	Interrupt handling
 *
 *
 *
 *
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kthread.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <asm-generic/current.h>
#include <linux/spinlock.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/mm_types.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/uaccess.h>
#include <linux/device.h>
#include <linux/of.h>

#include <linux/fpga/fpga-mgr.h>
#include <linux/of.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/ioctl.h>

#include <linux/platform_device.h>

#include <linux/dma-mapping.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/ioport.h>


/* Resource locking policy:
	// dev_close
 * virt_dev -> event queue -> slot
 * Locking may only happen in this direction. !!!
 *
 * if vdev->status == OPERATING && vdev->slot->status == OPERATING than the pairing is guaratied!!!
 */
 
#define SLOT_FREE 0
#define SLOT_USED 1

struct dev_priv
{
	int acc_id;
	struct device *dev;
	struct list_head waiters;
	struct list_head active_list;
};
struct virtual_dev_t;
struct slot_info_t
{
	struct virtual_dev_t *user;
	int status;
	struct dev_priv *actual_dev;
	uint8_t busy;

	// slot virtual address
	uint8_t *slot_kaddr;
	// slot physical address range
	uint32_t base_address;
	uint32_t address_length;
	// compatible accelerator list
	uint32_t compatible_accel_num;
	struct dev_priv **compatible_accels;
};
// Contains information about the device session.
// This structure is linked to the file descriptor and thus to the session.
#define DEV_STATUS_BLANK 		0
// user thread changes to QUEUED
#define DEV_STATUS_QUEUED 		1
/*
 * Slot connection is ready!!!
 * kthread changes to operating
 */
#define DEV_STATUS_OPERATING	2
// kthread or user thread changes state to releasing or closing
#define DEV_STATUS_RELEASING	3
#define DEV_STATUS_CLOSING		4
struct virtual_dev_t
{
	// virtual device status
	int status;
	struct spinlock status_lock;
	// Given slot
	struct slot_info_t *slot;
	// linked list for enqueueing
	struct list_head waiter_list;
	// wait point for status "operating"
	struct completion compl;
	// for debugging
	struct task_struct *user;
	// deleting flag
	int marked_for_death;
	// DMA buffer desciptor
	struct dev_dma_buffer_desc *dma_buffer_desc;
};

#define EVENT_REQUEST 	1
#define EVENT_CLOSE		2
struct user_event
{
	int event_type;
	// sender process
	struct task_struct *sender;
	// list element for listing
	struct list_head waiter_list;
	// request specific values
	int acc_id;
	// corresponding virtual device
	struct virtual_dev_t *vdev;
};

#define AXI_DMA_BASE_OFFSET 			(0x00U)
	#define AXI_DMA_TX_OFFSET				(0x00U)
	#define AXI_DMA_RX_OFFSET				(0x30U)
	#define AXI_DMA_CONTROL_OFFSET			(0x00U)
	#define AXI_DMA_ADDRESS_OFFSET			(0x18U)
	#define AXI_DMA_LENGTH_OFFSET			(0x28U)
#define AXI_DECOUPLER_BASE_OFFSET		(0x1000U)
#define AXI_RST_BASE_OFFSET				(0x2000U)
#define AXI_ACCEL_BASE_OFFSET			(0x3000U)

/* Descriptor for the dma buffer subsystem
*	ref_cntr - reference counter for the buffer (2 sources: virtual device and the vm areas from the mmapping)
	dma_addr - physical address of the buffer
	kaddr - kernel virtual address
	length - length of the buffer
	lock - spinlock for the reference counter
*/
struct dev_dma_buffer_desc
{
	uint32_t ref_cntr;
	struct spinlock lock;
	uint32_t dma_addr;
	uint8_t *kaddr;
	uint32_t length;
	
	uint32_t tx_base;
	uint32_t tx_length;
	uint32_t rx_base;
	uint32_t rx_length;
}
	
#define FPGA_STATIC_CONFIG_NAME "static.bit"

/**
 * lock policy: only kthread can get more than one spin lock, other threads may only get 1!
 */
// global variables
//#define SLOT_NUM 2 // TODO CORE gather it form device tree

// user interface
	static struct miscdevice misc;
	static struct file_operations dev_fops;

// event queue
	DEFINE_SPINLOCK(event_list_lock);
	LIST_HEAD(event_list);
	DECLARE_COMPLETION(event_in);
	static void add_event(struct user_event *e);

// scheduler thread
	static uint8_t quit = 0;
	DECLARE_COMPLETION(thread_stop);
	static struct task_struct *sched_thread;

//fpga manager data
	static struct fpga_manager *mgr;

// fpga slot data
	DEFINE_SPINLOCK(slot_lock);
	unsigned int SLOT_NUM;
	unsigned free_slot_num;
	struct slot_info_t *slot_info;

// fpga accelerators data
	unsigned device_num;
	struct dev_priv *device_data; // pointer to the dev_priv array

// device modell data
	static struct device fpga_virtual_bus;
	static struct device *devices;

	
	/***************************************
		Internal functions
	***************************************/
	
	static struct dev_dma_buffer_desc* dev_dma_buffer_alloc(uint32_t length);
	static void dev_dma_buffer_get(struct dev_dma_buffer_desc* desc);
	static void dev_dma_buffer_put(struct dev_dma_buffer_desc* desc);
	static int lock_operating_vdev(struct virtual_dev_t *vdev);
	static void unlock_operating_vdev(struct virtual_dev_t *vdev);
	
	static int dev_dma_buffer_start_dma(struct dev_dma_buffer_desc* desc,struct slot_info_t *slot);
	
	
	// database builder functions
	static int gather_slot_data(void);
	static void free_slot_data(void);
	static void print_slot_data(void)
	static int gather_accel_data(void);
	static void free_accel_data(void);

	// fpga manager
	static int connect_to_fpga_mgr(void);
	static void disconnect_from_fpga_mgr(void);

	// linux device modell functions
	static int build_device_modell(void);
	static void clean_device_modell(void);

	// procfs interface
	static int procfs_init(void);
	static void procfs_destroy(void);

	
	
	//------------------------------------------------------------//
static int lock_operating_vdev(struct virtual_dev_t *vdev)
{
	spin_lock(&(vdev->status_lock));
	if(vdev->status == DEV_STATUS_OPERATING)
	{
		spin_lock(&slot_lock);
		if(vdev->slot->status == SLOT_USED)
			return 0;
		else
		{
			spin_unlock(&slot_lock);
			spin_unlock(&(vdev->status_lock));
			return -1;
		}
	}
	else
		spin_unlock(&(vdev->status_lock));
	return -1;
}

static void unlock_operating_vdev(struct virtual_dev_t *vdev)
{
	spin_unlock(&slot_lock);
	spin_unlock(&(vdev->status_lock));
}
	
static void add_event(struct user_event *e)
{
	int empty;

	spin_lock(&event_list_lock);
	empty = list_empty(&event_list);
	list_add_tail(&(e->waiter_list),&event_list);
	spin_unlock(&event_list_lock);
	// notify scheduler thread
	if(empty) complete(&event_in);
}


static struct user_event* get_event(void)
{
	struct user_event *ue;
	spin_lock(&(event_list_lock));
	if(list_empty(&event_list))
		ue = NULL;
	else
	{
		ue = list_entry(event_list.next,struct user_event, waiter_list);
		// remove event from the queue
		list_del_init(&(ue->waiter_list));
	}
	spin_unlock(&(event_list_lock));
	return ue;
}

static int program_fpga(const char *name)
{
	uint32_t ret;
	// load the fpga with the static configuration
	ret = fpga_mgr_firmware_load(mgr, 0, name);
	if(ret)
		pr_err("Cannot initialize static fpga_region.\n");
	return ret;
}
static int program_slot(int slot_num, int acc_id)
{
	uint32_t tmp;
	int ret;

	if(slot_num < 0 || slot_num >= SLOT_NUM) return -1;
	// stop the previous accelerator
	// reset accelerator
	iowrite8(1,slot_info[slot_num].slot_kaddr + AXI_RST_BASE_OFFSET);
	// decouple accelerator
	iowrite8(1, slot_info[slot_num].slot_kaddr + AXI_DECOUPLER_BASE_OFFSET);
	// reset TX and RX DMA channels
	tmp = ioread32(slot_info[slot_num].slot_kaddr + AXI_DMA_TX_OFFSET + AXI_DMA_CONTROL_OFFSET);
	tmp |= (1<<2); // Set Reset bit in the control register
	iowrite32(tmp,slot_info[slot_num].slot_kaddr + AXI_DMA_TX_OFFSET + AXI_DMA_CONTROL_OFFSET);


	static char name[100];
	snprintf(name,100,"%s.bit",device_data[acc_id].dev->of_node->name);
	// start programming
	pr_info("Loading configuration: %s - id :%d.\n",name,acc_id);
	ret = fpga_mgr_firmware_load(mgr, FPGA_MGR_PARTIAL_RECONFIG, name);
	pr_info("FPGA configuration finished.\n");
	if(ret)
	{
		pr_err("FPGA config failed...\n");
	}
	else
	{
		// starting the new accelerator
		// clear decoupling
		iowrite8(0, slot_info[slot_num].slot_kaddr + AXI_DECOUPLER_BASE_OFFSET);
		// clear reset signal
		iowrite8(0,slot_info[slot_num].slot_kaddr + AXI_RST_BASE_OFFSET);
	}
	return ret;
}
static void hw_schedule(void)
{
	int i,j;
	struct virtual_dev_t *vdev;
	struct dev_priv *d = NULL;

	spin_lock(&slot_lock);
	if(free_slot_num==0)
	{
		spin_unlock(&slot_lock);
		return;
	}
	pr_info("Scheduling");
	for(i=0;i<SLOT_NUM;i++)
		if(slot_info[i].status==SLOT_FREE)
		{
			// try to match request with this slot
			spin_unlock(&slot_lock);
			// try actual dev
			if(slot_info[i].actual_dev && !list_empty(&(slot_info[i].actual_dev->waiters)))
			{
					vdev = list_entry(slot_info[i].actual_dev->waiters.next,struct virtual_dev_t,waiter_list);
					d = slot_info[i].actual_dev;

					list_del_init(&(vdev->waiter_list));
					spin_lock(&slot_lock);
					slot_info[i].status = SLOT_USED;
					slot_info[i].user = vdev;
					free_slot_num--;
					spin_unlock(&slot_lock);

					spin_lock(&(vdev->status_lock));
					vdev->slot = &slot_info[i];
					vdev->status = DEV_STATUS_OPERATING;
					spin_unlock(&(vdev->status_lock));
					pr_info("Starting process %d for slot: %d, with accel: %d.\n",vdev->user ? vdev->user->pid : -1,i,slot_info[i].actual_dev->acc_id);
					complete(&(vdev->compl));
			}
			else
			{
				struct dev_priv *dev;
				vdev = NULL;
				// look for other requests that are waiting for a device that is compatible with the current slot
				for(j=0; j< slot_info[i].compatible_accel_num; j++)
				{
					dev = slot_info[i].compatible_accel_num[j];
					// examine the waiters list of the selected device
					if(!list_empty(&(dev->waiters)))
					{
						vdev = list_entry(dev->waiters.next,struct virtual_dev_t,waiter_list);
						d = dev;
						break;
					}
				}

				// serve the selected request
				if(vdev)
				{
					list_del_init(&(vdev->waiter_list));

					// start slot programming
					if(!program_slot(i,d->acc_id))
					{
						// programming succeeded
						spin_lock(&(vdev->status_lock));
						spin_lock(&slot_lock);
						slot_info[i].status = SLOT_USED;
						slot_info[i].actual_dev = d;
						slot_info[i].user = vdev;
						free_slot_num--;
						vdev->status = DEV_STATUS_OPERATING;
						vdev->slot = &slot_info[i];
						spin_unlock(&slot_lock);
						spin_unlock(&(vdev->status_lock));
						pr_info("Starting process %d for slot: %d, with accel: %d.\n",vdev->user?vdev->user->pid:-1,i,d->acc_id);
					}
					else
					{
						// programming failed
						spin_lock(&(vdev->status_lock));
						vdev->status = DEV_STATUS_BLANK;
						vdev->slot = NULL;
						spin_unlock(&(vdev->status_lock));
						pr_err("FPGA programming failed for slot %d with accel %d.\n",i,d->acc_id);
					}

					// start waiter process
					complete(&(vdev->compl));
				}
			}
			spin_lock(&slot_lock);
		}
	spin_unlock(&slot_lock);
}

static int sched_thread_fn(void *data)
{
	pr_info("Scheduler thread started.\n");

	//wait for incoming events
	wait_for_completion_interruptible(&event_in);

	while(!quit)
	{
		struct user_event *r = NULL;
		struct virtual_dev_t *vdev = NULL;
		int acc_id;

		while((r = get_event()) != NULL)
			{
				pr_info("New event.\n");
				vdev = r->vdev;
				switch(r->event_type)
				{
					case EVENT_REQUEST:
					{
						acc_id = r->acc_id;
						// add virtual device to the waiting queue
						list_add_tail(&(vdev->waiter_list),&(device_data[acc_id].waiters));

						// free event
						kfree(r);
						r=NULL;
						pr_info("Request processed for accel: %d.\n",acc_id);
						break;
					}
					case EVENT_CLOSE:
					{
						// EVENT_CLOSE is generated when the hw file is closed
						int status;
						struct slot_info_t *slot;

						// if the status is releasing, wait for the user process thread to finish the releasing
						do
						{
							spin_lock(&(vdev->status_lock));
							status = vdev->status;
							if(status != DEV_STATUS_RELEASING) vdev->status = DEV_STATUS_CLOSING;
							spin_unlock(&(vdev->status_lock));
							if(status == DEV_STATUS_RELEASING) msleep(1);
						}while(status==DEV_STATUS_RELEASING);

						// close the virtual device according to the status
						switch(status)
						{
						case DEV_STATUS_BLANK:
							dev_dma_buffer_put(vdev->dma_buffer_desc);
							kfree(vdev);
							break;
						case DEV_STATUS_QUEUED:
							list_del(&(vdev->waiter_list));
							dev_dma_buffer_put(vdev->dma_buffer_desc);
							kfree(vdev);
							break;
						case DEV_STATUS_OPERATING:
							slot = vdev->slot;
							if(!slot) {pr_err("STATE anomaly: operating vs slot null.\n"); break;}
							// destroy slot conection
							spin_lock(&slot_lock);
							slot->status = SLOT_FREE;
							slot->user = NULL;
							free_slot_num++;
							vdev->slot = NULL;
							spin_unlock(&slot_lock);
							dev_dma_buffer_put(vdev->dma_buffer_desc);
							kfree(vdev);
							break;
						default:
							pr_err("Unknown vdev status: %d.\n",status);
						}

						pr_info("Virtual device closed.\n");
						break;
					}
					default:
						pr_err("Unknown event type. This might cause memory leak.\n");
						break;
				} //switch
			} //while

		hw_schedule();


		//wait for incoming events
		wait_for_completion_interruptible(&event_in);
	}
	pr_info("Scheduler thread stopped.\n");
	// TODO SAFETY free waiting request structures
	complete(&thread_stop);
	return 0;
}

// user interface

static ssize_t dev_open(struct inode *inode, struct file *pfile)
{
	struct virtual_dev_t *vdev;
	// Allocate new virtual device

	vdev = (struct virtual_dev_t*)kmalloc(sizeof(struct virtual_dev_t),GFP_KERNEL);
	if(!vdev)
		return -ENOMEM;
	// initialize virtual_dev
	init_completion(&(vdev->compl));
	vdev->marked_for_death = 0;
	vdev->slot= NULL;
	vdev->status = DEV_STATUS_BLANK;
	vdev->dma_buffer_desc = NULL;
	vdev-> user = NULL;
	spin_lock_init(&(vdev->status_lock));
	INIT_LIST_HEAD(&(vdev->waiter_list));

	pfile->private_data = (void*)vdev;

	try_module_get(THIS_MODULE);
	return 0;
}

// ioctl command codes
#define IOCTL_RELEASE 			0
#define IOCTL_REQUIRE 			1
#define IOCTL_AXI_READ			2
#define IOCTL_AXI_WRITE			3
#define IOCTL_DMA_SET_TX_BASE	4
#define IOCTL_DMA_SET_TX_LENGTH	5
#define IOCTL_DMA_SET_RX_BASE	6
#define IOCTL_DMA_SET_RX_LENGTH	7
#define IOCTL_DMA_START			8

// TODO EXTRA non-blocking require

// arguments:
//	IOCTL_RELEASE: slot number
//  IOCTL_REQUIRE: accel_id
// IOCTL_AXI_READ: offset
// IOCTL_AXI_WRITE: offset <32 bit value>

// ADDRESS ECODING IN THE READ / WRITE operations
// cmd = <16 bit addr> | <16bit command>
static long dev_ioctl(struct file *pfile, unsigned int cmd, unsigned long arg)
{
	struct virtual_dev_t *vdev;
	int ret;
	uint16_t command;
	uint16_t address;
	uint32_t data;
	
	command = cmd & 0xFFFF;
	address = (cmd>>16) & 0xFFFC; //Masking to 32bit address (lower 2 bits are 0)
	data = arg;
	

	vdev = (struct virtual_dev_t*)pfile->private_data;
	switch(command)
	{
		case IOCTL_RELEASE:
			// check for status
			if(!lock_operating_vdev(vdev))
			{
				if(vdev->slot != NULL && vdev->slot->user == vdev)
				{
					vdev->slot->status = SLOT_FREE;
					vdev->slot->user = NULL;

					vdev->slot = NULL;
					vdev->status = DEV_STATUS_BLANK;
					free_slot_num++;
					ret = 0;
				}
				else 
				{
					pr_err("Unauthorized slot access.");
					ret = -EPERM;

				}
				unlock_operating_vdev(vdev);
			}
			else
			{
				pr_err("Unauthorized slot access.\n");
				ret = -EPERM;
			}
		

			complete(&event_in);
			return ret;
			break;
		case IOCTL_REQUIRE:
		{
			struct user_event *my_request;
			int slot_id;

			// check acc_id
			if(arg >= device_num)
			{
				pr_err("Requesting non existent accelerator: %ld.\n",arg);
				return -EPERM;
			}

			if(pfile->private_data == NULL)
			{
				pr_err("Private data is empty.\n");
				return -EINVAL;
			}

			my_request = (struct user_event*)kmalloc(sizeof(struct user_event),GFP_KERNEL);
			if(!my_request)
			{
				pr_err("No memory for request allocation.\n");
				return -ENOMEM;
			}
			// initialize request
			my_request->acc_id = arg;
			my_request->event_type = EVENT_REQUEST;
			my_request->sender = current;
			my_request->vdev = (struct virtual_dev_t*)pfile->private_data;
			INIT_LIST_HEAD(&(my_request->waiter_list));


			vdev->user = current;
			// check vdev status
			spin_lock(&(vdev->status_lock));
			if(vdev->status == DEV_STATUS_BLANK && vdev->marked_for_death==0)
			{
				vdev->status = DEV_STATUS_QUEUED;
				// append request to the event list
				add_event(my_request);
			}
			else
			{
				// close request already given
				spin_unlock(&(vdev->status_lock));
				kfree(my_request);
				return -EBUSY;
			}
			spin_unlock(&(vdev->status_lock));

			// wait for load ready
			wait_for_completion_killable(&(vdev->compl));
			// accel loaded
			slot_id = vdev->slot - slot_info;
			return slot_id;
		}
			break;
		case IOCTL_AXI_READ:
			spin_lock(&(vdev->status_lock));
			if(vdev->status == DEV_STATUS_OPERATING)
			{
				spin_lock(&slot_lock);
				if(vdev->slot->status == SLOT_USED)
				{
					uint8_t *hw_addr;
					// Physical address = slot base + accel base offset + offset
					hw_addr = vdev->slot->slot_kaddr + AXI_ACCEL_BASE_OFFSET + address;
					data = ioread32(hw_addr);
				}
				spin_unlock(slot_lock);
			}
			spin_unlock(&(vdev->status_lock));
			return data;
			break;
		case IOCTL_AXI_WRITE:
			spin_lock(&(vdev->status_lock));
			if(vdev->status == DEV_STATUS_OPERATING)
			{
				spin_lock(&slot_lock);
				if(vdev->slot->status == SLOT_USED)
				{
					uint8_t *hw_addr;
					// Physical address = slot base + accel base offset + offset
					hw_addr = vdev->slot->slot_kaddr + AXI_ACCEL_BASE_OFFSET + address;
					iowrite32(data,hw_addr);	
				}
				spin_unlock(&slot_lock);
			}
			spin_unlock(&(vdev->status_lock));
			break;
		case IOCTL_DMA_SET_RX_BASE:
			spin_lock(&(vdev->status_lock));
				if(vdev->status != DEV_STATUS_CLOSING && vdev->dma_buffer_desc!=NULL)
				{
					vdev->dma_buffer_desc.rx_base = arg;
				}
			spin_unlock(&(vdev->status_lock));
			break;
		case IOCTL_DMA_SET_RX_LENGTH:
			spin_lock(&(vdev->status_lock));
				if(vdev->status != DEV_STATUS_CLOSING && vdev->dma_buffer_desc!=NULL)
				{
					vdev->dma_buffer_desc.rx_length = arg;
				}
			spin_unlock(&(vdev->status_lock));
			break;
		case IOCTL_DMA_SET_TX_BASE:
			spin_lock(&(vdev->status_lock));
				if(vdev->status != DEV_STATUS_CLOSING && vdev->dma_buffer_desc!=NULL)
				{
					vdev->dma_buffer_desc.tx_base = arg;
				}
			spin_unlock(&(vdev->status_lock));
			break;
		case IOCTL_DMA_SET_TX_LENGTH:
			spin_lock(&(vdev->status_lock));
				if(vdev->status != DEV_STATUS_CLOSING && vdev->dma_buffer_desc!=NULL)
				{
					vdev->dma_buffer_desc.tx_length = arg;
				}
			spin_unlock(&(vdev->status_lock));
			break;
		case IOCTL_DMA_START:
			if(!lock_operating_vdev(vdev))
			{
				vdev->slot->busy = 1;
				if(!dev_dma_buffer_start_dma(vdev->dma_buffer_desc, vdev->slot)) ret = 0;
				else ret = -EINVAL;
				unlock_operating_vdev(vdev);
			}
			else
			{
				pr_err("Accelerator is not working.\n");
				ret = -EPERM;
			}
			return ret;
			break;
		default:
			pr_err("Unknown ioctl command code: %u.\n",cmd);
			return -EPERM;
			break;
	}
}

// give status informations
#define BUFF_LEN 256
static char buffer[BUFF_LEN];
static ssize_t dev_read (struct file *pfile, char __user *buff, size_t len, loff_t *ppos)
{
	int status;
	int slot = -1;
	int acc_id = -1;
	struct virtual_dev_t *vdev;
	int data_len;

	vdev = (struct virtual_dev_t*)pfile->private_data;
	if(!vdev) return -ENODEV;

	spin_lock(&(vdev->status_lock));
	status = vdev->status;
	if(status == DEV_STATUS_OPERATING)
	{
		slot = vdev->slot-slot_info;
		acc_id = vdev->slot->actual_dev->acc_id;
	}
	spin_unlock(&(vdev->status_lock));
	// create status report
	data_len = snprintf(buffer,BUFF_LEN,"Status :%d\nSlot: %d\nAcc_id: %d\n",status,slot,acc_id)+1;
	if(data_len > BUFF_LEN) data_len = BUFF_LEN;

	if(len > data_len) len = data_len;


	if(copy_to_user(buff,buffer,len))
		return -EFAULT;
	return len;
}

static int dev_close (struct inode *inode, struct file *pfile)
{
	struct virtual_dev_t *vdev;
	// send event to delete all task related requests from the system
	struct user_event *e;

	vdev = (struct virtual_dev_t*)pfile->private_data;

	e = (struct user_event*)kmalloc(sizeof(struct user_event),GFP_KERNEL);
	e->event_type = EVENT_CLOSE;
	e->vdev = (struct virtual_dev_t*)pfile->private_data;
	e->sender = current;

	spin_lock(&(vdev->status_lock));
	vdev->marked_for_death = 1;
	add_event(e);
	spin_unlock(&(vdev->status_lock));

	module_put(THIS_MODULE);
	return 0;
}

/* ======================================================================== 
					MMAP IMPLEMENTATION

	static void dev_dma_buffer_get(struct dev_dma_buffer_desc *desc);
	static void dev_dma_buffer_put(struct dev_dma_buffer_desc *desc);
	static struct dev_dma_buffer_desc* dev_dma_buffer_alloc(uint32_t length);
	dev_dma_buffer_start_dma(struct dev_dma_buffer_desc* desc, struct slot_info_t *slot);
   ========================================================================*/

// ----------------- DMA BUFFER subsystem ---------------- //
#define MMAP_MAX_BUFFER_LENGTH	4096
	static void dev_dma_buffer_get(struct dev_dma_buffer_desc *desc)
	{
		spin_lock(&(desc->lock));
		desc->ref_cntr++;
		spin_unlock(&(desc->lock));
	}
	static void dev_dma_buffer_put(struct dev_dma_buffer_desc *desc)
	{
		uint32_t cntr;
		spin_lock(&(desc->lock));
		desc->ref_cntr--;
		cntr = desc->ref_cntr;
		spin_unlock(&(desc->lock));
		if(cntr==0)
		{
			// Free the buffer and the descriptor
			dev_info(misc->this_device,"Freeing coherent dma buffer.\n");
			dma_free_coherent(misc->this_device, desc->length, desc->kaddr, desc->dma_addr);
			kfree(desc);		
		}
	}

	static struct dev_dma_buffer_desc* dev_dma_buffer_alloc(uint32_t length)
	{
		struct dev_dma_buffer_desc *desc;
		uint32_t dma_addr;
		uint8_t *kbuf;
		
		dev_info(misc->this_device, "Allocating coherent DMA buffer.\n");
		
		// TODO use the given length		
		length = MMAP_MAX_BUFFER_LENGTH;
		// Allocate coherent dma buffer
		kbuf = dma_alloc_coherent(misc->this_device, length, &dma_addr, GFP_KERNEL);
		if(!kbuf)
		{
			pr_err("Cannot allocate coherent dma buffer.\n");
			return NULL;
		}
		
		// Create descriptor for the dma buffer
		desc = kmalloc(MMAP_MAX_BUFFER_LENGTH,GFP_KERNEL);
		if(!desc)
		{
			// rewind coherent allocation
			dma_free_coherent(misc->this_device, length, kbuf, dma_addr);
			return NULL;
		}
		desc-> ref_cntr = 1;
		desc->dma_addr = dma_addr;
		desc-> kaddr = kbuf;
		desc->length = length;
		desc-> tx_base = 0;
		desc->rx_base = 0;
		desc-> tx_length = 0;
		desc->rx_length = 0;
		spin_lock_init(&(desc->lock));
		return desc;
	}
	
	struct dev_dma_buffer_desc
	{
		uint32_t ref_cntr;
		struct spinlock lock;
		uint32_t dma_addr;
		uint8_t *kaddr;
		uint32_t length;
		
		uint32_t tx_base;
		uint32_t tx_length;
		uint32_t rx_base;
		uint32_t rx_length;
	};

static int dev_dma_buffer_start_dma(struct dev_dma_buffer_desc* desc, struct slot_info_t *slot)
{
		uint8_t *slot_base = slot->slot_kaddr;
		uint32_t buf_len = desc->length;
		// Validate rx and rx parameters
		if((tx_base+tx_length > buf_len) || (rx_base+rx_length > buf_len) || (tx_base & 0x3F !=0) || (rx_base & 0x3F !=0))
		{
			dev_err(misc->this_device,"Invalid dma parameters.\n");
			return -1;
		}
		//program the tx channel
		iowrite8(1,slot_base + AXI_DMA_BASE_OFFSET + AXI_DMA_TX_OFFSET + AXI_DMA_CONTROL_OFFSET );
		iowrite32(desc->dma_addr + desc->tx_base, slot_base+AXI_DMA_BASE_OFFSET + AXI_DMA_TX_OFFSET + AXI_DMA_ADDRESS_OFFSET);
		
		iowrite8(1,slot_base + AXI_DMA_BASE_OFFSET + AXI_DMA_RX_OFFSET + AXI_DMA_CONTROL_OFFSET );
		iowrite32(desc->dma_addr + desc->rx_base, slot_base+AXI_DMA_BASE_OFFSET + AXI_DMA_RX_OFFSET + AXI_DMA_ADDRESS_OFFSET);
		
		
		iowrite32(desc->rx_length, slot_base+AXI_DMA_BASE_OFFSET + AXI_DMA_RX_OFFSET + AXI_DMA_LENGTH_OFFSET);
		iowrite32(desc->tx_length, slot_base+AXI_DMA_BASE_OFFSET + AXI_DMA_TX_OFFSET + AXI_DMA_LENGTH_OFFSET);
		
		return 0;		
}
	// --------------------- dma buffer subsys end ---------------------//

static void dev_vm_open(struct vm_area_struct * area)
{
	struct dev_dma_buffer_desc *desc = (struct dev_dma_buffer_desc *)area->vm_private_data;
	
	dev_info(misc->this_device,"Openinng VM region for page: 0x%08x.\n", area->vm_pgoff);
	// Incrementing rerference counter
	dev_dma_buffer_get(desc);
	
}
static void dev_vm_close(struct vm_area_struct * area)
{
	struct dev_dma_buffer_desc *desc = (struct dev_dma_buffer_desc *)area->vm_private_data;
	dev_info(misc->this_device,"Closing VM region for page: 0x%08x.\n",area->vm_pgoff);
	dev_dma_buffer_put(desc);
}

struct vm_operations_struct dev_vm_ops = {
	.open = dev_vm_open,
	.close = dev_vm_close
};

static int dev_mmap (struct file *pfile, struct vm_area_struct *vma)
{
	struct virtual_dev_t *vdev;
	struct dev_dma_buffer_desc *buf_desc;
	
	//Getting current dma buffer
	// TODO lock the vdev pointer
	vdev = (struct virtual_dev_t*)pfile->private_data;
	if(vdev->dma_buffer_desc == NULL)
	{
		// Allocate a new dma buffer, if it has not done yet
		buf_desc = dev_dma_buffer_alloc(MMAP_MAX_BUFFER_LENGTH);
		if(!buf_desc)
		{
			return -ENOMEM;
		}
		vdev->dma_buffer_desc = buf_desc;
	}
	
	// Remap the buffer to the user space virtual memory
	dev_info(misc->this_device,"Remapping dma coherent buffer.\n");
	if(remap_pfn_range(vma, vma->vm_start, buf_desc->dma_addr >> PAGE_SHIFT, buf_desc->length, vma->vm_page_prot))
	{
		return -EAGAIN;
	}
	// Overwriting vma operations
	vma->vm_ops = &dev_vm_ops;
	vma->vm_private_data = buf_desc;
	dev_dma_buffer_get(buf_desc);

	return 0;
}

static struct file_operations dev_fops =
{
		.owner = THIS_MODULE,
		.open = dev_open,
		.release = dev_close,
		.unlocked_ioctl = dev_ioctl,
		.read = dev_read,
		.mmap = dev_mmap
};



/* 	************************************************************************
	************************************************************************
	****					MODULE INIT / EXIT							****
	************************************************************************
	************************************************************************
*/
static int fpga_sched_init(void)
{
	int ret;
	int i;
	pr_info("Starting fpga scheduler module.\n");

	if(build_device_modell())
	{
		pr_err("Cannot build device modell.\n");
		goto err;
	}

	// build database
	if(gather_slot_data())
	{
		pr_err("Failed to init slot data.\n");
		goto err1;
	}

	if(gather_accel_data())
	{
		pr_err("Failed to load accel data.\n");
		goto err2;
	}

	if(connect_to_fpga_mgr())
	{
		pr_err("Cannot init fpga manager.\n");
		goto err3;
	}


	// register misc device in the system
	misc.fops = &dev_fops;
	misc.minor = MISC_DYNAMIC_MINOR;
	misc.name = "fpga_mgr";
	if(misc_register(&misc))
	{
		pr_warn("Couldn't initialize miscdevice /dev/fpga_mgr.\n");
		goto err4;
	}
	pr_info("Misc device initialized: /dev/fpga_mgr.\n");

	// create event handler kernel thread
	sched_thread = kthread_run(sched_thread_fn,NULL,"fpga_sched");
	if(IS_ERR(sched_thread))
	{
		pr_err("Scheduler thread failed to start.\n");
		goto err5;
	}


	procfs_init();

	return 0;
	err5:
		misc_deregister(&misc);
	err4:
		disconnect_from_fpga_mgr();
	err3:
		free_accel_data();
	err2:
		free_slot_data();
	err1:
		clean_device_modell();
	err:
	return -1;
}

static void fpga_sched_exit(void)
{
	//send stop signal to the kernel thread
	quit = 1;
	complete(&event_in);
	wait_for_completion(&thread_stop);

	procfs_destroy();
	misc_deregister();
	disconnect_from_fpga_mgr();
	free_accel_data();
	free_slot_data();
	clean_device_modell();
	pr_info("Fpga scheduler module exited.\n");
}


/* ======================================================================== 
					LINUX DEVICE MODELL HANDLER

	static int build_device_modell(void);
	static void clean_device_modell(void);
   ========================================================================*/


static int fpga_virtual_bus_match(struct device *dev, struct device_driver *drv)
{
	if(dev->bus == &fpga_virtual_bus_type) return 1;
	return 0;
}

static struct bus_type fpga_virtual_bus_type =
{
		.name="fpga_virtual_bus",
		.match = fpga_virtual_bus_match
};


// BUS DRIVER
static int fpga_virtual_driver_probe(struct device *dev)
{
	if(dev->bus == &fpga_virtual_bus_type) return 1;
	return 0;
}
static struct device_driver fpga_virtual_driver =
{
		.name = "fpga_virtual_driver",
		.bus = &fpga_virtual_bus_type,
		.owner = THIS_MODULE,
		.probe = fpga_virtual_driver_probe
};

// DEVICE ATTRIBUTE
static ssize_t id_show (struct device *dev, struct device_attribute *attr, char *buf)
{
	struct dev_priv *p = (struct dev_priv*)dev_get_drvdata(dev);
	return sprintf(buf,"%d",p->acc_id);
}
DEVICE_ATTR(id,0444,id_show,NULL);

static int build_device_modell(void)
{
	int ret = 0;

	// registering fpga virtual bus
	if(bus_register(&fpga_virtual_bus_type))
	{
		pr_err("Cannot register fpga_virtual bus.\n");
		ret = -ENODEV;
		goto err;
	}

	// register fpga_virtual_bus device
	//fpga_virtual_bus.of_node = base;
	dev_set_name(&fpga_virtual_bus,"fpga_virtual_bus_device");
	if(device_register(&fpga_virtual_bus))
	{
		pr_err("Cannot register fpga_virtual_bus device.\n");
		ret = -ENODEV;
		goto err1;
	}

	// register fpga virtual bus driver
	if(driver_register(&fpga_virtual_driver))
	{
		pr_err("Cannot register fpga_virutal driver.\n");
		ret = -1;
		goto err2;
	}

	return 0;

err2:
	device_unregister(&fpga_virtual_bus);
err1:
	bus_unregister(&fpga_virtual_bus_type);
err:
	return ret;
}

static void clean_device_modell(void)
{
	driver_unregister(&fpga_virtual_driver);
	device_unregister(&fpga_virtual_bus);
	bus_unregister(&fpga_virtual_bus_type);	
}

/* ======================================================================== 
							SLOT DATA MANAGER

	static int gather_slot_data(void);
	static void free_slot_data(void);
	static void print_slot_data(void);
   ========================================================================*/
/* Target global variables:
	DEFINE_SPINLOCK(slot_lock);
	unsigned int SLOT_NUM;
	unsigned free_slot_num;
	struct slot_info_t *slot_info;
	*/
static int gather_slot_data(void)
{
	struct device_node *base;
	struct device_node *slot;
	struct resource res;
	int i;
	int slot_no;
	void *slot_kaddr;
	int ok;

	// initialize slot database
	SLOT_NUM = 0;
	free_slot_num = 0;
	slot_info = NULL;

	ret = 0;

	// search the device tree for the data
	base = of_find_node_by_name(NULL,"fpga_virtual_slots");
	if(!base)
	{
		pr_err("FPGA_VIRTUAL_SLOTS node not found. Terminating...\n");
		ret = -1;
		goto err0;
	}
	
	// COUNTING SLOTS
	slot_no = of_get_child_count(base);
	if(slot_no < 1)
	{
		pr_err("No slots found.\n");
		ret = -ENODEV;
		goto err1;
	}
	pr_info("%d slots found.\n",SLOT_NUM);

	// allocate memory for the slot info structures
	slot_info = (struct slot_info_t*)kmalloc(sizeof(struct slot_info_t) * slot_no, GFP_KERNEL);
	if(!slot_info)
	{
		pr_err("Cannot allocate memory for the slot descriptors.\n");
		ret = -ENOMEM;
		goto err1;
	}


	// ACQUIRING SLOT PARAMETERS (static fpga region data) from the device tree
	i=0;
	for_each_child_of_node(base,slot)
	{
		ok = 0;
		// allocate static region addresses for the slot
		if(of_address_to_resource(slot,0,&res))
		{

			if(request_mem_region(res.start, resource_size(&res), "FPGA ACCELERATOR SLOT") != NULL)
			{
				slot_kaddr = of_iomap(slot,0);
				if(slot_kaddr)
				{
					// address mapping succeded
					slot_info[i].base_address = res.start;
					slot_info[i].address_length = resource_size(&res);
					slot_info[i].slot_kaddr = slot_kaddr;
					ok = 1;
				}
				else
					release_mem_region(res.start, resource_size(&res));
			}	
		}
			// check for success
			if(!ok)
			{
				pr_err("Cannot acquire slot %d base address.\n",i);
				slot_info[i].base_address = 0;
				slot_info[i].address_length = 0;
				slot_info[i].slot_kaddr = NULL;
				ret = -1;
			}

		slot_info[i].user = NULL;
		slot_info[i].status = SLOT_FREE;
		slot_info[i].actual_dev = NULL;
		slot_info[i].busy = 0;
		slot_info[i].compatible_accel_num = 0;
		slot_info[i].compatible_accels = NULL;
			i++;
	} //foreach


	// Exporting SLOT counter

	free_slot_num = SLOT_NUM = slot_no;

	err1:
		of_node_put(base);
	err0:
		return ret;
}

static void free_slot_data(void)
{
	int i;

	// FREE interrupt lines

	// FREE address mappings
	for(i=0;i<SLOT_NUM;i++)
	{
		if(slot_info[i].slot_kaddr)
		{
			iounmap(slot_info[i].kaddr);
			release_mem_region(slot_info[i].base_address, slot_info[i].address_length);
		}
		if(slot_info[i].compatible_accels != NULL)
			kfree(slot_info[i].compatible_accels);
	}

	kfree(slot_info);
}

static void print_slot_data(void)
{
	int i,j;
	for(i=0;i<SLOT_NUM;i++)
	{
		printk("Slot no: %d,\tslot base address: 0x%08x,\tslot address length: 0x%x,associated accel num: %d.\n",i,slot_info[i].base_address, slot_info[i].address_length, slot_info[i].compatible_accel_num);
		for(j=0;j<slot_info[i].compatible_accel_num;j++)
			printk("Slot no --%d-- is compatible with accel --%d--.\n",i,slot_info[i].compatible_accels[j]->acc_id);
		printk("There are %d slots and %d accelerators in the system.\n",SLOT_NUM,device_num);
	}
}

/* ======================================================================== 
							ACCELERATORS MANAGER

	static int gather_accel_data(void);
	static void free_accel_data(void);
   ========================================================================*/

static int gather_accel_data(void)
{
	struct device_node *base;
	struct device_node *acc;
	int i,j;
	int ret;
	uint32_t accel_num;
	uint8_t *text_prop;
	uint32_t *word_prop;
	int prop_len;
	uint8_t name_buf[64] = {0};

	
	// init the global variables
	device_num = 0;
	device_data = NULL;
	devices = NULL

	base = of_find_node_by_name(NULL,"fpga_virtual_accels");
	if(!base)
	{
		pr_err("FPGA_VIRTUAL_ACCELS node not found. Terminating...\n");
		ret = -1;
		goto err;
	}


	accel_num = of_get_child_count(base);
	if(accel_num<1)
	{
		pr_err("No accelerators found.\n");
		ret = -ENODEV;
		goto err1;
	}
	pr_info("%d accelerators found.\n",accel_num);	


	// allocate structures for all the dev_privs
	device_data = (struct dev_priv*)kmalloc(device_num*sizeof(struct dev_priv),GFP_KERNEL);
	if(!device_data)
	{
		ret = -ENOMEM;
		goto err1;
	}
	// allocate device_data
	devices = (struct device*)kzalloc(device_num*sizeof(struct device),GFP_KERNEL);
	if(!devices)
	{
		ret = -ENOMEM;
		goto err2;
	}

	// build compatible accel database for the slots
	for(i=0;i<SLOT_NUM;i++)
	{
		slot_info[i].compatible_accels = (struct dev_priv**)kmalloc(accel_num* sizeof(struct dev_priv*));
		if(slot_info[i].compatible_accels == NULL)
		{
			pr_err("Cannot allocate memory for compatible accel list.\n");
			continue;
		}
	}

	// iterate over the child nodes, and fill the corresponing structures
	i=0;
	for_each_child_of_node(base,acc)
	{
		// get accel name property
		text_prop = of_get_property(acc,"fpga_virtual_config_name",&prop_len);
		if(text_prop==NULL || prop_len==0)
		{
			pr_err("Cannot get accelerator name.\n");
			dev_set_name(&devices[i],"%s",acc->name);
		}
		else
		{
			strncpy(name_buf, text_prop, 63);
			dev_set_name(&devices[i],"%s",name_buf);
		}

		// get compatible slot list
		word_prop = of_get_property(acc,"fpga_virtual_compatible_slots",&prop_len);
		if(word_prop==NULL || prop_len==0)
		{
			pr_err("Cannot get compatible slot no.\n");
		}
		else
		{
			// iterate over the compatible slots
			pr_info("Compatible slot count: %d.\n",prop_len);
			for(j=0;j<prop_len;j++)
			{
				uint32_t slot_index = word_prop[j];
				pr_info("Accel %d is compatible with slot %d.\n",i,slot_index);

				if(slot_index < SLOT_NUM)
				{
					uint32_t idx = slot_info[slot_index].compatible_accel_num;
					slot_info[slot_index].compatible_accels[idx] = &device_data[i];
					slot_info[slot_index].compatible_accel_num++;
				}
				else
				{
					pr_err("Invalid slot index: %d",slot_index);
				}
			}
		}

		devices[i].bus = &fpga_virtual_bus_type;
		devices[i].driver = &fpga_virtual_driver;
		devices[i].of_node = acc;
		of_node_get(acc);
		devices[i].parent = &fpga_virtual_bus;
		if(device_register(&devices[i]))
		{
			pr_err("Unable to register device.\n");
		}

		device_data[i].acc_id = i;
		INIT_LIST_HEAD(&(device_data[i].active_list));
		INIT_LIST_HEAD(&(device_data[i].waiters));
		device_data[i].dev = &devices[i];

		dev_set_drvdata(&devices[i],(void*)&device_data[i]);

		// add device attribute
		device_create_file(&devices[i],&dev_attr_id);
		i++;
	}

	of_node_put(base);
	// export data
	device_num = accel_num;
	return 0;

	err3:
	kfree(devices);
	err2:
	kfree(device_data);
	err1:
	of_node_put(base);
	err:
	return ret;

}

static void free_accel_data(void)
{
	int i;
	/*
	// TODO SAFETY wake all waiting processes
	*/
	for(i=0;i<device_num;i++)
	{
		device_remove_file(&devices[i],&dev_attr_id);
		of_node_put(devices[i].of_node);
		device_unregister(&devices[i]);
	}

	kfree(device_data);
	kfree(devices);

}


/* ======================================================================== 
							FPGA MGR ADAPTER

	static int connet_to_fpga_mgr(void);
	static void disconnet_from_fpga_mgr(void);
   ========================================================================*/

static int connect_to_fpga_mgr(void)
{
	struct device_node *devcfg;
	int ret = 0;

	// find devcfg device
	devcfg = of_find_node_by_name(NULL,"devcfg");
	if(!devcfg)
	{
		pr_err("Can't find devcfg node in device tree.\n");
		ret = -ENODEV;
		goto err;
	}
	mgr = of_fpga_mgr_get(devcfg);
	of_node_put(devcfg);
	if(IS_ERR(mgr))
	{
		pr_err("Cannot get fpga manager.\n");
		ret = -ENODEV;
	}

	// program fpga with the static config
	ret = program_fpga(FPGA_STATIC_CONFIG_NAME);
err:
	return ret;
}

static void disconnect_from_fpga_mgr(void)
{
	fpga_mgr_put(mgr);
}


/* ======================================================================== 
							PROCFS INTERFACE

	static int build_device_modell(void);
	static void clean_device_modell(void);
   ========================================================================*/
// procfs interface for slot status observation

#define PROCFS_NAME "fpga_mgr"
static struct proc_dir_entry *proc_file;
static struct file_operations proc_fops;

static int procfs_init(void)
{
	// register procfs interface
	proc_file = proc_create(PROCFS_NAME,0444,NULL,&proc_fops);
	return 0;
}
static void procfs_destroy(void)
{
	proc_remove(proc_file);
}

char log_buf[256];
ssize_t procfile_read(struct file *file, char __user *buffer, size_t bufsize, loff_t * offset)
{
	int i;
	int len = 0;

	spin_lock(&slot_lock);
	for(i=0;i<SLOT_NUM;i++)
	{
		int acc_num = slot_info[i].actual_dev ? slot_info[i].actual_dev->acc_id : -1;
		len += sprintf(log_buf,"slot %d \t status: %d \t accel: %d\n",i,slot_info[i].status,acc_num);
	}
	spin_unlock(&slot_lock);


	if(*offset>=len) return 0;

	if(copy_to_user(buffer,log_buf,len))
		return -EFAULT;
	*offset += len;
	return len;
}

static struct file_operations proc_fops =
{
		.owner = THIS_MODULE,
		.read = procfile_read
};



module_init(fpga_sched_init);
module_exit(fpga_sched_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tusori Tibor");
MODULE_DESCRIPTION("FPGA scheduler");
