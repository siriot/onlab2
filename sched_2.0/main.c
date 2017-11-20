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
 		devtree kiterjesztés + image.ub ujracsomagolása
 		reset bennt lévő slothoz ütemezéskor is
		dma csatornák interrupt számai(melyik a tx, melyik az rx)
		slot init errornál bennt maradnak a rossz slotok, nem száll el a modul init
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
#include <linux/byteorder/generic.h>
#include <linux/interrupt.h>

#include <linux/fpga/fpga-mgr.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#include <linux/fs.h>
#include <linux/miscdevice.h>

#include <asm/ioctl.h>

#include <linux/platform_device.h>

#include <linux/dma-mapping.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/ioport.h>

#include <linux/iopoll.h>



#define DRIVER_NAME "FPGA_scheduler"

/* Resource locking policy:
	// dev_close
 * virt_dev -> event queue -> slot
 * Locking may only happen in this direction. !!!
 *
 * if vdev->status == OPERATING && vdev->slot->status == OPERATING than the pairing is guaratied!!!
 */

struct dma_channel_t{
	unsigned int irq; 		// interrupt line of the channel
	void __iomem *base;		// dma base address for the channel
	struct completion comp;	// channel ready signal
	uint32_t error;			// error code, signalling the result of the transfer
	uint32_t busy;			// dma transfer is happening
};

struct dma_device_t{
	void __iomem *base;					// base address for the dma
	struct dma_channel_t tx_channel;
	struct dma_channel_t rx_channel;
};


struct dev_priv
{
	int acc_id;
	struct device *dev;
	struct list_head waiters;
	struct list_head active_list;
	char name[32];
};
struct virtual_dev_t;

enum slot_status_t {SLOT_FREE, SLOT_USED};
struct slot_info_t
{
	struct virtual_dev_t *user;
	enum slot_status_t status;
	struct dev_priv *actual_dev;

	// slot virtual address
	uint8_t *slot_kaddr;
	// slot physical address range
	uint32_t base_address;
	uint32_t address_length;

	struct dma_device_t dma;		// corresponding dma device

	// compatible accelerator list
	uint32_t compatible_accel_num;
	struct dev_priv **compatible_accels;
};

/* 	Buffer descpriptor format.
 *	User space program describes the dma request in this form, and then executes
 *	a dma start ioctl with a pointer to such a structure.
 */
struct user_dma_buffer_desc
{
	u32 tx_offset;
	u32 rx_offset;
	u32 tx_length;
	u32 rx_length;
};

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
	// buffer properties
	uint8_t *kaddr;
	uint32_t length;
	// requeset properties
	struct user_dma_buffer_desc user_request;
	// physical address of the buffers
	dma_addr_t tx_base;		// mapped physical address for the tx buffer. 0 if inmapped
	dma_addr_t rx_base;		// mapped physical address for the rx buffer. 0 if unmapped
};

// Contains information about the device session.
// This structure is linked to the file descriptor and thus to the session.
//#define DEV_STATUS_BLANK 		0
// user thread changes to QUEUED
//#define DEV_STATUS_QUEUED 		1
/*
 * Slot connection is ready!!!
 * kthread changes to operating
 */
//#define DEV_STATUS_OPERATING	2
// kthread or user thread changes state to releasing or closing
//#define DEV_STATUS_RELEASING	3
//#define DEV_STATUS_CLOSING		4
enum virtual_dev_status_t {DEV_STATUS_BLANK, DEV_STATUS_QUEUED, DEV_STATUS_OPERATING, 
							DEV_STATUS_RELEASING,DEV_STATUS_CLOSING};
struct virtual_dev_t
{
	// virtual device status
	enum virtual_dev_status_t status;
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

//#define EVENT_REQUEST 	1
//#define EVENT_CLOSE		2
enum user_event_type_t{ EVENT_REQUEST=1, EVENT_CLOSE=2};
struct user_event
{
	enum user_event_type_t event_type;
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
	#define AXI_DMA_STATUS_OFFSET			(0x04U)
	#define AXI_DMA_ADDRESS_OFFSET			(0x18U)
	#define AXI_DMA_LENGTH_OFFSET			(0x28U)
#define AXI_DECOUPLER_BASE_OFFSET		(0x1000U)
#define AXI_RST_BASE_OFFSET				(0x2000U)
#define AXI_ACCEL_BASE_OFFSET			(0x3000U)

//#define XILINX_DMA_DMASR_EOL_LATE_ERR		BIT(15)//
#define XILINX_DMA_DMASR_ERR_IRQ			BIT(14)
#define XILINX_DMA_DMASR_DLY_CNT_IRQ		BIT(13)
#define XILINX_DMA_DMASR_IOC				BIT(12)
//#define XILINX_DMA_DMASR_SOF_LATE_ERR		BIT(11)//
#define XILINX_DMA_DMASR_SG_DEC_ERR			BIT(10)
#define XILINX_DMA_DMASR_SG_SLV_ERR			BIT(9)
//#define XILINX_DMA_DMASR_EOF_EARLY_ERR		BIT(8) //
//#define XILINX_DMA_DMASR_SOF_EARLY_ERR		BIT(7) //
#define XILINX_DMA_DMASR_DMA_DEC_ERR		BIT(6)
#define XILINX_DMA_DMASR_DMA_SLAVE_ERR		BIT(5)
#define XILINX_DMA_DMASR_DMA_INT_ERR		BIT(4)
#define XILINX_DMA_DMASR_IDLE				BIT(1)
#define XILINX_DMA_DMASR_HALTED				BIT(0)
#define XILINX_DMA_DMASR_DELAY_MASK			GENMASK(31, 24)
#define XILINX_DMA_DMASR_FRAME_COUNT_MASK	GENMASK(23, 16)

#define XILINX_DMA_DMASR_ALL_ERR_MASK	\
		 (XILINX_DMA_DMASR_ERR_IRQ | \
		 XILINX_DMA_DMASR_SG_DEC_ERR | \
		 XILINX_DMA_DMASR_SG_SLV_ERR | \
		 XILINX_DMA_DMASR_DMA_DEC_ERR | \
		 XILINX_DMA_DMASR_DMA_SLAVE_ERR | \
		 XILINX_DMA_DMASR_DMA_INT_ERR)

#define AXI_DMA_DMACR_RS				BIT(0)
#define AXI_DMA_DMACR_RST				BIT(2)
#define AXI_DMA_DMACR_ERR_IRQ			BIT(14)
#define AXI_DMA_DMACR_IOC_IRQ			BIT(12)

/* Bitfile nameing conventions:
 * 	static: static.bit
 * 	accels: <accel_name from dev tree>_slotx.bit; x= slot idex (0-)
 */
#define FPGA_STATIC_CONFIG_NAME "static.bit"

/**
 * lock policy: only kthread can get more than one spin lock, other threads may only get 1!
 */


//-----------------------------//
//		 GLOBAL VARIABLES
//-----------------------------//

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
	struct platform_device * platform_dev; // this comes from the device tree
	// for dev_err(...)
	#define sdev (&(platform_dev->dev))
	static struct device fpga_virtual_bus;
	static struct device *devices;

	
	/***************************************
		Internal functions
	***************************************/
	
	static struct dev_dma_buffer_desc* dev_dma_buffer_alloc(uint32_t length);
	static void dev_dma_buffer_get(struct dev_dma_buffer_desc* desc);
	static void dev_dma_buffer_put(struct dev_dma_buffer_desc* desc);
	static int dev_dma_buffer_map(struct dev_dma_buffer_desc *desc);
	static void dev_dma_buffer_unmap(struct dev_dma_buffer_desc *desc);
	static int lock_operating_vdev(struct virtual_dev_t *vdev);
	static void unlock_operating_vdev(struct virtual_dev_t *vdev);
	
	
	// database builder functions
	static int gather_slot_data(void);
	static void free_slot_data(void);
	static void print_slot_data(void);
	static int gather_accel_data(void);
	static void free_accel_data(void);

	// fpga manager
	static int connect_to_fpga_mgr(void);
	static void disconnect_from_fpga_mgr(void);

	// linux device modell functions
	static int build_device_modell(struct platform_device *pdev);
	static void clean_device_modell(void);

	// procfs interface
	static int procfs_init(void);
	static void procfs_destroy(void);

	
	/* ======================================================================== 
					HARDWARE MANAGEMENT

	static void dev_dma_buffer_get(struct dev_dma_buffer_desc *desc);
	static void dev_dma_buffer_put(struct dev_dma_buffer_desc *desc);
	static struct dev_dma_buffer_desc* dev_dma_buffer_alloc(uint32_t length);
   ========================================================================*/


//#dmaapi
/* IO accessors */
static inline u8 dma_read8(struct dma_channel_t *chan, u32 reg) {return ioread8(chan->base + reg);}
static inline void dma_write8(struct dma_channel_t *chan, u32 reg, u8 value) {iowrite8(value, chan->base + reg);}
static inline u32 dma_read(struct dma_channel_t *chan, u32 reg) {return ioread32(chan->base + reg);}
static inline void dma_write(struct dma_channel_t *chan, u32 reg, u32 value) {iowrite32(value, chan->base + reg);}
#define dma_poll_timeout(chan, reg, val, cond, delay_us, timeout_us) \
	readl_poll_timeout(chan.base+reg, val, \
			   cond, delay_us, timeout_us)

// data - struct slot_info_t pointer to the actual slot
static irqreturn_t dma_irq_handler(int irq, void *data)
{
	struct dma_channel_t *chan = (struct dma_channel_t*)data;
	uint32_t dma_status;

	dma_status = dma_read(chan, AXI_DMA_STATUS_OFFSET);
	if((dma_status & (AXI_DMA_DMACR_IOC_IRQ | AXI_DMA_DMACR_ERR_IRQ)) == 0)
		return IRQ_NONE;

	// clear interrupt flag
	dma_write(chan,AXI_DMA_STATUS_OFFSET, dma_status);
	if(dma_status & XILINX_DMA_DMASR_ALL_ERR_MASK)
		chan->error = dma_status;
	else
		chan->error = 0;

	complete_all(&(chan->comp));
	chan->busy = 0;
	return IRQ_HANDLED;
}

static int dma_channel_init(struct dma_channel_t *chan, void __iomem * base, int irq_line)
{
	int ret;

	chan->base = base;
	chan->error = 0;
	chan->busy = 0;
	init_completion(&(chan->comp));
	chan->irq = irq_line;
	ret = request_irq(irq_line, dma_irq_handler, IRQF_SHARED,
				  "fpga_sched_dma_irq_hander", chan);
	if(ret)
		dev_err(sdev,"Cannot request irq line.\n");
	
	return ret;
}

static void dma_channel_destroy(struct dma_channel_t *chan)
{
	free_irq(chan->irq,chan);
}

static void dma_channel_irq_enable(struct dma_channel_t *chan)
{
	uint32_t reg;
	reg = dma_read(chan, AXI_DMA_CONTROL_OFFSET);
	dma_write(chan, AXI_DMA_CONTROL_OFFSET, reg | AXI_DMA_DMACR_ERR_IRQ | AXI_DMA_DMACR_IOC_IRQ);
}

static void dma_channel_irq_disable(struct dma_channel_t *chan)
{
	uint32_t reg;
	reg = dma_read(chan, AXI_DMA_CONTROL_OFFSET);
	dma_write(chan, AXI_DMA_CONTROL_OFFSET, reg &( ~(AXI_DMA_DMACR_ERR_IRQ | AXI_DMA_DMACR_IOC_IRQ)));
}

static int dma_reset(struct dma_device_t *dma)
{
	int err;
	u32 tmp;
	dma_write(&(dma->tx_channel),AXI_DMA_CONTROL_OFFSET,AXI_DMA_DMACR_RST);
	err = dma_poll_timeout(dma->tx_channel, AXI_DMA_CONTROL_OFFSET, tmp, ((tmp&AXI_DMA_DMACR_RST)==0),1,1000);
	return err;
}
static int dma_init(struct dma_device_t *dma, uint8_t __iomem *base, int tx_irq, int rx_irq)
{
	int ret = 0;
	int err;
	// fill dma data structures
	dma->base = base = base;
	// tx channel
	ret = dma_channel_init(&(dma->tx_channel), dma->base + AXI_DMA_TX_OFFSET, tx_irq);
	if(ret)
	{
		dev_err(sdev,"DMA TX IRQ init error.\n");
		return -1;
	}

	dma_channel_init(&(dma->rx_channel), dma->base + AXI_DMA_RX_OFFSET, rx_irq);
	if(ret)
	{
		dev_err(sdev,"DMA RX IRQ init error.\n");
		dma_channel_destroy(&(dma->tx_channel));
		return -1;
	}


	// reset the dma
	err = dma_reset(dma);
	if(err)
	{
		dev_err(sdev,"Cannot reset dma.\n");
		return -1;
	}
	// enable irqs
	dma_channel_irq_enable(&(dma->tx_channel));
	dma_channel_irq_enable(&(dma->rx_channel));
	return 0;
}

static void dma_destroy(struct dma_device_t *dma)
{
	dma_channel_irq_disable(&(dma->tx_channel));
	dma_channel_irq_disable(&(dma->rx_channel));
	dma_channel_destroy(&(dma->tx_channel));
	dma_channel_destroy(&(dma->rx_channel));
}

#define DMA_WAIT_TIMEOUT		5000
static int dma_wait_channel_ready(struct dma_channel_t *chan)
{
	long s;
	int ret;

	// channel is allready ready
	if(chan->busy == 0)
		return 0;

	s = wait_for_completion_killable_timeout(&(chan->comp),DMA_WAIT_TIMEOUT);
	if(s<0)
	{
		dev_err(sdev,"Waiting for dma channel interrupted.\n");
		ret = -1;
	}
	else if(s== 0)
	{
		dev_err(sdev,"Waiting for dma chanel timeouted.\n");
		ret = -2;
	}
	else if(chan->error)
	{
		dev_err(sdev,"Dma channel error: 0x%08x",chan->error);
		ret = -3;
	}
	else ret = 0 ;

	return ret;
}

static int dma_start_channel(struct dma_channel_t *chan, uint32_t addr, uint32_t len)
{
	uint32_t reg;
	if(chan->busy == 1)
		return -1;
	chan->busy = 1;
	reinit_completion(&(chan->comp));
	reg = dma_read(chan, AXI_DMA_CONTROL_OFFSET);
	dma_write8(chan, AXI_DMA_CONTROL_OFFSET,reg | AXI_DMA_DMACR_RS);
	dma_write(chan,  AXI_DMA_ADDRESS_OFFSET, addr);
	dma_write(chan,  AXI_DMA_LENGTH_OFFSET, len);
	return 0;
}
static int dma_wait_transfer_ready(struct slot_info_t *slot)
{
	int tx_ok, rx_ok;

	if(dma_wait_channel_ready(&(slot->dma.tx_channel)))
	{
		dev_err(sdev,"DMA TX failed.\n");
		tx_ok = 0;
	}
	else
	{
		tx_ok = 1;
	}

	if(dma_wait_channel_ready(&(slot->dma.rx_channel)))
	{
		dev_err(sdev,"DMA RX failed.\n");
		rx_ok = 0;
	}
	else 
	{
		tx_ok = 1;
	}

	return ~(tx_ok && rx_ok);
}

static int dma_start_tx_rx(struct slot_info_t *slot, uint32_t tx_base, uint32_t rx_base, uint32_t tx_len, uint32_t rx_len)
{
	dma_start_channel(&(slot->dma.rx_channel),rx_base,rx_len);
	dma_start_channel(&(slot->dma.tx_channel),tx_base,tx_len);
	return 0;
}



//#devcfgapi
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
	int ret;
	char config_file_name[100];

	if(slot_num < 0 || slot_num >= SLOT_NUM) return -1;
	// stop the previous accelerator
	// reset accelerator
	iowrite8(1,slot_info[slot_num].slot_kaddr + AXI_RST_BASE_OFFSET);
	// decouple accelerator
	iowrite8(1, slot_info[slot_num].slot_kaddr + AXI_DECOUPLER_BASE_OFFSET);
	// reset TX and RX DMA channels
	dma_reset(&(slot_info[slot_num].dma));


	
	snprintf(config_file_name,100,"%s_slot%d.bit",device_data[acc_id].name,slot_num);
	// start programming
	pr_info("Loading configuration: %s - id :%d.\n",config_file_name,acc_id);
	ret = fpga_mgr_firmware_load(mgr, FPGA_MGR_PARTIAL_RECONFIG, config_file_name);
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
					dev = slot_info[i].compatible_accels[j];
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
							if(vdev->dma_buffer_desc != NULL) dev_dma_buffer_put(vdev->dma_buffer_desc);
							kfree(vdev);
							break;
						case DEV_STATUS_QUEUED:
							list_del(&(vdev->waiter_list));
							if(vdev->dma_buffer_desc != NULL) dev_dma_buffer_put(vdev->dma_buffer_desc);
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
							if(vdev->dma_buffer_desc != NULL)  dev_dma_buffer_put(vdev->dma_buffer_desc);
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
#define IOCTL_AXI_READ			20
#define IOCTL_AXI_WRITE			3
#define IOCTL_DMA_START			8
#define IOCTL_DMA_WAIT_FOR_RDY	9

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
	uint8_t *hw_addr;
	
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
					ret = -EPERM;

				}
				unlock_operating_vdev(vdev);
			}
			else
			{
				ret = -EPERM;
			}

			if(ret == -EPERM)
				pr_err("Unauthorized slot access.\n");

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
			pr_info("ACCEL READ with address: %d, data: %u",address,data);
			// check read permission
			if(address + AXI_ACCEL_BASE_OFFSET>= vdev->slot->address_length) return -EPERM;

			if(!lock_operating_vdev(vdev))
			{
				// Physical address = slot base + accel base offset + offset
				hw_addr = vdev->slot->slot_kaddr + AXI_ACCEL_BASE_OFFSET + address;
				data = ioread32(hw_addr);
				
				unlock_operating_vdev(vdev);
			}
			else
			{
				pr_err("Accelerator is not working.\n");
				ret = -ENODEV;
			}

			if(copy_to_user((void*)arg,&data,sizeof(data)))
			{
				dev_err(sdev,"Cannot copy data to user space.\n");
				return -ENOMEM;
			}
			return 0;
			break;
		case IOCTL_AXI_WRITE:
		pr_info("ACCEL WRITE with address: %d, data: %u",address,data);
			// check write permission
			if(address  + AXI_ACCEL_BASE_OFFSET >= vdev->slot->address_length) return -EPERM;

			if(!lock_operating_vdev(vdev))
			{
				uint8_t *hw_addr;
				// Physical address = slot base + accel base offset + offset
				hw_addr = vdev->slot->slot_kaddr + AXI_ACCEL_BASE_OFFSET + address;
				iowrite32(data,hw_addr);	
				ret = 0;
				
				unlock_operating_vdev(vdev);
			}
			else
			{
				pr_err("Accelerator is not working.\n");
				ret = -ENODEV;
			}
			return ret;
			break;
		case IOCTL_DMA_START:
		{
			struct dev_dma_buffer_desc *desc = vdev->dma_buffer_desc;
			struct user_dma_buffer_desc *rqst = &(desc->user_request);
			if(copy_from_user(rqst, (void*)arg, sizeof(struct user_dma_buffer_desc)))
			{
				dev_err(sdev,"Cannot copy user dma request from user space.\n");
				return -ENOMEM;
			}

			// Validate rx and rx parameters
			if((rqst->tx_offset+rqst->tx_length > desc->length) || 
				(rqst->rx_offset+rqst->rx_length > desc->length) || 
				(rqst->tx_offset == rqst->rx_offset) ||
				((rqst->tx_offset<rqst->rx_offset) && (rqst->tx_offset+rqst->tx_length>rqst->rx_offset))|| 
				((rqst->rx_offset<rqst->tx_offset) && (rqst->rx_offset+rqst->rx_length>rqst->tx_offset))||
				((rqst->tx_offset & 0x3F) !=0) || 
				((rqst->rx_offset & 0x3F) !=0))
			{
				dev_err(sdev,"Invalid dma request.\n");
				return -EINVAL;
			}

			if(dev_dma_buffer_map(desc))
				return -EINVAL;

			if(!lock_operating_vdev(vdev))
			{
				ret = dma_start_tx_rx(vdev->slot,
					desc->tx_base,	// tx base
					desc->rx_base, // rx_base
					desc->user_request.tx_length,				// tx length
					desc->user_request.rx_length					// rx length
							);	
				unlock_operating_vdev(vdev);
			}
			else
			{
				pr_err("Accelerator is not working.\n");
				dev_dma_buffer_unmap(desc);
				ret = -ENODEV;
			}
			return ret;
		}
			break;
		case IOCTL_DMA_WAIT_FOR_RDY:
			if(!lock_operating_vdev(vdev))
			{
				ret = dma_wait_transfer_ready(vdev->slot);
				dev_dma_buffer_unmap(vdev->dma_buffer_desc);
				unlock_operating_vdev(vdev);
			}
			else
			{
				pr_err("Accelerator is not working.\n");
				ret = -ENODEV;
			}
			return 0;
			break;
		default:
			pr_err("Unknown ioctl command code: %u.\n",cmd);
			return -EINVAL;
			break;
	}
	return 0;
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

		if(desc==NULL) return;

		spin_lock(&(desc->lock));
		desc->ref_cntr--;
		cntr = desc->ref_cntr;
		spin_unlock(&(desc->lock));
		if(cntr==0)
		{
			// Free the buffer and the descriptor
			dev_info(misc.this_device,"Freeing coherent dma buffer.\n");
			free_page((unsigned long)(desc->kaddr));//dma_free_coherent(NULL, desc->length, desc->kaddr, desc->dma_addr);
			kfree(desc);		
		}
	}

	static struct dev_dma_buffer_desc* dev_dma_buffer_alloc(uint32_t length)
	{
		struct dev_dma_buffer_desc *desc;
		uint8_t *kbuf;
		
		dev_info(misc.this_device, "Allocating DMA buffer.\n");
		
		// TODO use the given length		
		length = MMAP_MAX_BUFFER_LENGTH;
		// Allocate coherent dma buffer
		kbuf = (uint8_t*)get_zeroed_page(0);//dma_alloc_coherent(NULL, length, &dma_addr, GFP_KERNEL);
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
			free_page((unsigned long)kbuf);//dma_free_coherent(NULL, length, kbuf, dma_addr);
			return NULL;
		}
		desc-> ref_cntr = 1;
		desc-> kaddr = kbuf;
		desc->length = length;
		desc-> tx_base = 0;
		desc->rx_base = 0;
		desc->user_request.tx_offset = 0;
		desc->user_request.rx_offset = 0;
		desc->user_request.tx_length = 0;
		desc->user_request.rx_length = 0;
		spin_lock_init(&(desc->lock));
		return desc;
	}

static void dev_dma_buffer_unmap(struct dev_dma_buffer_desc *desc)
{
	if(desc->tx_base > 0)
	{	
		dma_unmap_single(sdev, 
								desc->tx_base,
								desc->user_request.tx_length,
								DMA_TO_DEVICE);
		desc->tx_base = 0;
	}
	if(desc->rx_base >0)
	{
		dma_unmap_single(sdev, 
								desc->rx_base,
								desc->user_request.rx_length,
								DMA_FROM_DEVICE);
		desc->rx_base = 0;
	}
}

static int dev_dma_buffer_map(struct dev_dma_buffer_desc *desc)
{
	// map the dma buffers
	if(desc->tx_base == 0 && desc->tx_base == 0)
	{
		desc->tx_base = dma_map_single(sdev, 
										desc->kaddr + desc->user_request.tx_offset,
										desc->user_request.tx_length,
										DMA_TO_DEVICE);
		if(desc->tx_base == 0)
		{
			pr_err("Cannot map tx buffer.\n");
			return -1;
		}

		desc->rx_base = dma_map_single(sdev,
										desc->kaddr + desc->user_request.rx_offset,
										desc->user_request.rx_offset,
										DMA_FROM_DEVICE);
		if(desc->rx_base == 0)
		{
			pr_err("Cannot map rx buffer.\n");
			dma_unmap_single(sdev, 
								desc->tx_base,
								desc->user_request.tx_length,
								DMA_TO_DEVICE);
			return -1;
		}
	}
	return 0;
}

	// --------------------- dma buffer subsys end ---------------------//

static void dev_vm_open(struct vm_area_struct * area)
{
	struct dev_dma_buffer_desc *desc = (struct dev_dma_buffer_desc *)area->vm_private_data;
	
	dev_info(misc.this_device,"Openinng VM region for page: 0x%08lx.\n", area->vm_pgoff);
	// Incrementing rerference counter
	dev_dma_buffer_get(desc);
	
}
static void dev_vm_close(struct vm_area_struct * area)
{
	struct dev_dma_buffer_desc *desc = (struct dev_dma_buffer_desc *)area->vm_private_data;
	dev_info(misc.this_device,"Closing VM region for page: 0x%08lx.\n",area->vm_pgoff);
	dev_dma_buffer_put(desc);
}

static int dev_vm_fault(struct vm_area_struct *vma, struct vm_fault *vmf)
{
	struct dev_dma_buffer_desc *buf_desc = (struct dev_dma_buffer_desc *)vma->vm_private_data;
	//unsigned long physaddr = buf_desc->dma_addr;
	//unsigned long pageframe = physaddr >> PAGE_SHIFT;
	struct page *p = virt_to_page(buf_desc->kaddr);

	pr_info("Page fault.\n");

	get_page(p); //increase page reference counter
	vmf->page = p;
	return 0;
}

struct vm_operations_struct dev_vm_ops = {
	.open = dev_vm_open,
	.close = dev_vm_close,
	.fault = dev_vm_fault
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
	/*dev_info(misc.this_device,"Remapping dma coherent buffer.\n");
	if(remap_pfn_range(vma, vma->vm_start, buf_desc->dma_addr >> PAGE_SHIFT, buf_desc->length, vma->vm_page_prot))
	{
		return -EAGAIN;
	}*/
	// Overwriting vma operations
	vma->vm_ops = &dev_vm_ops;
	vma->vm_private_data = buf_desc;
	vma->vm_flags |= VM_DONTEXPAND;// | VM_RESERVED;
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


/* ======================================================================== 
					LINUX DEVICE MODELL HANDLER

	static int build_device_modell(void);
	static void clean_device_modell(void);
   ========================================================================*/

static struct bus_type fpga_virtual_bus_type;

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

static int build_device_modell(struct platform_device *pdev)
{
	int ret = 0;

	// saving the platform device pointer
	platform_dev = pdev;

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
	static int init_slot(struct device_node *of, struct slot_info_t *slot);
	static int gather_slot_data(void);
	static void free_slot_data(void);
	static void print_slot_data(void);
   ========================================================================*/
/*	of - pointer to the device node of the slot
 *	slot - pointer to the target slot_info_t structure
 */
static int init_slot(struct device_node *of, struct slot_info_t *slot)
{
	struct resource res;
	void *slot_kaddr;
	int tx_irq, rx_irq;

	// preinit 
	slot->user = NULL;
	slot->status = SLOT_FREE;
	slot->actual_dev = NULL;
	slot->compatible_accel_num = 0;
	slot->compatible_accels = NULL;

	// allocate static region addresses for the slot
	if(of_address_to_resource(of,0,&res))
		goto err;
	if(request_mem_region(res.start, resource_size(&res), "FPGA ACCELERATOR SLOT") == NULL);
		goto err;
	slot_kaddr = ioremap(res.start,resource_size(&res));
	if(!slot_kaddr)
		goto err1;

	// address mapping succeded
	slot->base_address = res.start;
	slot->address_length = resource_size(&res);
	slot->slot_kaddr = slot_kaddr;

	// request irq lines
	tx_irq = irq_of_parse_and_map(of, 0);
	rx_irq = irq_of_parse_and_map(of, 1);

	// init slot dma
	if(dma_init(&(slot->dma),slot_kaddr + AXI_DMA_BASE_OFFSET, tx_irq, rx_irq))
		goto err2;

	return 0;
err2:
	iounmap(slot->slot_kaddr);
err1:
	release_mem_region(res.start, resource_size(&res));
err:
	// safe state
	slot->slot_kaddr = NULL;
	slot->base_address = 0;
	slot->address_length = 0;
	dev_err(sdev,"Slot init failed.\n");
	return -1;
}
/* Target global variables:
	DEFINE_SPINLOCK(slot_lock);
	unsigned int SLOT_NUM;
	unsigned free_slot_num;
	struct slot_info_t *slot_info;
	*/
static int gather_slot_data(void)
{
	struct device_node *base;
	struct device_node *slot_node;
	int i;
	int slot_no;
	
	int ret;

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
	pr_info("%d slots found.\n",slot_no);

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
	for_each_child_of_node(base,slot_node)
	{
			if(init_slot(slot_node,&(slot_info[i])))
				dev_err(sdev,"Slot %d init failed.\n",i);
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

	// FREE address mappings
	for(i=0;i<SLOT_NUM;i++)
	{
		if(slot_info[i].slot_kaddr)
		{
			dma_destroy(&(slot_info[i].dma));
			iounmap(slot_info[i].slot_kaddr);
			release_mem_region(slot_info[i].base_address, slot_info[i].address_length);
		}
		if(slot_info[i].compatible_accels != NULL)
			kfree(slot_info[i].compatible_accels);
	}

	kfree(slot_info);
}

/* ======================================================================== 
							DEBUG DUMP FUNCTIONS

   ========================================================================*/
static void print_channel_data(struct dma_channel_t *chan)
{
	printk("\t\t\tInterrupt line: %d, base address: %p.\n",chan->irq, chan->base);
}
static void print_dma_data(struct dma_device_t *dma)
{
	printk("\t\tBase: %p\n",dma->base);
	printk("\t\tTx_channel:\n");
	print_channel_data(&(dma->tx_channel));
	printk("\t\tRx_channel:\n");
	print_channel_data(&(dma->rx_channel));

} 
static void print_slot_data(void)
{
	int i,j;
	for(i=0;i<SLOT_NUM;i++)
	{
		printk("Slot %d\n",i);
		printk("\tBase address: 0x%08x,\n\tslot address length: 0x%x,\n\tassociated accel num: %d.\n",slot_info[i].base_address, slot_info[i].address_length, slot_info[i].compatible_accel_num);
		printk("\tCurrent accel name: %s",slot_info[i].actual_dev?slot_info[i].actual_dev->name:" - ");
		for(j=0;j<slot_info[i].compatible_accel_num;j++)
			printk("\tIt is compatible with accel --%d--.\n",slot_info[i].compatible_accels[j]->acc_id);
		printk("\tSlot dma:\n");
		print_dma_data(&(slot_info[i].dma));
	}
	printk("There are %d slots and %d accelerators in the system.\n",SLOT_NUM,device_num);
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
	const uint8_t *text_prop;
	const uint32_t *word_prop;
	int prop_len;

	
	// init the global variables
	device_num = 0;
	device_data = NULL;
	devices = NULL;

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
	device_data = (struct dev_priv*)kmalloc(accel_num*sizeof(struct dev_priv),GFP_KERNEL);
	if(!device_data)
	{
		ret = -ENOMEM;
		goto err1;
	}
	// allocate device_data
	devices = (struct device*)kzalloc(accel_num*sizeof(struct device),GFP_KERNEL);
	if(!devices)
	{
		ret = -ENOMEM;
		goto err2;
	}

	// build compatible accel database for the slots
	for(i=0;i<SLOT_NUM;i++)
	{
		slot_info[i].compatible_accels = (struct dev_priv**)kmalloc(accel_num* sizeof(struct dev_priv*),GFP_KERNEL);
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
		// get accel name propertyl
		text_prop = of_get_property(acc,"fpga_virtual_config_name",&prop_len);
		if(text_prop==NULL || prop_len==0)
		{
			pr_err("Cannot get accelerator name.\n");
			dev_set_name(&devices[i],"%s",acc->name);
			sprintf(device_data[i].name,"acc_%d",i);
		}
		else
		{
			//pr_info("Name found for accel %d, name: %s Length: %d, address: %p, dev name ptr: %p",i,text_prop,prop_len,text_prop,device_data[i].name);
			strncpy(device_data[i].name,text_prop,32>prop_len? prop_len:32);
			dev_set_name(&devices[i],"%s",device_data[i].name);
			//pr_info("RDY");
		}

		// get compatible slot list
		word_prop = of_get_property(acc,"fpga_virtual_compatible_slots",&prop_len);
		if(word_prop==NULL || prop_len==0)
		{
			pr_err("Cannot get compatible slot no.\n");
		}
		else
		{
			prop_len/=4;
			// iterate over the compatible slots
			pr_info("Compatible slot count: %d.\n",prop_len);
			for(j=0;j<prop_len;j++)
			{
				uint32_t slot_index =be32_to_cpu(word_prop[j]);
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

//	err3:
//	kfree(devices);
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
		mgr=NULL;
		ret = -ENODEV;
	}

	// program fpga with the static config
	pr_info("Loading static FPGA configuration.\n");
	ret = program_fpga(FPGA_STATIC_CONFIG_NAME);
	//ret=0;
err:
	return ret;
}

static void disconnect_from_fpga_mgr(void)
{
	if(mgr)
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

ssize_t procfile_read(struct file *file, char __user *buffer, size_t bufsize, loff_t * offset)
{
	print_slot_data();	
	return 0;
}

static struct file_operations proc_fops =
{
		.owner = THIS_MODULE,
		.read = procfile_read
};



/* 	************************************************************************
	************************************************************************
	****			    PLATFORM DRIVER PROBE / REMOVE				    ****
	************************************************************************
	************************************************************************
*/
static int fpga_sched_probe(struct platform_device *pdev)
{
	pr_info("Starting fpga scheduler module.\n");
	pr_info("Building device modell.\n");

	platform_dev = pdev;
	// set dma bit mask
	if(build_device_modell(pdev))
	{
		pr_err("Cannot build device modell.\n");
		goto err;
	}

	// build database
	pr_info("Gathering slot data.\n");
	if(gather_slot_data())
	{
		pr_err("Failed to init slot data.\n");
		goto err1;
	}

	pr_info("Gathering accel data.\n");
	if(gather_accel_data())
	{
		pr_err("Failed to load accel data.\n");
		goto err2;
	}

	pr_info("Connecting to the fpga manager.\n");
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

static int fpga_sched_remove(struct platform_device *pdev)
{
	//send stop signal to the kernel thread
	quit = 1;
	complete(&event_in);
	wait_for_completion(&thread_stop);

	procfs_destroy();
	misc_deregister(&misc);
	disconnect_from_fpga_mgr();
	free_accel_data();
	free_slot_data();
	clean_device_modell();
	pr_info("Fpga scheduler module exited.\n");
	return 0;
}				



static struct of_device_id fpga_sched_of_match[] = {
	{ .compatible = "fpga_virtual_scheduler", },
	{ }
};


static struct platform_driver fpga_sched_platform_driver = {
	.probe = fpga_sched_probe,
	.remove = fpga_sched_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DRIVER_NAME,
		.of_match_table = fpga_sched_of_match,
	},
};

module_platform_driver(fpga_sched_platform_driver);

/*
module_init(fpga_sched_init);
module_exit(fpga_sched_exit);*/

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Tusori Tibor");
MODULE_DESCRIPTION("FPGA scheduler");
