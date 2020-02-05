//
//  VoodooI2CSynapticsDevice.hpp
//  VoodooI2CSynaptics
//
//  Created by Alexandre on 25/12/2017.
//  Based on code written by CoolStar and Kishor Prins
//  Copyright © 2017 Alexandre Daoud. All rights reserved.
//

#ifndef VoodooI2CSynapticsDevice_hpp
#define VoodooI2CSynapticsDevice_hpp

#include <IOKit/IOLib.h>
#include <IOKit/IOKitKeys.h>
#include <IOKit/IOService.h>

#include <IOKit/IOLib.h>
#include <IOKit/IOWorkLoop.h>
#include <IOKit/IOInterruptEventSource.h>
#include <IOKit/acpi/IOACPIPlatformDevice.h>
#include <IOKit/IOLocks.h>
#include <IOKit/IOCommandGate.h>
#include <IOKit/IOTimerEventSource.h>

#include "../../../VoodooI2C/VoodooI2C/VoodooI2CDevice/VoodooI2CDeviceNub.hpp"
#include "../../../Multitouch Support/VoodooI2CMultitouchInterface.hpp"
#include "../../../Dependencies/helpers.hpp"

#include "VoodooI2CSynapticsConstants.h"

enum rmi_mode_type {
    RMI_MODE_OFF = 0,
    RMI_MODE_ATTN_REPORTS = 1,
    RMI_MODE_NO_PACKED_ATTN_REPORTS = 2,
};

/* describes a single packet register */
struct rmi_register_desc_item {
    uint16_t reg = 0;
    unsigned long reg_size;
    uint8_t num_subpackets;
    unsigned long subpacket_map[BITS_TO_LONGS(RMI_REG_DESC_SUBPACKET_BITS)];
};

struct rmi4_attn_data {
    unsigned long irq_status;
    size_t size;
    void *data;
};
                                                        
/*
 * describes the packet registers for a particular type
 * (ie query, control, data)
 */
struct rmi_register_descriptor {
    unsigned long struct_size;
    unsigned long presense_map[BITS_TO_LONGS(RMI_REG_DESC_PRESENSE_BITS)];
    uint8_t num_registers;
    struct rmi_register_desc_item *registers;
};

struct rmi_function {
    unsigned page;            /* page of the function */
    uint16_t query_base_addr;        /* base address for queries */
    uint16_t command_base_addr;        /* base address for commands */
    uint16_t control_base_addr;        /* base address for controls */
    uint16_t data_base_addr;        /* base address for datas */
    unsigned int interrupt_base;    /* cross-function interrupt number
                                     * (uniq in the device)*/
    unsigned int interrupt_count;    /* number of interrupts */
    size_t report_size;    /* size of a report */
    unsigned long irq_mask;        /* mask of the interrupts
                                    * (to be applied against ATTN IRQ) */
    struct rmi_register_descriptor query_reg_desc;
    struct rmi_register_descriptor control_reg_desc;
    struct rmi_register_descriptor data_reg_desc;
    uint8_t *data_buf;
    /* F12 Data1 describes sensed objects */
    const struct rmi_register_desc_item *data1;
    uint16_t data1_offset;
    
    /* F12 Data5 describes finger ACM */
    const struct rmi_register_desc_item *data5;
    uint16_t data5_offset;
    
    /* F12 Data5 describes Pen */
    const struct rmi_register_desc_item *data6;
    uint16_t data6_offset;
    
    
    /* F12 Data9 reports relative data */
    const struct rmi_register_desc_item *data9;
    uint16_t data9_offset;
    
    const struct rmi_register_desc_item *data15;
    uint16_t data15_offset;

};


typedef struct __attribute__((__packed__)) pdt_entry {
    uint8_t query_base_addr : 8;
    uint8_t command_base_addr : 8;
    uint8_t control_base_addr : 8;
    uint8_t data_base_addr : 8;
    uint8_t interrupt_source_count : 3;
    uint8_t bits3and4 : 2;
    uint8_t function_version : 2;
    uint8_t bit7 : 1;
    uint8_t function_number : 8;
};

class VoodooI2CSynapticsDevice : public IOService
{
    typedef IOService super;
    OSDeclareDefaultStructors(VoodooI2CSynapticsDevice);
    
private:
    IOACPIPlatformDevice* acpi_device;
    VoodooI2CDeviceNub* api;
    IOCommandGate* command_gate;
    UInt16 hid_descriptor_register;
    IOInterruptEventSource* interrupt_source;
    
    OSArray* transducers;
    
    uint16_t max_x;
    uint16_t max_y;
    
    int page;
    
    unsigned long flags;
    
    struct rmi_function f01;
    struct rmi_function f11;
    struct rmi_function f12;
    struct rmi_function f30;
    
    unsigned int max_fingers;
    unsigned int x_size_mm;
    unsigned int y_size_mm;
    bool read_f11_ctrl_regs;
    uint8_t f11_ctrl_regs[RMI_F11_CTRL_REG_COUNT];
    
    unsigned int gpio_led_count;
    unsigned int button_count;
    unsigned long button_mask;
    unsigned long button_state_mask;
    
    unsigned long device_flags;
    unsigned long firmware_id;
    
    uint8_t f01_ctrl0;
    uint8_t interrupt_enable_mask;
    bool restore_interrupt_mask;
    
    VoodooI2CMultitouchInterface* mt_interface;
    
protected:
    bool awake;
    const char* name;
    IOWorkLoop* work_loop;
    bool reading;
    IOLock* client_lock;
    IOLock* stop_lock;
    OSArray* clients;
    
public:
    void stop(IOService* device) override;
    
    bool start(IOService* api);
    
    bool init(OSDictionary* properties);
    
    void free();
        
    VoodooI2CSynapticsDevice* probe(IOService* provider, SInt32* score);
    
    bool open(IOService *forClient, IOOptionBits options = 0, void *arg = 0) override;
    
    void close(IOService *forClient, IOOptionBits options) override;
    
    void interruptOccured(OSObject* owner, IOInterruptEventSource* src, int intCount);
    
    void get_input();
    
    IOReturn setPowerState(unsigned long powerState, IOService *whatDevice) override;
    
    int rmi_read_block(uint16_t addr, uint8_t *buf, const int len);
    int rmi_write_report(uint8_t *report, size_t report_size);
    int rmi_read(uint16_t addr, uint8_t *buf);
    int rmi_write_block(uint16_t addr, uint8_t *buf, const int len);
    int rmi_write(uint16_t addr, uint8_t *buf);
    void rmi_register_function(struct pdt_entry *pdt_entry, int page, unsigned interrupt_count);
    
    int rmi_scan_pdt();
    int rmi_populate_f01();
    int rmi_populate_f11();
    int rmi_populate_f12();
    int rmi_populate_f30();
    int rmi_populate();
    
    int rmi_set_mode(uint8_t mode);
    
    int rmi_set_page(uint8_t _page);
    
    void rmi_f11_process_touch(OSArray* transducers, int transducer_id, AbsoluteTime timestamp, uint8_t finger_state, uint8_t *touch_data);
    int rmi_f11_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t *rmiInput);
    
    void rmi_f12_process_touch(OSArray* transducers, int transducer_id, AbsoluteTime timestamp, uint8_t finger_state, uint8_t *touch_data);
    int rmi_f12_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t *rmiInput);
    
    int rmi_f30_input(OSArray* transducers, AbsoluteTime timestamp, uint8_t irq, uint8_t *rmiInput, int size);
    void TrackpadRawInput(uint8_t report[40]);
    
    bool publish_multitouch_interface();
    void unpublish_multitouch_interface();
    
    size_t rmi_register_desc_calc_size(struct rmi_register_descriptor *rdesc);
    const struct rmi_register_desc_item *rmi_get_register_desc_item(struct rmi_register_descriptor *rdesc, uint16_t reg);
    int rmi_register_desc_calc_reg_offset(struct rmi_register_descriptor *rdesc, uint16_t reg);
    int rmi_read_register_desc(uint16_t addr, struct rmi_register_descriptor *rdesc);
    bool rmi_register_desc_has_subpacket(const struct rmi_register_desc_item *item,
                                         uint8_t subpacket);
    
    void releaseResources();
};


#endif /* VoodooI2CSynapticsDevice_hpp */
