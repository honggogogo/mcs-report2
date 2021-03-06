/*
// File: task_1.cpp
//              
// Framework to implement Task 1 of the Advances in Computer Architecture lab 
// session. This uses the ACA 2009 library to interface with tracefiles which
// will drive the read/write requests
//
// Author(s): Michiel W. van Tol, Mike Lankamp, Jony Zhang, 
//            Konstantinos Bousias
// Copyright (C) 2005-2009 by Computer Systems Architecture group, 
//                            University of Amsterdam
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//
*/

#include "aca2009.h"
#include <systemc.h>
#include <iostream>

using namespace std;

static const int MEM_SIZE = 2 << 20;

static const int SET_COUNT = 128;     //There are 8 sets in the cache
static const int SET_SIZE = 8;    //One set contains 125 lines
static const int LINE_SIZE = 32;    //One line contais 32 Bytes

TraceFile* tracefile_ptr_old;
bool eof_tracefile(TraceFile* tracefile_ptr){
    return tracefile_ptr_old == tracefile_ptr;
}

SC_MODULE(Memory) 
{

public:
    enum Function 
    {
        FUNC_READ,
        FUNC_WRITE
    };

    enum RetCode 
    {
        RET_READ_DONE,
        RET_WRITE_DONE,
    };

    sc_in<bool>     Port_CLK;
    sc_in<Function> Port_Func;
    sc_in<int>      Port_Addr;
    sc_out<RetCode> Port_Done;
    sc_inout_rv<32> Port_Data;

    SC_CTOR(Memory) 
    {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();

        m_data = new int[MEM_SIZE];
    }

    ~Memory() 
    {
        delete[] m_data;
    }

private:
    int* m_data;

    void execute() 
    {
        while (true)
        {
            wait(Port_Func.value_changed_event());	// this is fine since we use sc_buffer
            
            Function f = Port_Func.read();
            int addr   = Port_Addr.read();
            //int offset = (addr & 31);
            //int tag = (addr >> 5);
            //int index = (tag & 127);
            //tag = (tag >> 7);


            int data   = 0;
            if (f == FUNC_WRITE) 
            {
                cout << sc_time_stamp() << ": MEM received write" << endl;
                data = Port_Data.read().to_int();
            }
            else
            {
                cout << sc_time_stamp() << ": MEM received read" << endl;
            }

            // This simulates memory read/write delay
            wait(99);

            if (f == FUNC_READ) 
            {
                Port_Data.write( (addr < MEM_SIZE) ? m_data[addr] : 0 );
                Port_Done.write( RET_READ_DONE );
                wait();
                Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
            }
            else
            {
                if (addr < MEM_SIZE) 
                {
                    m_data[addr] = data;
                }
                Port_Done.write( RET_WRITE_DONE );
            }
        }
    }

};

//Added Bus Interface

class Bus_if : public virtual sc_interface
{
public:
    virtual bool read(int addr) = 0;
    virtual bool write(int addr, int data) = 0;
};

//Added Bus Module

class Bus : public Bus_if , public sc_module
{
public:
    enum Function
    {
        FUNC_READ,
        FUNC_WRITE
    };
    // ports
    sc_in<bool>             Port_CLK;
    sc_signal_rv<int>       Port_BusAddress;
    sc_out<int>             Port_BusWriter;
    sc_out<Function>        Port_BusFunction;
    sc_out<int>             Port_BusLocked;

    //std::mutex              mutex;
public:
    SC_CTOR(Bus)
    {
        
        sensitive << Port_CLK.pos();
        
    // Handle Port_CLK to simulate delay
    // Initialize some bus properties
    }
    virtual bool read(int addr)
    {
        // Bus might be in contention
        locked = true;
        Port_BusAddress.write(addr);
        Port_BusFunction.write(FUNC_READ);
        wait(99);
        locked = false;
        return true;
    }
    
    virtual bool write(int addr, int data)
    {
        // Handle contention if any
        locked = true;
        Port_BusAddress.write(addr);
        Port_BusFunction.write(FUNC_WRITE);
        wait(99);
        locked = false;
        // Data does not have to be handled in the simulation
        return true;
    }
private:
    bool                    locked;
    


};

//Added Cache Module

SC_MODULE(Cache)
{

public:
    enum Function
    {
        FUNC_READ,
        FUNC_WRITE
    };

    enum RetCode
    {
        RET_READ_DONE,
        RET_WRITE_DONE,
    };

    enum HitType //enum not displayed in waveform file, replaced by integers
    {
        READ_MISS,  //0
        READ_HIT,   //1
        WRITE_MISS, //2
        WRITE_HIT   //3
    };

    sc_in<bool>     Port_CLK;
    sc_in<Function> Port_Func;
    sc_in<int>      Port_Addr;
    sc_out<RetCode> Port_Done;
    sc_out<int>     Port_Tag;
    sc_out<int>     Port_Set;
    sc_out<int>     Port_Line;
    sc_out<int>     Port_Hit;
    sc_inout_rv<32> Port_Data;
    
    sc_port<Bus_if> Port_Bus;
    sc_in<int>      Port_BusLocked;


    SC_CTOR(Cache)
    {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();
        
    }
    
    Cache() : Cache("cache"){
        
    }

    ~Cache()
    {
        //delete[] c_data;                            //warning
    }

private:
    int c_data[SET_COUNT][SET_SIZE*(LINE_SIZE)];    //cache data, 125 sets with 8 lines of 32 bytes
    short lru[SET_COUNT][SET_SIZE];                 //lru number for each line, could be done differently
    int tags[SET_COUNT][SET_SIZE];                  //tags for each line in the cache
    bool valid[SET_COUNT][SET_SIZE];
    
    void execute()
    {
        
        while (true)
        {
            wait(Port_Func.value_changed_event());	// this is fine since we use sc_buffer

            Function f = Port_Func.read();
            int addr   = Port_Addr.read();
            int offset = (addr & 31);                   //first 5 bit are the offset
            int tag = (addr >> 5);
            int index = (tag & 127);                    //next 7 bit are the index
            Port_Set.write(index);
            
            tag = (tag >> 7);                           //last 20 bit are the tag
            Port_Tag.write(tag);
            
            int data = 0;
            if (f == FUNC_WRITE)
            {
                cout << sc_time_stamp() << ": " <<name()<<" received write" << endl;
                data = Port_Data.read().to_int();   //cache reads from the cpu
            }
            else
            {
                cout << sc_time_stamp() << ": " <<name()<<" received read" << endl;
            }


            if (f == FUNC_READ)
            {
                int set_index = hit(index,tag); //set index should be called line index
                if(set_index == -1){
                    Port_Hit.write(0);
                } else {
                    Port_Hit.write(1);
                }
                if (set_index == -1){           //if miss simulate a read from the memory
                    
                    Port_Bus->read(addr);
                                       // This simulates memory read/write delay
                    set_index = get_lru(index); //get the lru index
                    tags[index][set_index]=tag; //simulate read, copy data from memory to the cache
                    c_data[index][set_index*LINE_SIZE + offset] = set_index*offset; //random value instead of value from memory
                }
                Port_Data.write(c_data[index][set_index*LINE_SIZE + offset]);
                mru(index,set_index);
                Port_Done.write( RET_READ_DONE );
                Port_Line.write(set_index);
                wait();
                Port_Data.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
            }
            else //if write
            {
                if (addr < MEM_SIZE) //optional
                {
                    int set_index = hit(index,tag);
                    if(set_index == -1){
                        Port_Hit.write(2);
                        set_index = get_lru(index);
                        tags[index][set_index]=tag;
                        //Port_Bus->write(addr,0);
                        wait(99);   //simulate memory delay
                    } else {
                        Port_Hit.write(3);
                        mru(index,set_index);
                        wait();
                    }
                    //index in the set of the cache
                    Port_Line.write(set_index);
                    c_data[index][set_index*LINE_SIZE + offset] = data; //only overwrites one byte, not fully correct



                }
                Port_Done.write( RET_WRITE_DONE );
            }
        }
    }
    
    int read_bus(int addr){
        
        
        //Port_BusFunction.write(Bus::FUNC_READ);
        //Port_BusAddress.write(addr);
        
        return 0;
        
    }
    
    bool write_bus(int addr, int data){
        
        
        //Port_BusFunction.write(Bus::FUNC_WRITE);
        //Port_BusAddress.write(addr);
        
        return true;
    }

    int hit(int index,int tag){ //returns the index of the line with the same tag or if miss -1
        for(int i = 0;i<SET_SIZE;i++)
            if(tags[index][i] == tag){
                mru(index,i);
                return i;
            }
        return -1;
    }
    
    void mru(int index, int mru_index){ //set most recently used after succesfull read
        for(int i=0;i<SET_SIZE;i++){
            if(lru[index][i]<lru[index][mru_index])
                lru[index][mru_index]++;
        }
        lru[index][mru_index]=0;
    }
    
    int get_lru(int index){ //returns index of the lru line
        int max = 0;
        int max_index=0;
        for(int i = 0; i<SET_SIZE;i++){
            if(lru[index][i] > max){
                max =  lru[index][i];
                max_index = i;
            }

        }
        return max_index;
        
    }
};

SC_MODULE(CPU) 
{

public:
    sc_in<bool>                Port_CLK;
    sc_in<Cache::RetCode>   Port_MemDone;
    sc_out<Cache::Function> Port_MemFunc;
    sc_out<int>                Port_MemAddr;
    sc_inout_rv<32>            Port_MemData;
    sc_in<int>                 Port_Hit;
    
    int cpu_id;
    unsigned int counter = 0;
    
    SC_CTOR(CPU) 
    {
        SC_THREAD(execute);
        sensitive << Port_CLK.pos();
        dont_initialize();
    }
    
        
    CPU():CPU("cpu"){
        
    }

private:
    
    void execute() 
    {
        TraceFile::Entry    tr_data;
        Cache::Function  f;

        /*for(int i = 0;i<20;i++){
            Port_MemAddr.write(0x7FFFFFFF);
            Port_MemFunc.write(Cache::FUNC_READ);
            cout << sc_time_stamp() << ": CPU sends read" << endl;
            wait(Port_MemDone.value_changed_event());
            cout << sc_time_stamp() << ": CPU reads: " << Port_MemData.read() << endl;

        }*/
        // Loop until end of tracefile
        while((!tracefile_ptr->eof()) && counter <=1000) //eof() function did not work
        {
            counter ++;
            //eof_tracefile(tracefile_ptr);
           
            // Get the next action for the processor in the trace
            if(!tracefile_ptr->next(cpu_id, tr_data))
            {
                cerr << "Error reading trace for CPU" << endl;
                break;
            }

            // To demonstrate the statistic functions, we generate a 50%
            // probability of a 'hit' or 'miss', and call the statistic
            // functions below
            //int j = rand()%2;

            switch(tr_data.type)
            {
                case TraceFile::ENTRY_TYPE_READ:
                    f = Cache::FUNC_READ;
                    /*if(j)
                        stats_readhit(0);
                    else
                        stats_readmiss(0);
                    */
                    break;

                case TraceFile::ENTRY_TYPE_WRITE:
                    f = Cache::FUNC_WRITE;
                    /*if(j)
                        stats_writehit(0);
                    else
                        stats_writemiss(0);
                    */
                    break;
                    
                case TraceFile::ENTRY_TYPE_NOP:
                    break;

                default:
                    cerr << "Error, got invalid data from Trace" << endl;
                    //exit(0);
                    break;
            }

            if(tr_data.type != TraceFile::ENTRY_TYPE_NOP)
            {
                Port_MemAddr.write(tr_data.addr);
                Port_MemFunc.write(f);

                if (f == Cache::FUNC_WRITE)
                {
                    cout << sc_time_stamp() << ": " <<name()<<" sends write" << endl;

                    uint32_t data = rand();
                    Port_MemData.write(data);
                    wait();
                    Port_MemData.write("ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ");
                }
                else
                {
                    cout << sc_time_stamp() << ": " <<name()<<" sends read" << endl;
                }

                wait(Port_MemDone.value_changed_event());
                int hit_id = Port_Hit.read();
                stats(hit_id);

                if (f == Cache::FUNC_READ)
                {
                    cout << sc_time_stamp() << ": " <<name()<<" reads: " << Port_MemData.read() << endl;
                }
            }
            else
            {
                cout << sc_time_stamp() << ": "<< name() <<" executes NOP" <<endl;
                
            }
            // Advance one cycle in simulated time            
            wait();
        }
        
        // Finished the Tracefile, now stop the simulation
        sc_stop();
    }
    
    void stats(int hit_id){
        switch(hit_id){
            case 0:
                stats_readmiss(cpu_id);
                break;
            case 1:
                stats_readhit(cpu_id);
                break;
            case 2:
                stats_writemiss(cpu_id);
                break;
            case 3:
                stats_writehit(cpu_id);
                break;
            default:
                break;
            
        }
        
    }
};


int sc_main(int argc, char* argv[])
{
    try
    {
        // Get the tracefile argument and create Tracefile object
        // This function sets tracefile_ptr and num_cpus
        init_tracefile(&argc, &argv);
        // Initialize statistics counters
        stats_init();

        // Instantiate Modules
        Cache   cache[num_cpus];        
        CPU     cpu[num_cpus];
        
        Bus    bus("bus");
        
        //Bus
        sc_signal<int>                  sigBusAddress;
        sc_signal<Bus::Function>        sigBusFunction;
        sc_signal<int>                  sigBusLocked;
        
        // Signals
        sc_buffer<Cache::Function> sigMemFunc[num_cpus];
        sc_buffer<Cache::RetCode>  sigMemDone[num_cpus];
        sc_signal<int>             sigMemAddr[num_cpus];
        sc_signal_rv<32>           sigMemData[num_cpus];


        sc_signal<int>             sigCacheSet[num_cpus] ;
        sc_signal<int>             sigCacheLine[num_cpus] ;
        sc_signal<int>             sigCacheHit[num_cpus] ;
        sc_signal<int>             sigCacheTag[num_cpus] ;

        // The clock that will drive the CPU and Memory
        sc_clock clk;

        // Connecting module ports with signals
        for(unsigned int i = 0; i< num_cpus;i++){
            cache[i].Port_Func(sigMemFunc[i]);
            cache[i].Port_Addr(sigMemAddr[i]);
            cache[i].Port_Data(sigMemData[i]);
            cache[i].Port_Done(sigMemDone[i]);
            cache[i].Port_Hit(sigCacheHit[i]);
            cache[i].Port_Line(sigCacheLine[i]);
            cache[i].Port_Set(sigCacheSet[i]);
            cache[i].Port_Tag(sigCacheTag[i]);
            cache[i].Port_BusLocked(sigBusLocked);
            //bind bus
            cache[i].Port_Bus(bus);
            
            cpu[i].Port_MemFunc(sigMemFunc[i]);
            cpu[i].Port_MemAddr(sigMemAddr[i]);
            cpu[i].Port_MemData(sigMemData[i]);
            cpu[i].Port_MemDone(sigMemDone[i]);
            cpu[i].Port_Hit(sigCacheHit[i]);
            cpu[i].cpu_id = i;
            cache[i].Port_CLK(clk);
            cpu[i].Port_CLK(clk);

            
                
        }

        bus.Port_BusAddress(sigBusAddress);
        bus.Port_BusFunction(sigBusFunction);
        
        bus.Port_BusLocked(sigBusLocked);
        
        bus.Port_CLK(clk);

        cout << "Running (press CTRL+C to interrupt)... " << endl;

        sc_trace_file* wf;
        wf = sc_create_vcd_trace_file("task_1");
        int i = 0;
        sc_trace(wf,sigMemFunc[i],"MemFunc");
        sc_trace(wf,sigMemAddr[i],"MemAddr");
        sc_trace(wf,sigMemData[i],"MemData");
        sc_trace(wf,sigMemDone[i],"MemDone");
        sc_trace(wf,sigCacheHit[i],"CacheHit");
        sc_trace(wf,sigCacheLine[i],"CacheLine");
        sc_trace(wf,sigCacheSet[i],"CacheSet");
        sc_trace(wf,sigCacheTag[i],"CacheTag");
        sc_trace(wf,sigBusAddress,"BusAddress");
        sc_trace(wf,sigBusFunction,"BusFunction");
        sc_trace(wf,sigBusLocked,"BusLocked");


        // Start Simulation
        sc_start();
        sc_close_vcd_trace_file(wf);
        // Print statistics after simulation finished
        stats_print();
    }

    catch (exception& e)
    {
        cerr << e.what() << endl;
    }
    
    return 0;
}
