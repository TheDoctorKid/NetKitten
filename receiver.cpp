#include "netkitten.cpp"



class Receiver
{
    private:
    TimedQueue & pending_ack;          //stores the sequence_num's which havn't been acknowledged yet
    TimedQueue & ack_queue;            //stores the acknowledgments which still need to be sent
    TimedQueue & neg_ack_queue;        //stores negative acknowledgments which need to be sent
    unsigned short currentState;
    std::atomic<bool> & established;
    std::atomic<bool> & listening;
    std::vector<uint8_t> read_buffer;                //reads tetra bits in order which they arrived
    std::vector<uint8_t> transmission_content;       //perceived transmission content
    std::mutex & hardware_lock;
    int mode = 0;
    B15F* b15f;
    boost::asio::serial_port* serial;


    public:
    Receiver(B15F* board, boost::asio::serial_port* ser, TimedQueue & pen_ack, TimedQueue & ack_q, TimedQueue & neg_ack_q, std::atomic<bool> & es, std::atomic<bool> & li, std::mutex & hl, int mo)
        : b15f(board), serial(ser), pending_ack(pen_ack), ack_queue(ack_q), neg_ack_queue(neg_ack_q), established(es), listening(li), hardware_lock(hl), mode(mo)
    {
       
    }



    void beginListening()
    {
        currentState = 1;                   //start in sync state

        while(currentState != 0)                        //receiving main loop
        {
            switch (currentState)           //1=sync, 2=switch await, 3=reading
            {
            case 1:
                syncListen();
                break;

            case 2:
                awaitSwitch();
                break;

            case 3:
                receiveTransmission();
                break;
            
            default:
            std::cout << "A fatal error occured while receiving the message!" << std::endl;
                break;
            }

        }

        for(uint8_t byte : transmission_content)        //output all bytes from the transmission
        {
            std::cout << byte;
        }
        currentState = 1;               //go back to listening in another transmission is sent
    }



    void syncListen()
    {
        while(!(established.load() && listening.load()))
        {
            uint8_t curr = 0;
            uint8_t prev = 0;
            while(true)             //try catching the intermission intervall
            {
                curr = fastReadTetraPack();
                if(prev == 0x0F && curr == 0x00)
                {
                    break;
                }
                prev = curr;
            }

            for(uint32_t i = 0; i <= BYTE_BETWEEN_SYNC; i++)         
            {
                read_buffer.push_back(readTetraPack());
            }

            switch(determineCase())
            {
            case 1:             //buffer was full with SYNC
                listening.store(true);
                read_buffer.clear();
                break;

            case 2:             //buffer was full with ACK
                listening.store(true);
                established.store(true);
                read_buffer.clear();
                break;

            case 3:             //buffer didnt match a mask
                read_buffer.clear();
                continue;
            }
        }

        currentState = 3;       //everything synced up and now reading data
    }



    int determineCase() 
    {
        if (read_buffer.empty() || read_buffer[0] != 0x00) 
        {
            return 3;           //general failure
        }
        bool sync = true;
        bool ack = true;

        for (size_t i = 1; i < read_buffer.size(); ++i) {
            if (i % 2 == 0) 
            {
                if (read_buffer[i] != 0b0110) 
                {
                    return 3;
                }
            } 
            else 
            {
                if (read_buffer[i] != 0b0001) 
                {
                    sync = false;
                }
                if (read_buffer[i] != 0x00) 
                {
                    ack = false;
                }
            }
        }

        if (sync) 
        {
            return 1;
        } else if (ack) 
        {
            return 2;
        } 
        else 
        {
            return 3;
        }
    }




    uint8_t readTetraPack()
    {
        using namespace std::chrono;
        auto now = steady_clock::now();
        auto nextTick = now + milliseconds(SEND_DELAY);
        uint8_t incoming = 0;

        while(true) 
        {
            if(hardware_lock.try_lock())
            {
                switch (mode)
                {
                case 1:
                    incoming = b15f->getMem8(&PINA);        //read memory from PINA
                    break;
                
                case 2:
                    boost::asio::write(*serial, boost::asio::buffer("R", 1));
                    boost::asio::read(*serial, boost::asio::buffer(&incoming, 1));
                    break;
                }

                std::cout << int(incoming) << std::endl;
                hardware_lock.unlock();
            }
        }
        std::this_thread::sleep_until(nextTick);

        return (incoming);
    }



    void awaitSwitch() 
    {
    uint8_t prevByte = 0x00;
    uint8_t currentByte;

    while (true) 
    {
        currentByte = fastReadTetraPack();
        
        if (prevByte == 0x0F && currentByte == 0x00) 
        {
            currentState = 3;       //switch to reading state
            listening.store(true);  //receiver is now listening
            return;                 //high all was matched with all low
        }

        if (prevByte == 0x00 && currentByte != 0x00) 
        {
            read_buffer.clear();
            currentState = 1;       //switch back to sync state
            listening.store(false);
            return;                 //all high wasnt matched with all low
        }

        prevByte = currentByte;
        }
    }



    uint8_t fastReadTetraPack()
    {
        using namespace std::chrono;
        auto now = steady_clock::now();
        auto nextTick = now + milliseconds(1);
        uint8_t incoming = 0;

        while(true) 
        {
            if(hardware_lock.try_lock())
            {
                switch (mode)
                {
                case 1:
                    incoming = b15f->getMem8(&PINA);        //read memory from PINA
                    break;
                
                case 2:
                    serial->open("/dev/ttyUSB1");
                    boost::asio::write(*serial, boost::asio::buffer("R", 1));
                    boost::asio::read(*serial, boost::asio::buffer(&incoming, 1));
                    std::cout << "Reading " << int(incoming) << std::endl;
                    break;
                }

                hardware_lock.unlock();
            }
        }
        std::this_thread::sleep_until(nextTick);


        return (incoming);
    }



    uint8_t combine4BitValues(uint8_t first, uint8_t second) 
    {
        first &= 0x0F;  
        second &= 0x0F; 
        return (first << 4) | second;
    }



    void receiveTransmission()
    {
        readTetraPack();        //first read gets scrapped because its only for syncing purpouses
        for(uint32_t i = 0; i < BYTE_BETWEEN_SYNC; i++)      //reads BYTE_BETWEEN_SYNC amounts of bytes
        {
            uint8_t first = readTetraPack();
            uint8_t second = readTetraPack();
            read_buffer.push_back(combine4BitValues(first, second));
        }

        if(read_buffer.size() == BYTE_BETWEEN_SYNC && read_buffer.front() != 0x01)
        {
            read_buffer.clear();    //indirect check if there is still sync artifacts in buffer
        }

        if(read_buffer.size() == BYTE_PER_PACKAGE + HEADER_SIZE)
        {
            if(!checkPattern())            
            {
                currentState = 1;   //if pattern wasnt recognised go back to sync state
            }
            read_buffer.clear();
            return;
        }

        currentState = 2;
        return;
    }



    bool checkPattern() 
     {
        if (read_buffer[0] != 0x01) 
        {
            return false;
        }

        uint32_t received_package_sequence = (read_buffer[1] << 24) | (read_buffer[2] << 16) | (read_buffer[3] << 8) | read_buffer[4];

        if (read_buffer[5] != 0x06 && read_buffer[5] != 0x15) 
        {
            return false;
        }

        uint32_t acknowledged_sequence = (read_buffer[6] << 24) | (read_buffer[7] << 16) | (read_buffer[8] << 8) | read_buffer[9];

        if (read_buffer[10] != 0x16 || read_buffer[11] != 0x16) 
        {
            return false;
        }

        uint32_t checksum = (read_buffer[12] << 8) | (read_buffer[13] << 4) | read_buffer[14];

        if (read_buffer[15] != 0x02) 
        {
            return false;
        }

        if(!checkChecksum(checksum))
        {
            neg_ack_queue.push(received_package_sequence);      //the pattern matched but checksum was wrong
            return false;
        }

        //package is valid!
        ack_queue.push(received_package_sequence);              //tell transmitter to acknowledge this package
        if(read_buffer[5] == 0x06)
        {
            pending_ack.remove(acknowledged_sequence);              //tell transmitter to not wait for package he sent anymore with sequence number ACKNOWLEDGED SEQUENCE
        }

        if (transmission_content.size() < received_package_sequence * BYTE_PER_PACKAGE)
        {
            transmission_content.resize(received_package_sequence * BYTE_PER_PACKAGE);
        }
        transmission_content.insert(transmission_content.begin() + received_package_sequence*BYTE_PER_PACKAGE, read_buffer.begin()+HEADER_SIZE, read_buffer.end());


        return true;
    }



    bool checkChecksum(uint32_t received_checksum)           //compares checksum received with calculated from package 
    {
        uint32_t checksum = 0;
        for(uint32_t i = HEADER_SIZE; i<HEADER_SIZE+BYTE_PER_PACKAGE; i++) 
        {
            checksum += read_buffer[i];
        }

        return ((checksum + received_checksum) & 0xFFFFFF) == 0;        //check if addition gives 0
    }

};