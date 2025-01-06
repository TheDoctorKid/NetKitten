#include "netkitten.cpp"



class Transmitter
{
    private:
    std::vector<uint8_t> control_chars = {
    0x00, // NUL (Null)
    0x01, // SOH (Start of Heading)
    0x02, // STX (Start of Text)
    0x03, // ETX (End of Text)
    0x04, // EOT (End of Transmission)
    0x05, // ENQ (Enquiry)
    0x06, // ACK (Acknowledge)
    0x07, // BEL (Bell)
    0x08, // BS  (Backspace)
    0x09, // TAB (Horizontal Tab)
    0x0A, // LF  (Linefeed / Newline)
    0x0B, // VT  (Vertical Tab)
    0x0C, // FF  (Formfeed)
    0x0D, // CR  (Carriage Return)
    0x0E, // SO  (Shift Out)
    0x0F, // SI  (Shift In)
    0x10, // DLE (Data Link Escape)
    0x11, // DC1 (Device Control 1)
    0x12, // DC2 (Device Control 2)
    0x13, // DC3 (Device Control 3)
    0x14, // DC4 (Device Control 4)
    0x15, // NAK (Negative Acknowledge)
    0x16, // SYN (Synchronous Idle)
    0x17, // ETB (End of Block)
    0x18, // CAN (Cancel)
    0x19, // EM  (End of Medium)
    0x1A, // SUB (Substitute)
    0x1B, // ESC (Escape)
    0x1C, // FS  (File Separator)
    0x1D, // GS  (Group Separator)
    0x1E, // RS  (Record Separator)
    0x1F  // US  (Unit Separator)
    };

    std::vector<uint8_t> transmission_content;
    TimedQueue & pending_ack;           //stores the sequence_num's which havn't been acknowledged yet
    TimedQueue & ack_queue;             //stores the acknowledgments which still need to be sent
    TimedQueue & neg_ack_queue;         //stores negative acknowledgments which need to be sent
    std::queue<uint32_t> sequence_num_queue;      //stores all sequence numbers in order to be sent
    std::atomic<bool> & established;              //is sent data received
    std::atomic<bool> & listening;                //is own receiver currently reading data
    std::atomic<bool> & partner_finished;         //does other client finished transmission
    unsigned short resync_count = 0;              //counts sent nibbles to controll when to resync periodically
    bool list_mode = false;                       //true if started in listening mode
    std::mutex & hardware_lock;
    int mode = 0;
    B15F* b15f;
    boost::asio::serial_port* serial;


    public:
    Transmitter(B15F* board, boost::asio::serial_port* ser, TimedQueue & pen_ack, TimedQueue & ack_q, TimedQueue & neg_ack_q, std::atomic<bool> & es, std::atomic<bool> & li, std::atomic<bool> & pf, std::mutex & hl, int mo)
        : b15f(board), serial(ser), pending_ack(pen_ack), ack_queue(ack_q), neg_ack_queue(neg_ack_q), established(es), listening(li), partner_finished(pf), hardware_lock(hl), mode(mo)
    {

    }



    void beginTransmission()        //prepare by reading cin
    {
        if(!list_mode)
        {
            transmission_content.assign((std::istreambuf_iterator<char>(std::cin)), {});
            std::cout << "Sending given Message..." << std::endl;
        }

        else
        {
            std::cout << "Just listening..." << std::endl;
        }
        
        processData();
    }

    
    
    void processData()             //pads content and generates sequence numbers
    {
        if(transmission_content.size() % BYTE_PER_PACKAGE != 0)     //if there is something to send, save in content and pad it
        {
            transmission_content.insert(transmission_content.end(), BYTE_PER_PACKAGE - (transmission_content.size() % BYTE_PER_PACKAGE), 0x00);       //adds padding if needed
        }

        for(uint32_t i = 0; i < transmission_content.size() / BYTE_PER_PACKAGE; i++)        //generate all the sequence numbers
        {
            sequence_num_queue.push(i);
        }

        transmissionController();
        return;
    }
    
    
    
    void transmissionController()       //controlls everything from resyncing and sending the packages
    {
        bool final_ack = false;         //handles the last mandatory ACK to complete handshake
        bool transmission_complete = false;     //handles the last EOT signal if nothing anymore to send
        int status = 0;                 //decides the state of the transmitter

        while(true)         //transmission main loop
        {
            switch(status)
            {
            case 0:         //SYNC State
                std::cout << "Trying to sync communication." << std::endl;
                final_ack = true;
                syncComs();
                break;
            
            case 1:         //USUAL transmission State
                sendStack(sequence_num_queue.front());
                sequence_num_queue.pop();
                break;

            case 2:         //RESEND lost or delayed packages State
                std::cout << "Sending package " << pending_ack.front() << " again!" << std::endl;
                sendStack(pending_ack.front());
                pending_ack.pop();
                break;

            case 3:         //RESPOND only to received signal State
                if(!transmission_complete)
                {
                    transmission_complete = true;
                    writeByte(0x01);
                    for(int i = 0; i < HEADER_SIZE+BYTE_PER_PACKAGE-2; i++)
                    {
                        writeByte(0x04);        //send a stack filled with EOT to signal end of transmission
                    }
                    writeByte(0x03);
                    continue;
                }
                else
                {
                    sendStack(uint32_t(~0));    //only respond from now on
                    continue;
                }
                break;
            
            default:
                std::cout << "Transmitter ran into an unknown Problem!" << std::endl;
                break;
            }


            if(!established.load() || !listening.load())      //if desynced try resync
            {status = 0; continue;}

            if(established.load() && listening.load() && final_ack)     //sending ACK a last time to finish connection
            {
                for(uint32_t i = 0; i < BYTE_BETWEEN_SYNC; i++)     
                {
                    writeByte(0x06);
                }
                final_ack = false;
                status = 1;             //next state is going to be usual send
                continue;
            }

            if(pending_ack.empty() && sequence_num_queue.empty())               //there is no more to send, just respond other client
            {
                if(partner_finished.load())
                {return;}               //nothing to send and nothing to receive anymore
                status = 3;
            }

            if(pending_ack.size() < 10 && !sequence_num_queue.empty())          //send next package as usual
            {status = 1; continue;}

            if((pending_ack.size() >= 10) || sequence_num_queue.empty())        //pending ACKs exceed 10 limit, resend lost packages
            {status = 2; continue;}
        }
    }



    void syncComs()     //depending on receiver state send SYNC or ACK
    {
        if(!listening.load() && !established.load())
        {
            std::cout << "Sending SYNC IDLE." << std::endl;
            for(uint32_t i = 0; i < BYTE_BETWEEN_SYNC; i++)                  //sending SYNC IDLE
            {
                writeByte(0x16);
            }
        }
        
        else if(listening.load() && !established.load())
        {
            std::cout << "Sending ACK." << std::endl;
            for(uint32_t i = 0; i < BYTE_BETWEEN_SYNC; i++)                  //sending ACK for own receiver is listening
            {
                writeByte(0x06);
            }
        }
    }



    void sendStack(uint32_t package_index)                                                                  //send the correstponding package for package_index
    {
        std::vector<uint8_t> stack_package = {0x01};                                                        //begin with start of heading
        std::vector<uint8_t> index_conversion = uint32ToByte(uint32_t(package_index));
        stack_package.insert(stack_package.end(), index_conversion.begin(), index_conversion.end());        //insert sequence number

        if(neg_ack_queue.empty())
        {
            stack_package.push_back(0x06);                                                                  //send acknowledgment
            std::vector<uint8_t> ack_conversion = ack_queue.empty() ? uint32ToByte(uint32_t(0)) : uint32ToByte(ack_queue.front());
            stack_package.insert(stack_package.end(), ack_conversion.begin(), ack_conversion.end());        //insert acknowledgment number
            if(!ack_queue.empty()) 
            {
                ack_queue.pop();
            };
        }            
        else
        {
            stack_package.push_back(0x15);                                                                  //send negative acknowledgment
            std::vector<uint8_t> neg_ack_conversion = uint32ToByte(neg_ack_queue.front());
            stack_package.insert(stack_package.end(), neg_ack_conversion.begin(), neg_ack_conversion.end());//insert acknowledgment number
            neg_ack_queue.pop();
        }                                                          
        
        stack_package.insert(stack_package.end(), 2, 0x16);                                                 //insert a sync idle check
        uint32_t checksum = calcChecksum(package_index);
        std::vector<uint8_t> check_conversion = uint32ToByte(checksum);
        stack_package.insert(stack_package.end(), check_conversion.end() - 2 , check_conversion.end());     //insert the corresponding 24 bit checksum
        stack_package.push_back(0x02);                                                                      //insert start of text

        if(package_index == uint32_t(~0))       //send empty package
        {
            stack_package.insert(stack_package.end(), BYTE_PER_PACKAGE, 0x00);                              //add BYTE_PER_PACKAGE byte of null values
        }

        else        //send normal message content
        {
            for(uint32_t i = package_index*BYTE_PER_PACKAGE; i<package_index*BYTE_PER_PACKAGE+BYTE_PER_PACKAGE; i++)                                        
            { 
                stack_package.push_back(transmission_content[i]);                                           //add BYTE_PER_PACKAGE byte of data
            }

            pending_ack.push(package_index);                                                                //add sent package sequence number to pending acknowledgements
        }

        stack_package.push_back(0x03);                                                                      //end on end of text

        for(uint8_t byte : stack_package)                                                                   //actually sending the message
        {
            writeByte(byte);
        }
        std::cout << std::endl;
        
        std::cout << "Package " << package_index << " was sent." << std::endl;
        std::cout << "Checksum " << checksum << std::endl;
    }



    void writeByte(uint8_t byte)
    {
        std::cout << "Sending " << int(byte) << std::endl;

        if (resync_count % BYTE_BETWEEN_SYNC == 0 && resync_count != 0)
        {
            writeTetraPack(0b00001111);
            writeTetraPack(0b00000000);
        }

        writeTetraPack(getBits(byte, 4));
        writeTetraPack(getBits(byte, 0));

        if (resync_count % BYTE_BETWEEN_SYNC == BYTE_BETWEEN_SYNC - 1)
        {
            writeTetraPack(0b00000000);
            writeTetraPack(0b00001111);
        }

        resync_count++;
    }




    void writeTetraPack(uint8_t half_byte)
    {
        //TODO implement to write 4 bit onto register
        using namespace std::chrono;
        auto now = steady_clock::now();

        auto nextTick = now + milliseconds(SEND_DELAY);

        while(true) 
        {
            if(hardware_lock.try_lock())
            {
                switch (mode)
                {
                case 1:
                    b15f->setMem8(&PORTA, half_byte);                     //write values on 4 lines
                    break;
                
                case 2:
                    //send command
                    const char command = 'W';
                    boost::asio::write(*serial, boost::asio::buffer(&command, 1));
                    //send value
                    boost::asio::write(*serial, boost::asio::buffer(&half_byte, 1));
                    break;
                }
                
                hardware_lock.unlock();
                break;
            }
        }
        
        std::this_thread::sleep_until(nextTick);
    }



    uint8_t getBits(uint8_t c, int shift)
    {
        uint8_t sequence;
        sequence = (c >> shift) & 0x0F;

        return sequence;
    }



    uint32_t calcChecksum(uint32_t current_package)
    {
        uint32_t checksum = 0;
        for(uint32_t i = current_package*BYTE_PER_PACKAGE; i<current_package*BYTE_PER_PACKAGE+BYTE_PER_PACKAGE; i++) 
        {
            checksum += transmission_content[i];
        }

        checksum = ~checksum + 1;       //calc the two's complement of the sum

        return checksum;
    }



    std::vector<uint8_t> uint32ToByte(uint32_t value)
    {
        return 
        {
            static_cast<uint8_t>((value >> 24) & 0xFF), // Most significant byte
            static_cast<uint8_t>((value >> 16) & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>(value & 0xFF)          // Least significant byte
        };
    }

};