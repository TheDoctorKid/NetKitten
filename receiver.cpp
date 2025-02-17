#include "netkitten.cpp"



class Receiver
{
    private:
    B15F* b15f;
    boost::asio::serial_port* serial;
    TimedQueue & pending_ack;           //stores the sequence_num's which havn't been acknowledged yet
    TimedQueue & ack_queue;             //stores the acknowledgments which still need to be sent
    TimedQueue & neg_ack_queue;         //stores negative acknowledgments which need to be sent
    std::atomic<bool> & established;              //is sent data received
    std::atomic<bool> & listening;                //is own receiver currently reading data
    std::atomic<bool> & partner_finished;         //does other client finished transmission
    std::mutex & hardware_lock;
    int mode = 0;

    unsigned short currentState;
    std::vector<uint8_t> read_buffer;                //reads tetra bits in order which they arrived
    std::vector<uint8_t> transmission_content;       //perceived transmission content


    public:
    Receiver(B15F* board, boost::asio::serial_port* ser, TimedQueue & pen_ack, TimedQueue & ack_q, TimedQueue & neg_ack_q, std::atomic<bool> & es, std::atomic<bool> & li, std::atomic<bool> & pf, std::mutex & hl, int mo)
        : b15f(board), serial(ser), pending_ack(pen_ack), ack_queue(ack_q), neg_ack_queue(neg_ack_q), established(es), listening(li), partner_finished(pf), hardware_lock(hl), mode(mo)
    {

    }



    void beginListening()       //switch states of listening
    {
        currentState = 1;                   //start in sync state

        while(currentState != 0)            //receiving main loop
        {
            switch (currentState)           //1=sync, 2=switch await, 3=reading
            {
            case 1:
                // std::cout << "Receiver Sync" << std::endl;
                syncListen();
                continue;

            case 2:
                if(partner_finished.load())
                {
                    currentState = 3;               //go back to listening for transmission was already stored
                    continue;
                }
                // std::cout << "Received Content: ";
                transmission_content.erase(transmission_content.end() - transmission_content.back(), transmission_content.end());        //output all bytes from the transmission
                for(uint8_t byte:transmission_content)
                {
                    std::cout.write(reinterpret_cast<const char*>(&byte), 1);
                }
                std::cout.flush();
                // std::cout << "Partner Finished with EOT" << std::endl;
                partner_finished.store(true);   //store that transmission was fully received
                currentState = 3;               //go back to listening in another transmission is sent
                continue;

            case 3:
                // std::cout << "Receiving Comms" << std::endl;
                receiveTransmission();
                continue;
            
            default:
                // std::cout << "A fatal error occured while receiving the message!" << std::endl;
                break;
            }
        }
        return;
    }



    void syncListen()
    {
        while(!(established.load() && listening.load()))
        {
            read_buffer.clear();
            awaitSwitch();      //try to catch falling edge
            
            for(uint32_t i = 0; i < BYTE_BETWEEN_SYNC; i++)         
            {
                read_buffer.push_back(readTetraPack());
            }

            switch(determineCase())
            {
            case 1:             //buffer was full with SYNC
                listening.store(true);
                established.store(false);
                read_buffer.clear();
                continue;

            case 2:             //buffer was full with ACK
                listening.store(true);
                established.store(true);
                read_buffer.clear();
                currentState = 3;           //everything synced up and now reading data
                return;

            case 3:             //buffer didnt match a mask
                read_buffer.clear();
                continue;
            }
        }
    }



    int determineCase() 
    {
        bool is_sync = true; 
        bool is_ack = true;  

        for (size_t i = 1; i < read_buffer.size(); ++i) {
            if (i % 2 == 0) {
                if (read_buffer[i] != 0b0110) {
                    return 3;  
                }
            } else {
                if (read_buffer[i] != 0b0001) {
                    is_sync = false;  // Fails SYNC pattern
                }
                if (read_buffer[i] != 0x00) {
                    is_ack = false;  // Fails ACK pattern
                }
            }
        }

        if (is_sync) {
            return 1;  // SYNC case
        } 
        if (is_ack) {
            return 2;  // ACK case
        }
        return 3;  // General failure case
    }




    void awaitSwitch() 
    {
        uint8_t prevByte = 0xFF; // Initialize to an invalid value
        uint8_t currentByte;

        while (true) 
        {
            currentByte = fastReadTetraPack();

            if (prevByte == 0x0F && currentByte == 0x00) 
            {
                //exit on falling edge
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                return;
            } 
            else
            {
                prevByte = currentByte;
                continue;
            }
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
                    incoming = (incoming >> 4);
                    hardware_lock.unlock();
                    std::this_thread::sleep_until(nextTick);
                    return (incoming);
                    break;
                
                case 2:
                    // Sende Kommando
                    const char command = 'R';
                    boost::asio::write(*serial, boost::asio::buffer(&command, 1));
                    boost::system::error_code error;
                    boost::asio::read(*serial, boost::asio::buffer(&incoming, 1), error);
                    if (error) {
                        // std::cout << "Error: " << error.message() << std::endl;
                    }
                    hardware_lock.unlock();
                    std::this_thread::sleep_until(nextTick);
                    return (incoming);
                    break;
                }
            }
        } 
    }



    uint8_t fastReadTetraPack()
    {
        using namespace std::chrono;
        auto now = steady_clock::now();
        auto nextTick = now + milliseconds(10);
        uint8_t incoming = 0;

        while(true) 
        {
            if(hardware_lock.try_lock())
            {
                switch (mode)
                {
                case 1:
                    incoming = b15f->getMem8(&PINA);        //read memory from PINA
                    incoming = (incoming >> 4);
                    hardware_lock.unlock();
                    std::this_thread::sleep_until(nextTick);
                    return (incoming);
                    break;
                
                case 2:
                    // Sende Kommando
                    const char command = 'R';
                    boost::asio::write(*serial, boost::asio::buffer(&command, 1));
                    boost::system::error_code error;
                    boost::asio::read(*serial, boost::asio::buffer(&incoming, 1), error);
                    if (error) {
                        // std::cout << "Error: " << error.message() << std::endl;
                    }
                    hardware_lock.unlock();
                    std::this_thread::sleep_until(nextTick);
                    return (incoming);
                    break;
                }
            }
        } 
    }



    uint8_t combine4BitValues(uint8_t first, uint8_t second) 
    {
        first &= 0x0F;  
        second &= 0x0F; 
        return (first << 4) | second;
    }



    void receiveTransmission()
    {
        awaitSwitch();
        readTetraPack();        //first read gets scrapped because its only for syncing purpouses
        
        for(uint32_t i = 0; i < BYTE_BETWEEN_SYNC; i++)      //reads BYTE_BETWEEN_SYNC amounts of bytes
        {
            uint8_t first = readTetraPack();
            uint8_t second = readTetraPack();
            read_buffer.push_back(combine4BitValues(first, second));
        }

        if(read_buffer.size() == BYTE_BETWEEN_SYNC && read_buffer.front() != 0x01)
        {
            std::vector<uint8_t> ref(BYTE_BETWEEN_SYNC, 22);

            if(read_buffer == ref)
            {
                read_buffer.clear();
                listening.store(false);
                established.store(false);
                currentState = 1;       //other client needs resync
                return;
            }

            else
            {
                read_buffer.clear();    //indirect check if there is still sync artifacts in buffer
                return;
            }
        }

        if(read_buffer.size() == BYTE_PER_PACKAGE + HEADER_SIZE)
        {
            // std::cout << "ack queue size " << ack_queue.size() <<std::endl;
            // for(uint8_t byte : read_buffer)        //output all bytes from the transmission
            // {
            //     std::cout << int(byte);
            // }

            if(!checkPattern())            
            {
                // std::cout << "Pattern not recognised" << std::endl;
                currentState = 1;   //if pattern wasnt recognised go back to sync state
                listening.store(false);
                established.store(false);
                read_buffer.clear();
                // std::cout << std::endl;
                return;
            }
            // std::cout << "Pattern recognised, Data stored!" << std::endl;
            read_buffer.clear();
        }
        return;
    }



    bool checkPattern() 
    {
        std::vector<uint8_t> eot_reference(HEADER_SIZE + BYTE_PER_PACKAGE, 0x04);
        eot_reference.front() = 0x01;       //start of header
        eot_reference.back() = 0x03;        //end of text
        if(read_buffer == eot_reference)        //receiver noticed other client's end of transmission
        {
            currentState = 2;               //write received message into output
            return true;
        }

        if (read_buffer[0] != 0x01) 
        {
            // std::cout << "pos 1" <<std::endl;
            return false;
        }

        uint32_t received_package_sequence = (read_buffer[1] << 24) | (read_buffer[2] << 16) | (read_buffer[3] << 8) | read_buffer[4];

        if (read_buffer[5] != 0x06 && read_buffer[5] != 0x15) 
        {
            // std::cout << "pos 2" <<std::endl;
            return false;
        }

        uint32_t acknowledged_sequence = (read_buffer[6] << 24) | (read_buffer[7] << 16) | (read_buffer[8] << 8) | read_buffer[9];

        if (read_buffer[10] != 0x16 || read_buffer[11] != 0x16) 
        {
            // std::cout << "pos 3" <<std::endl;
            return false;
        }

        uint16_t checksum = (read_buffer[12] << 8) | (read_buffer[13]);

        if (read_buffer[14] != 0x02) 
        {
            // std::cout << "pos 4" <<std::endl;
            return false;
        }

        if (read_buffer[read_buffer.size()-1] != 0x03) 
        {
            // std::cout << "pos 5" <<std::endl;
            return false;
        }

        if(!checkChecksum(checksum))
        {
            neg_ack_queue.push(received_package_sequence);      //the pattern matched but checksum was wrong
            // std::cout << "pos 6" <<std::endl;
            return false;
        }

        //package is valid!
        if(read_buffer[5] == 0x06)
        {
            if (acknowledged_sequence == uint32_t(~0))
            {
                // std::cout << "Prevented removal of ~0" << std::endl;
            }

            else
            {
                // std::cout << "removed " << acknowledged_sequence << std::endl;
                if(pending_ack.remove(acknowledged_sequence))              //tell transmitter to not wait for package he sent anymore with sequence number ACKNOWLEDGED SEQUENCE
                {
                    // std::cout << "Successfully " << pending_ack.size() << std::endl;
                }
            }
        }

        if (received_package_sequence == uint32_t(~0))
        {
            // std::cout << "Scrapped ~0 Package" << std::endl;
            return true;
        }

        else
        {ack_queue.push(received_package_sequence);}        //tell transmitter to acknowledge this package
                      
        if (transmission_content.size() < (received_package_sequence+1) * BYTE_PER_PACKAGE)
        {
            transmission_content.resize((received_package_sequence+1) * BYTE_PER_PACKAGE);
        }

        auto insert_pos = transmission_content.begin() + received_package_sequence * BYTE_PER_PACKAGE;
        std::copy(read_buffer.begin() + HEADER_SIZE - 1, read_buffer.end() - 1, insert_pos);

        return true;
    }



    bool checkChecksum(uint16_t received_checksum)           //compares checksum received with calculated from package 
    {
        uint16_t checksum = 0;
        for(uint32_t i = HEADER_SIZE-1; i<HEADER_SIZE+BYTE_PER_PACKAGE-1; i++) 
        {
            checksum += read_buffer[i];
        }
        // std::cout << "checksum: " << int(checksum) << std::endl;
        return checksum == received_checksum;
    }

};