#include "transmitter.cpp"
#include "receiver.cpp"
#include <cstring> // For strcmp

int main(int argc, char** argv)
{
    // bool list_mode = false;
    TimedQueue pending_ack;
    TimedQueue ack_queue;
    TimedQueue neg_ack_queue;
    std::atomic<bool> established(false);
    std::atomic<bool> listening(false);
    std::atomic<bool> partner_finished(false);
    std::mutex hardware_lock;
    hardware_lock.unlock();
    int mode = 0;

    if (argc > 1) {
        // Compare the arguments with strcmp for correct string comparison
        if (strcmp(argv[1], "-b15f") == 0) { 
            mode = 1; 
        }
        else if (strcmp(argv[1], "-ard") == 0) { 
            mode = 2; 
        }

        // If there's a third argument, check for "-l"
        // if (argc > 2 && strcmp(argv[2], "-l") == 0) { 
        //     list_mode = true;  // Start listening-only mode
        // }
    }

    if(mode == 0)
    {return -1;}

    boost::asio::io_context io;
    boost::asio::serial_port serial(io);  // Declare serial port outside switch
    boost::asio::serial_port* serial_ptr = nullptr;
    B15F* b15f_ptr = nullptr;

    // Initialize B15F and serial outside of the switch block to avoid jumping across initialization
    if (mode == 1) 
    {
        B15F& b15f = B15F::getInstance();
        b15f_ptr = &b15f;
        b15f_ptr->setRegister(&DDRA, 0x0F);  // Assuming DDRA is defined elsewhere
    } 
    else if (mode == 2) 
    {
        serial_ptr = &serial;
        serial_ptr->open("/dev/ttyUSB0");  // Open serial port
        serial_ptr->set_option(boost::asio::serial_port_base::baud_rate(9600));
        serial_ptr->set_option(boost::asio::serial_port_base::character_size(8));
        serial_ptr->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_ptr->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
        serial_ptr->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
    }

    // std::cout << "Program starting..." << std::endl;

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));  // Small delay before starting transmission

    // Create Receiver and Transmitter instances with the appropriate pointers
    Receiver r(b15f_ptr, serial_ptr, pending_ack, ack_queue, neg_ack_queue, established, listening, partner_finished, hardware_lock, mode);
    std::thread receiver_thread(&Receiver::beginListening, &r);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));  // Small delay before starting transmission

    // std::cout << "Starting transmission..." << std::endl;

    Transmitter t(b15f_ptr, serial_ptr, pending_ack, ack_queue, neg_ack_queue, established, listening, partner_finished, hardware_lock, mode);
    std::thread transmitter_thread(&Transmitter::beginTransmission, &t);

    // Join both threads before finishing
    // receiver_thread.join();
    transmitter_thread.join();

    return 0;
}