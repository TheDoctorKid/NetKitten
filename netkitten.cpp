#pragma once
#include <iostream>
#include <thread>
#include <fstream>
#include <vector>
#include <array>
#include <unistd.h>
#include <string>
#include <algorithm>
#include <queue>
#include <chrono>
#include <atomic>
#include <optional>
#include <mutex>
#include <boost/asio.hpp>
#include <b15f/b15f.h>



const uint32_t HEADER_SIZE = 16;
const uint32_t BYTE_PER_PACKAGE = 256;
const uint32_t BYTE_BETWEEN_SYNC = 4;
const uint32_t SEND_DELAY = 60;


class TimedQueue 
{
private:
    std::queue<uint32_t> queue;
    std::timed_mutex mutex;

public:
    bool push(uint32_t value) 
    {
        if (mutex.try_lock_for(std::chrono::milliseconds(100))) 
        {
            queue.push(value);
            mutex.unlock();
            return true; // Successfully pushed
        } 
        else 
        {
            return false; // Timed out
        }
    }

    std::optional<uint32_t> pop() 
    {
        if (mutex.try_lock_for(std::chrono::milliseconds(100))) 
        {
            if (!queue.empty()) 
            {
                int value = queue.front();
                queue.pop();
                mutex.unlock();
                return value; // Successfully popped
            } 
            else 
            {
                mutex.unlock();
                return std::nullopt; // Queue was empty
            }
        } 
        else 
        {
            return std::nullopt; // Timed out
        }
    }

    bool remove(uint32_t value) 
    {
        if (mutex.try_lock_for(std::chrono::milliseconds(100))) 
        {
            std::deque<uint32_t> tempDeque;

            bool found = false;
            while (!queue.empty()) {
                if (queue.front() == value && !found) {
                    found = true;
                } 
                else 
                {
                    tempDeque.push_back(queue.front());
                }
                queue.pop();
            }

            for (int val : tempDeque) {
                queue.push(val);
            }

            mutex.unlock();
            return found;
        } 
        else 
        {
            return false; // Timed out
        }
    }

    bool empty()
    {
        return queue.empty();
    }

    int size()
    {
        return queue.size();
    }

    uint32_t front()
    {
        return queue.front();
    }
};