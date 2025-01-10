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
#include <unordered_set>
#include <deque>



const uint32_t HEADER_SIZE = 16;
const uint32_t BYTE_PER_PACKAGE = 64;      //256 bytes usually
const uint32_t BYTE_BETWEEN_SYNC = 4;
const uint32_t SEND_DELAY = 60;



class TimedQueue 
{
private:
    std::queue<uint32_t> queue;
    std::unordered_set<uint32_t> set; // Auxiliary set to track unique values
    std::timed_mutex mutex;

public:
    bool push(uint32_t value) 
    {
        if (mutex.try_lock_for(std::chrono::milliseconds(100))) 
        {
            // Check if the value is already in the set (and therefore the queue)
            if (set.find(value) != set.end()) 
            {
                mutex.unlock();
                return false; // Value already exists, do not push
            }

            // Push the value to the queue and add it to the set
            queue.push(value);
            set.insert(value);
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
                uint32_t value = queue.front();
                queue.pop();
                set.erase(value); // Remove from set since it's no longer in the queue
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

            // Iterate over the queue and move elements to tempDeque, skipping the one to remove
            while (!queue.empty()) 
            {
                if (queue.front() == value && !found) 
                {
                    found = true; // Skip adding the value to tempDeque
                } 
                else 
                {
                    tempDeque.push_back(queue.front()); // Add all other values to tempDeque
                }
                queue.pop();
            }

            // Refill the original queue with the modified tempDeque
            for (auto val : tempDeque) 
            {
                queue.push(val);
                set.insert(val); // Insert back into the set
            }

            // If the value was found, remove it from the set
            if (found) 
            {
                set.erase(value);
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
