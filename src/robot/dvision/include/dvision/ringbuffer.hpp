#pragma once
#include <condition_variable>
#include <deque>
#include <mutex>
#include <vector>
#include <ros/ros.h>

/*
A simple one producer, one consumer ring buffer.

User side                             Worker side

Consume <-- Deqeque --  [ Queue 1 ] <-- Enqueue -- New buffer
|                                                 |
|                                                 |
Release -- Enqueue -->  [ Queue 2 ] -- Dequeue -->  Work ...
    Buffer

*/

// TODO(MWX): move this to dlib

namespace dvision {

template <typename T>
class RingBuffer {
    typedef std::deque<size_t> Queue;
public:
    RingBuffer();

    explicit RingBuffer(size_t ring_size, size_t timeout);

    void Init(size_t ring_size, size_t timeout);

    T& UserRequest();

    T& WorkerRequest();

    void UserRelease();

    void WorkerRelease();

private:
    T& request(size_t& index, Queue& queue, std::mutex& lock, std::condition_variable& cond);
    void release(size_t& index, Queue& queue, std::mutex& lock, std::condition_variable& cond);

private:
    Queue dataQueue_;
    Queue workspaceQueue_;

    std::mutex dataLock_;
    std::mutex workspaceLock_;

    std::condition_variable dataAvailable_;
    std::condition_variable workspaceAvailable_;

    std::vector<T> buffer_;
    size_t maxWaitTime_;

    size_t lastUserIndex_;
    size_t lastWorkerIndex_;
};

template <typename T>
RingBuffer<T>::RingBuffer() {}

template <typename T>
void RingBuffer<T>::Init(size_t ring_size, size_t timeout) {
    buffer_.resize(ring_size);
    maxWaitTime_ = timeout;
    lastUserIndex_ = UINT_MAX;
    lastWorkerIndex_ = UINT_MAX;
    workspaceQueue_.clear();
    for(size_t i = 0; i < buffer_.size(); ++i)
        workspaceQueue_.push_back(i);
}

template <typename T>
RingBuffer<T>::RingBuffer(size_t ring_size, size_t timeout)
{
    Init(ring_size, timeout);
}

template <typename T>
T& RingBuffer<T>::request(size_t &index, Queue &queue, std::mutex &lock, std::condition_variable &cond) {
    std::unique_lock<std::mutex> lk(lock);
    while(queue.empty()) {
        if(cond.wait_for(lk, std::chrono::milliseconds(maxWaitTime_)) == std::cv_status::timeout) {
            // FIXME(MWX): handle timeout
//            ROS_ERROR("Request timeout!");
            lk.unlock();
            return request(index, queue, lock, cond);
        }
    }

    index = queue.front();
    queue.pop_front();

    assert(index < buffer_.size());
    return buffer_.at(index);
}

template <typename T>
void RingBuffer<T>::release(size_t &index, Queue &queue, std::mutex &lock, std::condition_variable &cond) {
    if(index == UINT_MAX)
        return;

    std::lock_guard<std::mutex> lk(lock);
    queue.push_back(index);
    index = UINT_MAX;

    if(queue.size() == 1)
        cond.notify_one();
}

template <typename T>
T& RingBuffer<T>::UserRequest() {
    return request(lastUserIndex_, dataQueue_, dataLock_, dataAvailable_);
}

template <typename T>
void RingBuffer<T>::UserRelease() {
    release(lastUserIndex_, workspaceQueue_, workspaceLock_, workspaceAvailable_);
}

template <typename T>
T& RingBuffer<T>::WorkerRequest() {
    return request(lastWorkerIndex_, workspaceQueue_, workspaceLock_, workspaceAvailable_);
}

template <typename T>
void RingBuffer<T>::WorkerRelease() {
    release(lastWorkerIndex_, dataQueue_, dataLock_, dataAvailable_);
}


} // namespace dvision