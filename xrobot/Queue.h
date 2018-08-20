/* Copyright (c) 2016 PaddlePaddle Authors. All Rights Reserve.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. */

#pragma once

#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>

using Millisecond = std::chrono::duration<int, std::ratio<1,1000>>;

//#include "Locks.h"


template <typename T>
class BlockingQueue {
public:
  /**
   * @brief Construct Function.
   * @param[in] capacity the max numer of elements the queue can have.
   */
  explicit BlockingQueue(size_t capacity) : capacity_(capacity), wait_time_(Millisecond(5000)) {}

  /**
   * @brief enqueue an element into Queue.
   * @param[in] x The enqueue element, pass by reference .
   * @note This method is thread-safe, and will wake up another thread
   * who was blocked because of the queue is empty.
   * @note If it's size() >= capacity before enqueue,
   * this method will block and wait until size() < capacity.
   */
  void enqueue(const T& x) {
    std::unique_lock<std::mutex> lock(mutex_);
    bool pred_is_true = notFull_.wait_for(lock, wait_time_, [&] { return queue_.size() < capacity_; });
      if (pred_is_true) {
          queue_.push_back(x);
          notEmpty_.notify_one();
      }
  }

  /**
   * Dequeue from a queue and return a element.
   * @note this method will be blocked until not empty.
   * @note this method will wake up another thread who was blocked because
   * of the queue is full.
   */
  bool dequeue(T& el) {
    std::unique_lock<std::mutex> lock(mutex_);
    bool pred_is_true = notEmpty_.wait_for(lock, wait_time_, [&] { return !queue_.empty(); });

      if (pred_is_true) {
          using std::swap;
          swap(queue_.front(), el);
          queue_.pop_front();
          notFull_.notify_one();
      }
      return pred_is_true;
  }

  /**
   * Return size of queue.
   *
   * @note This method is thread safe.
   * The size of the queue won't change until the method return.
   */
  size_t size() {
    std::lock_guard<std::mutex> guard(mutex_);
    return queue_.size();
  }

  /**
   * @brief is empty or not.
   * @return true if empty.
   * @note This method is thread safe.
   */
  size_t empty() {
    std::lock_guard<std::mutex> guard(mutex_);
    return queue_.empty();
  }

private:
  std::mutex mutex_;
  std::condition_variable notEmpty_;
  std::condition_variable notFull_;
  std::deque<T> queue_;
  size_t capacity_;
  Millisecond wait_time_;
};
