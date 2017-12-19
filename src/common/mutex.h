/*
 * Copyright 2016 The sample_carto Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef SAMPLE_CARTO_COMMON_MUTEX_H_
#define SAMPLE_CARTO_COMMON_MUTEX_H_

#include <chrono>
#include <condition_variable>
#include <mutex>

namespace sample_carto {
namespace common {

// Defines an annotated mutex that can only be locked through its scoped locker
// implementation.
class  Mutex {
 public:
  // A RAII class that acquires a mutex in its constructor, and
  // releases it in its destructor. It also implements waiting functionality on
  // conditions that get checked whenever the mutex is released.
  class  Locker {
   public:
    Locker(Mutex* mutex): mutex_(mutex), lock_(mutex->mutex_) {}

    ~Locker() {
      lock_.unlock();
      mutex_->condition_.notify_all();
    }

    template <typename Predicate>
    void Await(Predicate predicate) {
      mutex_->condition_.wait(lock_, predicate);
    }

    template <typename Predicate>
    bool AwaitWithTimeout(Predicate predicate, double timeout)
    {
      auto millisec = std::chrono::microseconds(1);
      return mutex_->condition_.wait_for(lock_, millisec*timeout*1e6, predicate);
    }

  private:
    Mutex* mutex_;
    std::unique_lock<std::mutex> lock_;
  };

 private:
  std::condition_variable condition_;
  std::mutex mutex_;
};

using MutexLocker = Mutex::Locker;

}  // namespace common
}  // namespace sample_carto

#endif  // SAMPLE_CARTO_COMMON_MUTEX_H_
