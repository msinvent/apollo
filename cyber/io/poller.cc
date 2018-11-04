/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "cyber/io/poller.h"

#include <fcntl.h>
#include <signal.h>
#include <string.h>
#include <sys/epoll.h>
#include <unistd.h>

#include "cyber/common/log.h"
#include "cyber/time/time.h"

namespace apollo {
namespace cyber {
namespace io {

using base::AtomicRWLock;
using base::ReadLockGuard;
using base::WriteLockGuard;

Poller::Poller() {
  if (!Init()) {
    AERROR << "Poller init failed!";
    Clear();
  }
}

Poller::~Poller() { Shutdown(); }

void Poller::Shutdown() {
  if (is_shutdown_.exchange(true)) {
    return;
  }
  Clear();
}

bool Poller::Register(const PollRequest& req) {
  if (is_shutdown_.load()) {
    return false;
  }

  PollCtrlParam ctrl_param;
  ctrl_param.fd = req.fd;
  ctrl_param.event.data.fd = req.fd;
  ctrl_param.event.events = req.events;

  {
    WriteLockGuard<AtomicRWLock> lck(requests_lock_);
    if (requests_.count(req.fd) == 0) {
      ctrl_param.operation = EPOLL_CTL_ADD;
      requests_[req.fd] = std::make_shared<PollRequest>();
    } else {
      ctrl_param.operation = EPOLL_CTL_MOD;
    }
    *requests_[req.fd] = req;
  }

  {
    ReadLockGuard<AtomicRWLock> lck(changes_lock_);
    changes_.push_back(ctrl_param);
  }

  Notify();
  return true;
}

bool Poller::Unregister(const PollRequest& req) {
  if (is_shutdown_.load()) {
    return false;
  }

  {
    WriteLockGuard<AtomicRWLock> lck(requests_lock_);
    auto size = requests_.erase(req.fd);
    if (size == 0) {
      AERROR << "unregister failed, can't find fd: " << req.fd;
      return false;
    }
  }

  PollCtrlParam ctrl_param;
  ctrl_param.operation = EPOLL_CTL_DEL;
  ctrl_param.fd = req.fd;

  {
    ReadLockGuard<AtomicRWLock> lck(changes_lock_);
    changes_.push_back(ctrl_param);
  }

  Notify();
  return true;
}

bool Poller::Init() {
  epoll_fd_ = epoll_create(kPollSize);
  if (epoll_fd_ < 0) {
    AERROR << "epoll create failed, " << strerror(errno);
    return false;
  }

  // create pipe, and set nonblock
  if (pipe(pipe_fd_) == -1) {
    AERROR << "create pipe failed, " << strerror(errno);
    return false;
  }
  if (fcntl(pipe_fd_[0], F_SETFL, O_NONBLOCK) == -1) {
    AERROR << "set nonblock failed, " << strerror(errno);
    return false;
  }
  if (fcntl(pipe_fd_[1], F_SETFL, O_NONBLOCK) == -1) {
    AERROR << "set nonblock failed, " << strerror(errno);
    return false;
  }

  // add pipe[0] to epoll
  auto request = std::make_shared<PollRequest>();
  request->fd = pipe_fd_[0];
  request->events = EPOLLIN;
  request->timeout_ms = -1;
  request->callback = [this](const PollResponse&) {
    char c = 0;
    while (read(pipe_fd_[0], &c, 1) > 0) {
    }
  };
  requests_[request->fd] = request;

  PollCtrlParam ctrl_param;
  ctrl_param.operation = EPOLL_CTL_ADD;
  ctrl_param.fd = pipe_fd_[0];
  ctrl_param.event.data.fd = pipe_fd_[0];
  ctrl_param.event.events = EPOLLIN;
  changes_.push_back(ctrl_param);

  thread_ = std::thread(&Poller::ThreadFunc, this);
  is_shutdown_.exchange(false);
  return true;
}

void Poller::Clear() {
  if (thread_.joinable()) {
    thread_.join();
  }

  if (epoll_fd_ >= 0) {
    close(epoll_fd_);
    epoll_fd_ = -1;
  }

  if (pipe_fd_[0] >= 0) {
    close(pipe_fd_[0]);
    pipe_fd_[0] = -1;
  }

  if (pipe_fd_[1] >= 0) {
    close(pipe_fd_[1]);
    pipe_fd_[1] = -1;
  }

  {
    WriteLockGuard<AtomicRWLock> lck(requests_lock_);
    requests_.clear();
  }

  {
    WriteLockGuard<AtomicRWLock> lck(changes_lock_);
    changes_.clear();
  }
}

void Poller::Poll(int timeout_ms) {
  epoll_event evt[kPollSize];
  auto before_time_ns = Time::Now().ToNanosecond();
  int ready_num = epoll_wait(epoll_fd_, evt, kPollSize, timeout_ms);
  auto after_time_ns = Time::Now().ToNanosecond();
  int interval_ms = (before_time_ns - after_time_ns) / 1000000;
  if (interval_ms == 0) {
    interval_ms = 1;
  }

  std::unordered_map<int, PollResponse> responses;
  {
    ReadLockGuard<AtomicRWLock> lck(requests_lock_);
    for (auto& item : requests_) {
      auto& request = item.second;
      if (request->timeout_ms > 0) {
        request->timeout_ms -= interval_ms;
        if (request->timeout_ms < 0) {
          request->timeout_ms = 0;
        }
      }

      if (request->timeout_ms == 0) {
        responses[item.first] = PollResponse();
      }
    }
  }

  if (ready_num > 0) {
    for (int i = 0; i < ready_num; ++i) {
      int fd = evt[i].data.fd;
      uint32_t events = evt[i].events;
      responses[fd] = PollResponse(events);
    }
  }

  for (auto& item : responses) {
    int fd = item.first;
    auto& response = item.second;

    ReadLockGuard<AtomicRWLock> lck(requests_lock_);
    auto search = requests_.find(fd);
    if (search != requests_.end()) {
      search->second->callback(response);
    }
  }

  if (ready_num < 0) {
    if (errno != EINTR) {
      AERROR << "epoll wait failed, " << strerror(errno);
    }
  }
}

void Poller::ThreadFunc() {
  // block all signals in this thread
  sigset_t signal_set;
  sigfillset(&signal_set);
  pthread_sigmask(SIG_BLOCK, &signal_set, NULL);

  while (!is_shutdown_.load()) {
    HandleChanges();
    int timeout_ms = GetTimeoutMs();
    ADEBUG << "this poll timeout ms: " << timeout_ms;
    Poll(timeout_ms);
  }
}

void Poller::HandleChanges() {
  ChangeList local_changes;
  {
    ReadLockGuard<AtomicRWLock> lck(changes_lock_);
    if (changes_.empty()) {
      return;
    }
    local_changes.swap(changes_);
  }

  for (auto& item : local_changes) {
    ADEBUG << "epoll ctl, op[" << item.operation << "] fd[" << item.fd
           << "] events[" << item.event.events << "]";
    if (epoll_ctl(epoll_fd_, item.operation, item.fd, &item.event) != 0 &&
        errno != EBADF) {
      AERROR << "epoll ctl failed, " << strerror(errno);
    }
  }
}

// we may use min heap to opt
int Poller::GetTimeoutMs() {
  int timeout_ms = kPollTimeoutMs;
  ReadLockGuard<AtomicRWLock> lck(requests_lock_);
  for (auto& item : requests_) {
    auto& req = item.second;
    if (req->timeout_ms > 0 && req->timeout_ms < timeout_ms) {
      timeout_ms = req->timeout_ms;
    }
  }
  return timeout_ms;
}

void Poller::Notify() {
  std::unique_lock<std::mutex> lock(pipe_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }

  char msg = 'C';
  if (write(pipe_fd_[1], &msg, 1) < 0) {
    AWARN << "notify failed, " << strerror(errno);
  }
}

}  // namespace io
}  // namespace cyber
}  // namespace apollo