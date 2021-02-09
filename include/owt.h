#ifndef OWT_H
#define OWT_H


#include <chrono>
#include <deque>
#include <mutex>

// Translate from pts to system timestmap using the 
// minimum measured offset from pts to system timestamps
// The overloaded operator()(DurationType pts, DurationType sys) 
// updates and returns the offset from pts to system
template <typename DurationType>
class Owt
{
  struct PtsSysTime {
    DurationType pts;
    DurationType sys;
  };

  inline DurationType getOffset(const PtsSysTime & t) const {
    return t.sys - t.pts;
  }

  std::deque<PtsSysTime> deq_;
  DurationType window_size_;
  std::mutex mtx_;

public:
  Owt(DurationType window_size): window_size_(window_size) {}
  DurationType operator()(DurationType pts, DurationType sys)
  {
    PtsSysTime t{pts,sys};
    std::lock_guard<std::mutex> lck(mtx_);

    if (deq_.empty())
    {
      deq_.emplace_back(t);
      return sys;
    }

    while (deq_.size() && getOffset(deq_.back()) >= getOffset(t)) {
      deq_.pop_back();
    }
    deq_.emplace_back(t);

    DurationType offset = getOffset(deq_.front());

    if (t.pts - deq_.front().pts > window_size_) {
      deq_.pop_front();
    }
    
    return pts + offset;
  }
};

#endif // OWT_H